import cv2
import numpy as np
import utlis
from picamera.array import PiRGBArray
from picamera import PiCamera
from picarx import Picarx
import time
import os


def getLaneCurve(img, curveList = [], display=2):
    
    avgVal = 2
    
    imgCopy = img.copy()
    imgResult = img.copy()
    #### STEP 1
    imgThres = utlis.thresholding(img)

    #### STEP 2
    hT, wT, c = img.shape
    points = utlis.valTrackbars()
    imgWarp = utlis.warpImg(imgThres, points, wT, hT)
    imgWarpPoints = utlis.drawPoints(imgCopy, points)

    #### STEP 3
    # whole image
    curveAveragePoint, imgHist = utlis.getHistogram(imgWarp, display=True, minPer=0.9)
    # at some reference point
    refer_point = 6
    middlePoint, imgHist = utlis.getHistogram(imgWarp, display=True, minPer=0.5, region=refer_point)
    
    curveRaw = curveAveragePoint - middlePoint

    #### SETP 4
    curveList.append(curveRaw)
    if len(curveList) > avgVal:
        curveList.pop(0)
    # average of curve values with size avgVal (= 10)
    curve = int(sum(curveList) / len(curveList))

    #### STEP 5
    if display != 0:

        #### drawing the reference line for middlePoint
        reference_line = imgWarp.shape[0] // refer_point
        cv2.line(imgWarp, (0, imgWarp.shape[0] - reference_line), (imgWarp.shape[1], imgWarp.shape[0] - reference_line),
                 (0, 0, 255), 2)

        imgInvWarp = utlis.warpImg(imgWarp, points, wT, hT, inv=True)
        imgInvWarp = cv2.cvtColor(imgInvWarp, cv2.COLOR_GRAY2BGR)
        imgInvWarp[0:hT // 3, 0:wT] = 0, 0, 0
        imgLaneColor = np.zeros_like(img)
        imgLaneColor[:] = 0, 255, 0
        imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)
        imgResult = cv2.addWeighted(imgResult, 1, imgLaneColor, 1, 0)
        midY = 450
        cv2.putText(imgResult, str(curve), (wT // 2 - 80, 85), cv2.FONT_HERSHEY_COMPLEX, 2, (255, 0, 255), 3)
        cv2.line(imgResult, (wT // 2, midY), (wT // 2 + (curve * 3), midY), (255, 0, 255), 5)
        cv2.line(imgResult, ((wT // 2 + (curve * 3)), midY - 25), (wT // 2 + (curve * 3), midY + 25), (0, 255, 0), 5)
        for x in range(-30, 30):
            w = wT // 20
            cv2.line(imgResult, (w * x + int(curve // 50), midY - 10),
                     (w * x + int(curve // 50), midY + 10), (0, 0, 255), 2)
        # fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
        # cv2.putText(imgResult, 'FPS ' + str(int(fps)), (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (230, 50, 50), 3);
    if display == 2:
        imgStacked = utlis.stackImages(0.7, ([img, imgWarpPoints, imgWarp],
                                             [imgHist, imgLaneColor, imgResult]))
        cv2.imshow('ImageStack', imgStacked)
        
    elif display == 1:
        cv2.imshow('Resutlt', imgResult)

    #### NORMALIZATION
#     curve = curve
#     if curve > 100: curve == 100
#     if curve < -100: curve == -100

    return curve, curveList, imgResult, imgWarp


def Find_steer(curveVal):
    
    sensitivity = 0.34
    if(curveVal > 0):
        #sensitivity = 10
        if(curveVal < 0.05):
                curveVal = 0
    else:
        if(curveVal > -0.05):
                curveVal = 0
    steer_angle = curveVal*sensitivity
    
    # if steer_angle is beyond the limit then steer upto the last limit
    if steer_angle < -25:
        steer_angle = -25
    elif steer_angle > 25:
        steer_angle = 25
    
    return steer_angle

# def Find_Obstacle():
#     distance = round(px.ultrasonic.read(), 2)
#     print("distance: ",distance)
#     if distance <= 15 and distance >=0:
#         print("Danger Distance")
#         px.forward(0)
#         px.set_dir_servo_angle(0)
#         time.sleep(0.1)
#         px.backward(1)
#         time.sleep(0.3)
#         px.forward(0)
#         time.sleep(0.1)
#         #rawCapture.truncate(0)   # Release cache
    



def Find_steer2(curveVal):
    
    #sensitivity = 0.34
    sensitivity = 0.55
    turn = 1 # 1 = right, -1 = left
    if(curveVal >= 0):
        turn = 1
    else:
        turn = -1
        
        
    if(curveVal > 0):
        #sensitivity = 10
        if(curveVal < 10):
                curveVal = 0
    else:
        if(curveVal > -10):
                curveVal = 0
                
    if(abs(curveVal) < 90):
        steer_angle = curveVal*sensitivity
        
    elif(abs(curveVal) < 100):
        steer_angle = 18 * turn
    elif(abs(curveVal) < 110):
        steer_angle = 20 * turn
    elif(abs(curveVal) < 120):
        steer_angle = 22 * turn
    else:
        steer_angle = 25 * turn
    
    
    # if steer_angle is beyond the limit then steer upto the last limit
    if steer_angle < -25:
        steer_angle = -25
    elif steer_angle > 25:
        steer_angle = 25
    
    return steer_angle
        

def main():
    curveList = []
    
    #intialTrackBarVals = [0, 155, 0, 240]
    intialTrackBarVals = [0, 122, 0, 240]
    utlis.initializeTrackbars(intialTrackBarVals)
    
    px = Picarx()
    
    # to record
    # Get path to the current working directory
    directory_path = '/home/pi/Car/Lane_recording'
    # number of files in recordedVideos
    files = os.listdir(directory_path)
    file_count = str(len(files) // 2 + 1)

    # defining the output video file formats, resolutions and frame rate
    fourcc_1 = cv2.VideoWriter_fourcc(*'MP4V')
    fourcc_2 = cv2.VideoWriter_fourcc(*'MP4V')
    out_1 = cv2.VideoWriter("Lane_recording/original" + file_count + ".avi", fourcc_1, 24.0, (640,480))
    out_2 = cv2.VideoWriter("Lane_recording/result" + file_count + ".avi", fourcc_2, 24.0, (960,240))

    with PiCamera() as camera:
        camera.exposure_compensation = 3
        camera.brightness = 65
        camera.resolution = (640,480)
        camera.framerate = 24
        rawCapture = PiRGBArray(camera, size=camera.resolution)  
        time.sleep(0.1)
        count=0
        steer_angle_ar = [0]*10
        actual_steer_angle =0
        temp_steer_angle =0
        #obstacle_detected = False

        for frame in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):# use_video_port=True            
            Img = frame.array
            img = cv2.resize(Img, (480, 240))
            
            curve, curveList, imgResult, imgWarp = getLaneCurve(img,curveList, display=2)
             #if(curve > 90) or (curve < -90):
                #time.sleep(0.1)
            steer_angle = Find_steer(curve)
            px.set_dir_servo_angle(steer_angle)
            print("steer_angle: ",steer_angle)
             
            steer_angle_ar.append(steer_angle)
            if len(steer_angle_ar)>10 :
                steer_angle_ar.pop(0)
            
            #### obstacle
            distance = round(px.ultrasonic.read(), 2) 
            print("distance: ",distance)
            if distance <= 15 and distance >=0:
                print("Danger Distance")
                if (count == 0) :
                    temp_steer_angle = steer_angle_ar[-6]
                    count+=1    
                px.forward(0)
                px.set_dir_servo_angle(0)
                time.sleep(0.1)
                px.backward(1)
                time.sleep(0.7)
                px.forward(0)
                time.sleep(0.1)
                if (temp_steer_angle == 0) :
                    actual_steer_angle = 10
                else :
                    actual_steer_angle = 1.5 * temp_steer_angle
                    
                if ( actual_steer_angle>=25):
                    actual_steer_angle = 25
                elif (actual_steer_angle <= -25):
                    actual_steer_angle =-25
                px.set_dir_servo_angle(actual_steer_angle)
                px.forward(1)
                time.sleep(0.1)
                #counter += 1
                rawCapture.truncate(0)   # Release cache
                continue
#             elif distance <= 15 and distance >=0 and count>3:
#                 count=0
#                 px.set_dir_servo_angle(temp_steer_angle)
#                 px.backward(1)
#                 time.sleep(0.7)
#                 px.forward(0)
#                 time.sleep(0.1)
#                 rawCapture.truncate(0)   # Release cache
#                 continue
#             elif distance <=40 and distance > 15 :
#                 px.forward(5)
                   
            ########## move the car  ################ 
            #if curve > 50 or curve <-50:
            if distance <=40 and distance > 15 :
                px.forward(3)
            else:
                px.forward(27)
                count=0
           
            #counter = 0    
            # print("steer angle = ", steer_angle)
            # print("curve", curve)
            
            # to record
            Result_mask_stack = utlis.stackImages(1, ([imgResult, imgWarp]))
            out_1.write(Img)
            out_2.write(Result_mask_stack)
        
            rawCapture.truncate(0)   # Release cache
            
            #cv2.imshow('Vid', img)
            key = cv2.waitKey(1)
            if key == 27: # 27 is ASCII value for ESC
                break
            
    print("Exitted!!!!")
    # setting steering angle to zero
    px.forward(0) 
    px.set_dir_servo_angle(0)
    # release
    out_1.release()
    out_2.release()
    # destroy all windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    
