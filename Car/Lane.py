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

    if display == 2:
        imgStacked = utlis.stackImages(0.7, ([img, imgWarpPoints, imgWarp],
                                             [imgHist, imgLaneColor, imgResult]))
        cv2.imshow('ImageStack', imgStacked)
        
    elif display == 1:
        cv2.imshow('Resutlt', imgResult)


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
        

def main():
    curveList = []
    
    #intialTrackBarVals = [0, 155, 0, 240]
    intialTrackBarVals = [0, 122, 0, 240]
    utlis.initializeTrackbars(intialTrackBarVals)
    
    px = Picarx()

    with PiCamera() as camera:
        camera.exposure_compensation = 3
        camera.brightness = 65
        camera.resolution = (640,480)
        camera.framerate = 24
        rawCapture = PiRGBArray(camera, size=camera.resolution)  
        time.sleep(0.1)


        for frame in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):# use_video_port=True            
            Img = frame.array
            img = cv2.resize(Img, (480, 240))
            
            curve, curveList, imgResult, imgWarp = getLaneCurve(img,curveList, display=0)

            steer_angle = Find_steer(curve)
            px.set_dir_servo_angle(steer_angle)
            print("steer_angle: ",steer_angle)
            
            #### obstacle
            distance = round(px.ultrasonic.read(), 2) 
            print("distance: ",distance)
            if distance <= 15 and distance >=0:
                print("Danger Distance")  
                px.forward(0)
                px.set_dir_servo_angle(0)
                time.sleep(0.1)
                px.backward(1)
                time.sleep(0.9)
                px.forward(0)
                
                rawCapture.truncate(0)   # Release cache
                continue
                   
            ########## move the car  ################ 
            #if curve > 50 or curve <-50:
            if distance <=40 and distance > 15 :
                px.forward(3)
            else:
                px.forward(35)
        
            rawCapture.truncate(0)   # Release cache
            
            #cv2.imshow('Vid', img)
            key = cv2.waitKey(1)
            if key == 27: # 27 is ASCII value for ESC
                break
            
    print("Exitted!!!!")
    # setting steering angle to zero
    px.forward(0) 
    px.set_dir_servo_angle(0)
    # destroy all windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    
