from picamera2 import Picamera2
import time
import cv2
import numpy as np
import utlis

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..')) # including parent directory for importing
import sensor
from drive import drive
from steer import Servo
import acc


def getLaneCurve(img, curveList = [], display=2):
    
    avgVal = 2
    
    imgCopy = img.copy()
    imgResult = img.copy()
    #### STEP 1
    imgThres = utlis.thresholding(img)
#     print("line15")
    #### STEP 2
    hT, wT, c = img.shape
    points = utlis.valTrackbars()
    imgWarp = utlis.warpImg(imgThres, points, wT, hT)
    imgWarpPoints = utlis.drawPoints(imgCopy, points)
#     print("line21")
    
    #### STEP 3
    # whole image
#     curveAveragePoint, imgHist = utlis.getHistogram(imgWarp, display=False, minPer=0.9)
    curveAveragePoint = utlis.getHistogram(imgWarp, display=False, minPer=0.9)
    # at some reference point
    refer_point = 6
#     middlePoint, imgHist = utlis.getHistogram(imgWarp, display=False, minPer=0.5, region=refer_point)
    middlePoint = utlis.getHistogram(imgWarp, display=False, minPer=0.5, region=refer_point)
    
    curveRaw = curveAveragePoint - middlePoint

    #### SETP 4
    curveList.append(curveRaw)
    if len(curveList) > avgVal:
        curveList.pop(0)
    # average of curve values with size avgVal (= 10)
    curve = int(sum(curveList) / len(curveList))
#     print("line37")

    #### STEP 5
    if display != 0:
#         print("line41")
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
#         imgStacked = utlis.stackImages(0.7, ([img, imgWarpPoints, imgWarp],
#                                              [imgHist, imgLaneColor, imgResult]))

        imgStacked = utlis.stackImages(0.7, ([img, imgWarpPoints, imgWarp],
                                             [imgLaneColor, imgResult]))
        
        cv2.imshow('ImageStack', imgStacked)
        
    elif display == 1:
        cv2.imshow('Resutlt', imgResult)


    return curve, curveList, imgResult, imgWarp


def Find_steer(curveVal):
    sensitivity = 0.34 #0.65 #0.34
#     sensitivity = 1.5
#     if(curveVal > 0):
#         if(curveVal < 0.05):
#             curveVal = 0
#     else:
#         if(curveVal < -0.05):
#             curveVal = 0
            
    steer_angle = curveVal * sensitivity
#     print("In function steer_angle", steer_angle)
#     print("In function curveVal", curveVal)
    
    # if steer angle beyond the limit
    if steer_angle < -40:
        steer_angle = -40
    elif steer_angle  > 40:
        steer_angle = 40
        
    return steer_angle
    
    
    
# picam2 = Picamera2()
# #config = picam2.create_still_configuration(main= {"size": (4056, 3040)}, lores = {"size": (480, 320)}, format=, display = "lores", buffer_count = 1, queue = False)
# config = picam2.create_video_configuration(main = {"size":(640,480), "format":"RGB888"}, raw = {"size":(640,480)})
# picam2.configure(config)
# 
# # picam2.set_controls({"ExposureTime": 10000, "AnalogueGain": 5}) #Shutter time and analogue signal boost
# picam2.start()
# # 
# # time.sleep(5)  #enjoy the preview
# 
# # t_0 = time.monotonic()
# while True:
#     img = picam2.capture_array() #this takes a picture. img can be used with cv2
#     cv2.imshow("frame", img)
#     
#     curve, curveList, imgResult, imgWarp = getLaneCurve(img,curveList, display=0)
#     
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
# 
# picam2.close()
# cv2.destroyAllWindows()


def main():
    
    acc_safe_distance = 20 #set the desired distance to maintain from the vehicle ahead
    acc_speed = 50
    buffer_distance = 5 #tunable
    delta_speed = 5
  
    
    pTime = 0
    cTime = 0

    fps_startTime = time.time()
    fps_counter = 0
    fps = 0
    
    curveList = []
    intialTrackBarVals = [0, 195, 0, 240]
    utlis.initializeTrackbars(intialTrackBarVals)
#     print("line 104")
    picam2 = Picamera2()
    #config = picam2.create_still_configuration(main= {"size": (4056, 3040)}, lores = {"size": (480, 320)}, format=, display = "lores", buffer_count = 1, queue = False)
    config = picam2.create_video_configuration(main = { "format":"RGB888"}, raw = {"size":(4608,2592)}, controls={"FrameRate":80})
    picam2.configure(config)

    # picam2.set_controls({"ExposureTime": 10000, "AnalogueGain": 5}) #Shutter time and analogue signal boost
    picam2.start()
    # 
    # time.sleep(5)  #enjoy the preview

    # t_0 = time.monotonic()
    
#     bot_drive.forward(5)
    
    ###saving image
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('output.mp4', fourcc, 20, 0, (480, 240))
    
    
    while True:
        img = picam2.capture_array() #this takes a picture. img can be used with cv2
        fps_counter += 1
    
        if time.time() - fps_startTime >= 1:
            fps = fps_counter / (time.time() - fps_startTime)
    
    #displaying fps in frame
        cv2.putText(img, str(int(fps)), (10,70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
#         cv2.imshow("frame", img)
#         print("line119")
        img = cv2.resize(img, (480, 240))
        cv2.imshow("resized frame", img)
#         print("line121")
        
        curve, curveList, imgResult, imgWarp = getLaneCurve(img,curveList, display=1)
        
            
        cv2.imshow("imgWarp", imgWarp)
#         print("Curve value = ", curve)
        
        steer_angle = Find_steer(curve)
        print("steer_angle", steer_angle)
        servo.set_SteeringAngle(steer_angle)
        
        ### USS
        distance = uss.getDistance('front')
        bot_drive.forward(15) #15
        #speed from encoder needs to be read her*****
        bot_speed = bot_drive.get_veh_speed()
        print("bot_speed",bot_speed)
        
        ## we are near object
        if(distance <= 20 and distance >=0):
            print("Danger Distance")
            bot_drive.forward(0)
            servo.set_SteeringAngle(0)
            
            time.sleep(0.1)
            bot_drive.reverse(5)
            time.sleep(0.9)
            bot_drive.forward(0)
        
        ## reducing the speed
        if(distance <= 40 and distance >=20):
            #Drive forward
            print("Driving forward")
            if distance > (acc_safe_distance+buffer_distance):
                #Vehicle is far from the vehicle stand, increase speed
                print("Vehicle is far from the vehicle stand, increasing the speed")
                if(bot_speed < acc_speed):
                    print("increasing the speed",(bot_speed+delta_speed))
                    bot_drive.forward(bot_speed+delta_speed)
                else:
                    print("decreasing the speed if reaches more than max speed",(bot_speed-delta_speed))
                    bot_drive.forward(bot_speed-delta_speed)
            elif distance < (acc_safe_distance-buffer_distance):
                #Vehicle is close to the vehicle ahead, reduce speed relative to front distance
                print("Vehicle is close to the vehicle ahead, reducing speed relative to front distance",((distance/acc_safe_distance)*acc_speed))
                bot_drive.forward((distance/acc_safe_distance)*acc_speed)
            
#             print("Danger Distance")
#             bot_drive.forward(5)
#         else:
#             bot_drive.forward(10)
        
        out.write(imgResult)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    picam2.close()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        bot_drive = drive(1) #single motor drive
        uss = sensor.uss.uss()
        servo = Servo()
        
        #check for green light
        servo.set_PanAngle(-20)
#         sensor.camera.color_detect_green.main()
        
        #green is detected
        servo.set_PanAngle(20)
                                
        servo.set_SteeringAngle(0)
        
        main()
    except KeyboardInterrupt:
        print("Program terminated by user")

    finally:
        # Clean up GPIO
        del bot_drive
        del uss
        del servo
        cv2.destroyAllWindows()
    
    
# t_1 = time.monotonic()
# picam2.close() #when you're done taking photos, this closes the camera connection

# print("taking the photo took %s seconds", round(t_1-t_0, 3))
# print("width height\t:", *img.shape[0:2][::-1])
# 
# # cv2.imshow("Title", cv2.resize(img, (0,0), fx=0.25, fy=0.25)) #resize the image to a quarter of the original size
# cv2.imshow("frame", img)
# cv2.waitKey(0) #Wait until a key is pressed while the img window is selected
