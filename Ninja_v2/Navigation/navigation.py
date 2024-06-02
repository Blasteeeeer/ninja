import cv2
from picamera2 import Picamera2

import sys
import os
import time
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '..')) # including parent directory for importing
import sensor
import utlis
from drive import drive
from steer import Servo


def create_mask(image):
    # Convert the image to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the range for blue color in HSV
    lower_blue = np.array([69, 73, 50])
    upper_blue = np.array([126, 255, 255])
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    blue_detected = blue_detect(blue_mask)
    
        
    # Define the range for white color in HSV
    lower_white = np.array([0, 0, 98])
    upper_white = np.array([162, 255, 255])
    white_mask = cv2.inRange(hsv, lower_white, upper_white)

    # Combine the blue and white masks
    combined_mask = cv2.bitwise_or(blue_mask, white_mask)

    # Create the final mask where blue and white areas are white and others are black
    final_mask = cv2.merge([combined_mask, combined_mask, combined_mask])

    # Mask the image
    result = cv2.bitwise_and(image, final_mask)

    # Convert the masked areas to white and others to black
    result[np.where((result != [0, 0, 0]).all(axis=2))] = [255, 255, 255]
    result[np.where((result == [0, 0, 0]).all(axis=2))] = [0, 0, 0]

    return result,blue_detected

#Detect blue
def blue_detect(mask):
    
    is_Detect = False
    kernel_5 = np.ones((3,3),np.uint8) #Define a 5×5 convolution kernel with element values of all 1.
    
    morphologyEx_img = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_5,iterations=1)              # Perform an open operation on the image 

    # Find the contour in morphologyEx_img, and the contours are arranged according to the area from small to large.
    _tuple = cv2.findContours(morphologyEx_img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)      
    # compatible with opencv3.x and openc4.x
    if len(_tuple) == 3:
        _, contours, hierarchy = _tuple
    else:
        contours, hierarchy = _tuple
    
    color_area_num = len(contours) # Count the number of contours

    if color_area_num > 0: 
        for i in contours:    # Traverse all contours
            x,y,w,h = cv2.boundingRect(i)      # Decompose the contour into the coordinates of the upper left corner and the width and height of the recognition object
            # Draw a rectangle on the image (picture, upper left corner coordinate, lower right corner coordinate, color, line width)
            #print("w=", w, "h=", h, "area=",w*h)
            if (w*h) > 50000:
                is_Detect = True
                break
            
    return is_Detect
            
                

def getLaneCurve(imgWarp, curveList = [], display=2):
    
    avgVal = 2

    curveAveragePoint = utlis.getHistogram(imgWarp, display=False, minPer=0.9)

    refer_point = 6

    middlePoint = utlis.getHistogram(imgWarp, display=False, minPer=0.5, region=refer_point)
    
    curveRaw = curveAveragePoint - middlePoint

    curveList.append(curveRaw)
    if len(curveList) > avgVal:
        curveList.pop(0)

    curve = int(sum(curveList) / len(curveList))

    if display != 0:
        reference_line = imgWarp.shape[0] // refer_point
        cv2.line(imgWarp, (0, imgWarp.shape[0] - reference_line), (imgWarp.shape[1], imgWarp.shape[0] - reference_line),
                 (0, 0, 255), 2)

        imgInvWarp = utlis.warpImg(imgWarp, points, wT, hT, inv=True)
        imgInvWarp = cv2.cvtColor(imgInvWarp, cv2.COLOR_GRAY2BGR)
        imgInvWarp[0:hT // 3, 0:wT] = 0, 0, 0
        imgLaneColor = np.zeros_like(img)
        imgLaneColor[:] = 0, 255, 0
        imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)
        imgResult = cv2.addWeighted(imgWarp, 1, imgLaneColor, 1, 0)
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
                                             [imgLaneColor, imgResult]))
        cv2.imshow('ImageStack', imgStacked)
        
    elif display == 1:
        cv2.imshow('Resutlt', imgResult)


    return curve, curveList


def Find_steer(curveVal):
    sensitivity = 0.34 #0.65
            
    steer_angle = curveVal * sensitivity
    
    # if steer angle beyond the limit
    if steer_angle < -40:
        steer_angle = -40
    elif steer_angle  > 40:
        steer_angle = 40
        
    return steer_angle

def avoid_barricade():
    #Check if we hit the baricade
        distance = uss.getDistance('front')
        
        # Hitting the baricade
        if(distance <= 20 and distance >=0):
            print("Danger Distance")
            bot_drive.forward(0)
            servo.set_SteeringAngle(0)
            
            time.sleep(0.1)
            bot_drive.reverse(5)
            time.sleep(0.9)
            bot_drive.forward(5)

# Main function
def main():
    # Capture the image
    picam2 = Picamera2()
    config = picam2.create_video_configuration(main = { "format":"RGB888"}, raw = {"size":(4608,2592)}, controls={"FrameRate":24})
    picam2.configure(config)

    picam2.start()
    time.sleep(0.1)
    
    curveList = []
    intialTrackBarVals = [0, 195, 0, 240]
    utlis.initializeTrackbars(intialTrackBarVals)
    
    bot_drive.forward(10)
    
    nav_count = 0
    
    exp_nav_count = 1
    
    while True:
        # taking frame from camera
        frame = picam2.capture_array()
        frame = cv2.resize(frame, (480, 240))
        cv2.imshow("frame", frame)
        
        points = utlis.valTrackbars()
        imgWarp = utlis.warpImg(frame, points, 480, 240)
        cv2.imshow("imgWarp", imgWarp)
        
        # Create the mask and get the result
        masked_image,blue_detected = create_mask(imgWarp)

        # Display the result
        cv2.imshow('Masked Image', masked_image)
        
        #Find curvature of lane
        curve, curveList = getLaneCurve(masked_image,curveList, display=0)
        
        #Find steering angle based on curve
        steer_angle = Find_steer(curve)
        print('Steering angle',steer_angle)

        #Set steering angle
        servo.set_SteeringAngle(steer_angle)
        
        avoid_barricade()
        #Check if blue detected with sufficient area
        if blue_detected:
            print("Blue detected")
            
            
            servo.set_SteeringAngle(0)
            #Avoid hitting barricade
            avoid_barricade()
            
            nav_count = nav_count + 1
            print('Navigation count',nav_count)
            
            current_dist = bot_drive.get_veh_distance()
            print('current distance=', current_dist)
            nav_dist = 35  # Tunable distance from blue detection to point B in nav map
            avoid_dist = 50 # Distance to overcome blue area
            
            if nav_count != exp_nav_count:
                #Driving to overcome blue area
                while bot_drive.get_veh_distance() <= (current_dist + avoid_dist):
                    print('running distance=', bot_drive.get_veh_distance())
                    bot_drive.forward(10)
                    
                print('Overcame the blue area')
                
            else:
                #Driving to reach the destination
                while bot_drive.get_veh_distance() <= (current_dist + nav_dist):
                    print('running distance=', bot_drive.get_veh_distance())
                    bot_drive.forward(10)
                print('Reached destination') 
                bot_drive.forward(0)

                #Stop for 10 secs
                time.sleep(10)

            bot_drive.forward(10)
        
        k = cv2.waitKey(1) & 0xFF
        # 27 is the ESC key, which means that if you press the ESC key to exit
        if k == 27:
            break

    picam2.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        bot_drive = drive(1) #single motor drive
        uss = sensor.uss.uss()
        servo = Servo()
        
        bot_drive.forward(0)
        
        #check for green light
        servo.set_PanAngle(-20)
        #sensor.camera.color_detect_green.main()
        
        #green is detected
        servo.set_PanAngle(20)
        
        #Set steering to straight direction
        servo.set_SteeringAngle(0)
    
        main()
        
    except KeyboardInterrupt:
        print("Program terminated by user")
        
    finally:
        # Clean up GPIO
        del bot_drive
        del servo
        cv2.destroyAllWindows()