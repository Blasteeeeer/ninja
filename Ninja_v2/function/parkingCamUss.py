from picamera2 import Picamera2
import cv2
import numpy as np
import time
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..')) # including parent directory for importing
import sensor
from drive import drive
from steer import Servo


def nothing(a):
    pass

def initializeTrackbars(intialTracbarVals, wT=480, hT=240):
    cv2.namedWindow("TopView")
    cv2.createTrackbar("Width Top", "TopView", intialTracbarVals[0], wT // 2, nothing)
    cv2.createTrackbar("Height Top", "TopView", intialTracbarVals[1], hT, nothing)
    cv2.createTrackbar("Width Bottom", "TopView", intialTracbarVals[2], wT // 2, nothing)
    cv2.createTrackbar("Height Bottom", "TopView", intialTracbarVals[3], hT, nothing)
    
    cv2.namedWindow("ColorMask")
    cv2.createTrackbar("lower_h", "ColorMask", 20, 255, nothing)
    cv2.createTrackbar("lower_s", "ColorMask", 35, 255, nothing)
    cv2.createTrackbar("lower_v", "ColorMask", 100, 255, nothing)
    cv2.createTrackbar("upper_h", "ColorMask", 35, 255, nothing)
    cv2.createTrackbar("upper_s", "ColorMask", 255, 255, nothing)
    cv2.createTrackbar("upper_v", "ColorMask", 255, 255, nothing)
    
def thresholding(img):
    
    is_Detect = False
    
    l_h = cv2.getTrackbarPos("lower_h", "ColorMask")
    l_s = cv2.getTrackbarPos("lower_s", "ColorMask")
    l_v = cv2.getTrackbarPos("lower_v", "ColorMask")
    u_h = cv2.getTrackbarPos("upper_h", "ColorMask")
    u_s = cv2.getTrackbarPos("upper_s", "ColorMask")
    u_v = cv2.getTrackbarPos("upper_v", "ColorMask")

    
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lowerMask = np.array([l_h, l_s, l_v])
    upperMask = np.array([u_h, u_s, u_v])
    mask = cv2.inRange(imgHsv, lowerMask, upperMask)
    
    kernel = np.ones((5,5), np.uint8)
    morph_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # Find the contour in morphologyEx_img, and the contours are arranged according to the area from small to large.
    _tuple = cv2.findContours(morph_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)      
    # compatible with opencv3.x and openc4.x
    if len(_tuple) == 3:
        _, contours, hierarchy = _tuple
    else:
        contours, hierarchy = _tuple
    
    color_area_num = len(contours) # Count the number of contours
    
    print("color_area_num", color_area_num)

    if color_area_num > 0: 
        for i in contours:    # Traverse all contours
            x,y,w,h = cv2.boundingRect(i)      # Decompose the contour into the coordinates of the upper left corner and the width and height of the recognition object
            # Draw a rectangle on the image (picture, upper left corner coordinate, lower right corner coordinate, color, line width)
            print("area",w* h)
            if (w*h)>1200: # Because the picture is reduced to a quarter of the original size, if you want to draw a rectangle on the original picture to circle the target, you have to multiply x, y, w, h by 4.
                is_Detect = True

    return morph_mask, is_Detect

def valTrackbars(wT=480, hT=240):
    widthTop = cv2.getTrackbarPos("Width Top", "TopView")
    heightTop = cv2.getTrackbarPos("Height Top", "TopView")
    widthBottom = cv2.getTrackbarPos("Width Bottom", "TopView")
    heightBottom = cv2.getTrackbarPos("Height Bottom", "TopView")
    points = np.float32([(widthTop, heightTop), (wT - widthTop, heightTop),
                         (widthBottom, heightBottom), (wT - widthBottom, heightBottom)])
    return points

def warpImg(img, points, w, h, inv=False):
    pts1 = np.float32(points)
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    if inv:
        matrix = cv2.getPerspectiveTransform(pts2, pts1)
    else:
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
    imgWarp = cv2.warpPerspective(img, matrix, (w, h))
    return imgWarp

def getHistogram(img, minPer=0.1, display=False, region=1):
    if region == 1:
        histValues = np.sum(img, axis=0)
    else:
        histValues = np.sum(img[img.shape[0] // region:, :], axis=0)

    # print(histValues)
    maxValue = np.max(histValues)
    minValue = minPer * maxValue

    indexArray = np.where(histValues >= minValue)
    basePoint = int(np.average(indexArray))
    # print(basePoint)

    if display:
        imgHist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
        imgResult = img.copy()

        for x, intensity in enumerate(histValues):
            cv2.line(imgHist, (x, img.shape[0]), (x, img.shape[0] - intensity // 255 // region), (255, 0, 255), 1)
            cv2.circle(imgHist, (basePoint, img.shape[0]), 20, (0, 255, 255), cv2.FILLED)
        return basePoint, imgHist

    return basePoint


def ParkDecision(parkDir):

    ParkingSpaceAvailable = False
    #Maximum number of Parking spaces
    ParkingspaceCount = 1
    
    while (ParkingSpaceAvailable == False) and (ParkingspaceCount < 1):
        
        uss_distance = []
        dist = bot_drive.get_veh_distance()
        park_dist = 20
        print("Current Parking space number",ParkingspaceCount)
        print(" Parking space occupied",ParkingSpaceAvailable)

        
        while bot_drive.get_veh_distance() <= (dist + park_dist):
            if (parkDir == 'right'):
#                 uss_distance = [uss_distance, uss.getDistance('right')]
                uss_distance.append(uss.getDistance('right'))
                print("Right distance :", uss_distance, "cm" )
            elif (parkDir == 'left'):
#                 uss_distance = [uss_distance, uss.getDistance('left')]
                uss_distance.append(uss.getDistance('left'))
                print("Left distance :", uss_distance, "cm" )
            else:
                pass 
            bot_drive.forward(5)
            
        print("Minimum of the distances",min(uss_distance))
         
        if min(uss_distance) < 30:
            ParkingSpaceAvailable = False
            ParkingspaceCount += 1
        else:
            ParkingSpaceAvailable = True
            bot_drive.forward(0)
            time.sleep(5)
            break
    
    return ParkingSpaceAvailable

def manoeuvreBotForParking(parkDir):
    
#     Front_distance = 0
#     Rear_distance = 0
#     
# #     while ((Rear_distance > 10) and (Front_distance > 10 )):
#         
    Front_distance = uss.getDistance('front')
    print("Front distance :", Front_distance, "cm" )
    
    Rear_distance = uss.getDistance('rear')
    print("Rear distance :", Rear_distance, "cm" )
    
    if (parkDir == 'right'):
        servo.set_SteeringAngle(20)
    elif (parkDir == 'left'):
        servo.set_SteeringAngle(-20)
    else:
        pass
     
#     print("Reversing")
#     time.sleep(1)
#     bot_drive.reverse(2)
#     time.sleep(10)
    
    print("Forward")
    bot_drive.forward(5)
    time.sleep(2)
    
    if (parkDir == 'right'):
        servo.set_SteeringAngle(-20)
    elif (parkDir == 'left'):
        servo.set_SteeringAngle(20)
    else:
        pass
    time.sleep(1)
    bot_drive.reverse(3)
    time.sleep(2)
    bot_drive.forward(0)
        
        

def main():
    picam2 = Picamera2()
    config = picam2.create_video_configuration(main = { "format":"RGB888"}, raw = {"size":(4608,2592)}, controls={"FrameRate":10})
    picam2.configure(config)

    # picam2.set_controls({"ExposureTime": 10000, "AnalogueGain": 5}) #Shutter time and analogue signal boost
    picam2.start()
    time.sleep(1)
    
    intialTracbarVals = [0, 129, 0, 240]
    initializeTrackbars(intialTracbarVals)
    ussParkSpaceAvailable = False
    
    bot_drive.forward(0)
    
    while True:
        # taking frame from camera
        frame = picam2.capture_array()
        frame = cv2.resize(frame, (480, 240))
        cv2.imshow("frame", frame)
        
        #bird eye view
        points = valTrackbars() #fetch values from trackbar
        BridView = warpImg(frame, points, frame.shape[1], frame.shape[0])
        cv2.imshow("BridView", BridView)
        
        gray = cv2.cvtColor(BridView, cv2.COLOR_BGR2GRAY)
        # extracting only yellow color and creating a mask
        yellow_mask, is_Detect = thresholding(BridView)
        cv2.imshow("yellow_mask", yellow_mask)
        print("\nyellow detected = ", is_Detect)
        
        if(is_Detect == True):
            #### STEP 3
            # whole image
            curveAveragePoint = getHistogram(yellow_mask, display=False, minPer=0.9)
            # at some reference point
            refer_point = 6
            middlePoint = getHistogram(yellow_mask, display=False, minPer=0.5, region=refer_point)
            parkingSide = middlePoint-(yellow_mask.shape[1]/2)
            print("parkingSide",parkingSide)
            
            if (parkingSide < 0):
                print("Parking on the left")
                dist = bot_drive.get_veh_distance()
                init_dist = 10 # Distance to be moved once P sign detected
                
                #Drive the bot until first parking slot
                while bot_drive.get_veh_distance() <= (dist + init_dist):
                     bot_drive.forward(5)
                    
                bot_drive.forward(0)
                time.sleep(5)
                
                # Iterate each parking space to find out free parking space
#                 ussParkSpaceAvailable = ParkDecision('left')
                
#                 if (ussParkSpaceAvailable == True):
                manoeuvreBotForParking('left')
                print("*************************************************************")
                break
#                 else:
#                     bot_drive.forward(0)
                  
                
            elif(parkingSide > 0):
                print("Parking on the right")
                dist = bot_drive.get_veh_distance()
                init_dist = 10 # Distance to be moved once P sign detected
                
                #Drive the bot until first parking slot
                print("Initisl distance = ", bot_drive.get_veh_distance())
                while bot_drive.get_veh_distance() <= (dist + init_dist):
                    bot_drive.forward(5)
                    
                print("Current distance = ", bot_drive.get_veh_distance())
                
                bot_drive.forward(0)
                time.sleep(5)
                
                # Iterate each parking space to find out free parking space
#                 ussParkSpaceAvailable = ParkDecision('right')
                
#                 if (ussParkSpaceAvailable == True):
                manoeuvreBotForParking('right')
                print("*************************************************************")
                break
#                 else:
#                     bot_drive.forward(0)
                 
                
                
            else:
                print("Do nthing")
        
        
        #introduce histogram to find more yellow on which side of frame
        
        # if camera returns right,
            # Reduce the speed
            #start calculating USS right distance travelled only till next 70cm
            # ie., 50 + 20cm buffer to reach the specified parking space
            #based on camera FOV we have to calculate start and end of first parking space
            # if there is no distance recived <30cm, then parking spec detected
            # if there are few distance recived <30cm, then restart the traversed distance from 0 to next 50cm
            # when 101 and 102 returns no object in the interval specified
            # reverse the bot by 10cm -> set steering to right steer by 2 deg and set reverse speed
            
            
        
         
        #else
        

        
        
        
        
#        edges = cv2.Canny(yellow_mask, 50, 150)
#         cv2.imshow("edges", edges)
        
#         contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         
#         hImg, wImg, _ = BridView.shape
#         for contour in contours:
#             approx = cv2.approxPolyDP(contour, 0.04*cv2.arcLength(contour, True), True)
#             
#             if len(approx) == 4:
#                 x, y, w, h = cv2.boundingRect(approx)
#                 aspect_ratio = float(w)/h
#                 #print("aspet_ratio = ",aspect_ratio)
#                # print("w=", w ,"h=", h)
# #                 if 40/50 < aspect_ratio < 50/40:
# #                 if (90< w <100) or (40< h <50):
# #                     print("w___=", w ,"h__=", h)
#                 cv2.rectangle(BridView, (x,y), (x+w, y+h), (0,255,0), 2)
#                 cv2.putText(BridView, (str(w) +" " + str(h)), (x,hImg-y), cv2.FONT_HERSHEY_COMPLEX, 1, (255,0,0),2)
#                 print("Side length of square = ", w)
#                 time.sleep(1)
# #         cv2.imshow("Side detected", BridView)  
        
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
        servo.set_PanAngle(-15)
        sensor.camera.color_detect_green.main()
        
        #green is detected
        servo.set_PanAngle(25)
        
        #go for parking
        main()
#         print("parking")
    except KeyboardInterrupt:
        print("Program terminated by user")

    finally:
        # Clean up GPIO
        del bot_drive
        del uss
        del servo
        cv2.destroyAllWindows()
        picam2.close()
        
    
    
    
