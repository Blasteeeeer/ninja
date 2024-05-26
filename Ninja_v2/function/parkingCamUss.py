from picamera2 import Picamera2
import cv2
import numpy as np
import time
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..')) # including parent directory for importing
import sensor

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

    return morph_mask

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

def main():
    
    uss = sensor.uss.uss()
    
    picam2 = Picamera2()
    config = picam2.create_video_configuration(main = { "format":"RGB888"}, raw = {"size":(4608,2592)}, controls={"FrameRate":10})
    picam2.configure(config)

    # picam2.set_controls({"ExposureTime": 10000, "AnalogueGain": 5}) #Shutter time and analogue signal boost
    picam2.start()
    
    intialTracbarVals = [0, 165, 0, 240]
    initializeTrackbars(intialTracbarVals)
    
    while True:
        # taking frame from camera
        frame = picam2.capture_array()
        frame = cv2.resize(frame, (480, 240))
        cv2.imshow("frame", frame)
        
        #bird eye view
        points = valTrackbars() #fetch values from trackbar
        BridView = warpImg(frame, points, frame.shape[1], frame.shape[0])
        
        gray = cv2.cvtColor(BridView, cv2.COLOR_BGR2GRAY)
        # extracting only yellow color and creating a mask
        yellow_mask = thresholding(BridView)
        
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
            
            
#         right_distance = uss.getDistance('right')
#         print("Right distance :", right_distance, "cm" )
#          
#         #else
#         left_distance = uss.getDistance('left')
#         print("Left distance :", left_distance, "cm" )

        
        
        
        
        edges = cv2.Canny(yellow_mask, 50, 150)
#         cv2.imshow("edges", edges)
        
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        hImg, wImg, _ = BridView.shape
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.04*cv2.arcLength(contour, True), True)
            
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = float(w)/h
                print("aspet_ratio = ",aspect_ratio)
                print("w=", w ,"h=", h)
#                 if 40/50 < aspect_ratio < 50/40:
#                 if (90< w <100) or (40< h <50):
#                     print("w___=", w ,"h__=", h)
                cv2.rectangle(BridView, (x,y), (x+w, y+h), (0,255,0), 2)
                cv2.putText(BridView, (str(w) +" " + str(h)), (x,hImg-y), cv2.FONT_HERSHEY_COMPLEX, 1, (255,0,0),2)
                print("Side length of square = ", w)
                time.sleep(1)
#         cv2.imshow("Side detected", BridView)  
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    picam2.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    
    
