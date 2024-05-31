import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
from picarx import Picarx
import numpy as np
import time
import avoiding_obstacles as obstacle
import sys
import Lane as ld
import utlis
import os

def main():
    print('started') 
    px = Picarx()
    px.set_camera_servo1_angle(20)


    with PiCamera() as camera:
        camera.awb_mode = 'auto'
        #camera.exposure_mode = 'night'
        camera.exposure_compensation = 3
        camera.brightness = 65
        #camera.awb_gains = 1.6
        #px.set_camera_servo2_angle(20)
        #print("start color detect")
        camera.resolution = (640,480)
        camera.framerate = 24
        rawCapture = PiRGBArray(camera, size=camera.resolution)  
        time.sleep(0.1)

        for frame in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):# use_video_port=True
            img = frame.array
            #img = cv2.imread("cmd_cap.jpg")       
            img,mask_red,approved_red, is_red, resize_img =  color_detect(img,'red')  # Red color detection function
            img,mask_green,approved_green, is_green, resize_img =  color_detect(img,'green')  #Green color detection function
            
            rawCapture.truncate(0)   # Release cache
            
            
            if(is_green):
                print('green detected')
                break
            
            
            if(is_red):
                # move the bot
                px.forward(0)
                #print("RED")
                
        
            k = cv2.waitKey(1) & 0xFF
            # 27 is the ESC key, which means that if you press the ESC key to exit
            if k == 27:
                break
            
            if k == ord('s'):
                print("image saved")
                cv2.imwrite("crop.jpg",resize_img)
                cv2.imwrite("raw_image.jpg",img)
            

    print('After break - Green light detected or Esc pressed')
    cv2.destroyAllWindows()
    camera.close()
    px.set_camera_servo1_angle(0)
    ld.main()
            
            
    
def color_detect(img,color_name):
    
    color_dict = {'red':[0,10],'orange':[11,18],'yellow':[25,35],'green':[42,85],'blue':[92,110],'purple':[115,165],'red_2':[172,180]}  #Here is the range of H in the HSV color space represented by the color
    kernel_5 = np.ones((3,3),np.uint8) #Define a 5×5 convolution kernel with element values of all 1.

    # The blue range will be different under different lighting conditions and can be adjusted flexibly.  H: chroma, S: saturation v: lightness
    raw_resize_img = cv2.resize(img, (160,120), interpolation=cv2.INTER_LINEAR)  # In order to reduce the amount of calculation, the size of the picture is reduced to (160,120)
    resize_img = raw_resize_img[0:40, 0:160]
    hsv = cv2.cvtColor(resize_img, cv2.COLOR_BGR2HSV)              # Convert from BGR to HSV
    color_type = color_name
    is_Detect = False
    
    mask = cv2.inRange(hsv,np.array([min(color_dict[color_type]), 80, 0]), np.array([max(color_dict[color_type]), 255,  255]) )           # inRange()：Make the ones between lower/upper white, and the rest black
    if color_type == 'red': 
            mask_2 = cv2.inRange(hsv, (color_dict['red_2'][0],50,70), (color_dict['red_2'][1],255,255))
            mask = cv2.bitwise_or(mask, mask_2)

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
            if w >= 3 and w <= 8 and h >= 3 and h <= 8: # Because the picture is reduced to a quarter of the original size, if you want to draw a rectangle on the original picture to circle the target, you have to multiply x, y, w, h by 4.
                x = x * 4
                y = y * 4 
                w = w * 4
                h = h * 4
                cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)  # Draw a rectangular frame
                cv2.putText(img,color_type,(x,y), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),2)# Add character description
                is_Detect = True

    return img,mask,morphologyEx_img,is_Detect, resize_img


if __name__ == "__main__":
    main()

