from time import sleep

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..')) # including parent directory for importing
import sensor
from drive import drive
from steer import Servo
import time

def adaptive_cruise_control(uss, bot_drive,Lane):
    acc_safe_distance = 20 #set the desired distance to maintain from the vehicle ahead
    acc_speed = 10
    buffer_distance = 45 #tunable
    delta_speed = 5

    while True:
        
        front_distance = uss.getDistance('front')
#         back_distance = uss.getDistance('rear')
        left_distance = uss.getDistance('left')
        right_distance = uss.getDistance('right')
        print("Front distance :", front_distance, "cm" )
#         print("Back distance :", back_distance, "cm" )
        print("Left distance :", left_distance, "cm" )
        print("Right distance :", right_distance, "cm" )

        if Lane =='left':
            print("Running in left lane")
            if left_distance <=10 :
    #                 steer right with more angle
                    servo.set_SteeringAngle(10)
            elif right_distance <=91  :
                    # steer left with less angle
                    servo.set_SteeringAngle(-10)
            
        elif Lane =='right':
            print("Running in right lane")
            if right_distance <=10 :
    #                 steer left with more angle
                    servo.set_SteeringAngle(-10)
            elif left_distance <=91  :
                    # steer right with less angle
                    servo.set_SteeringAngle(10)
        else:
            print("In middle")

            if (left_distance + right_distance) <= 105:
                if left_distance > 45 and left_distance < 55:
    #                 steer right with more angle
                    servo.set_SteeringAngle(10)
                elif left_distance > 35 and left_distance <45 :
                    # steer right with less angle
                    servo.set_SteeringAngle(15)
                
                if right_distance > 45 and right_distance < 55:
    #                 steer left with more angle
                    servo.set_SteeringAngle(-10)
                elif right_distance > 35 and right_distance <45:
                    # steer left with less angle
                    servo.set_SteeringAngle(-15)

            elif left_distance + right_distance > 105:
                if (left_distance > right_distance):
                    servo.set_SteeringAngle(-25)
                else :
                    servo.set_SteeringAngle(25)
                    
            else:
                pass

   
        
        #speed from encoder needs to be read her*****
        bot_speed = bot_drive.get_veh_speed()
        
        if front_distance > 35:
            #Drive forward
            print("Driving forward")
            if front_distance > (acc_safe_distance+buffer_distance):
                #Vehicle is far from the vehicle stand, increase speed
                if(bot_speed < acc_speed):
                    bot_drive.forward(bot_speed+delta_speed)
                else:
                    bot_drive.forward(bot_speed-delta_speed)
            elif front_distance < (acc_safe_distance-buffer_distance):
                #Vehicle is close to the vehicle ahead, reduce speed relative to front distance
                bot_drive.forward((front_distance/acc_safe_distance)*acc_speed)
                
        else:
             bot_drive.forward(0)
             sleep(1)
             
        print()     
        sleep(0.1)

                
if __name__ == '__main__':
    try:
        uss = sensor.uss.uss()
        bot_drive = drive(1)
        servo = Servo()
        
        #check for green light
        servo.set_PanAngle(-20)
        #sensor.camera.color_detect_green.main()
        
        #green is detected
        servo.set_PanAngle(20 )
        
        servo.set_SteeringAngle(0)

        time.sleep(1)

        left_distance = uss.getDistance('left')
        right_distance = uss.getDistance('right')

        print("Left distance :", left_distance, "cm" )
        print("Right distance :", right_distance, "cm" )

        time.sleep(1)

        #Check the lane
        if left_distance <=20:
            print("In left lane")
            Lane = 'left'
            
        elif right_distance <=20:
            print("In right lane")
            Lane = 'right'

        else:
            print("In middle lane")
            Lane = 'middle'
        
        #run the acc
        adaptive_cruise_control(uss, bot_drive,Lane)
#         print("ACC is running")
    
    except KeyboardInterrupt:
        print("Program end")
        
    finally:
        print("finally part")
        del bot_drive
        del uss
        del servo
