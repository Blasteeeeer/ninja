from time import sleep

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..')) # including parent directory for importing
import sensor
from drive import drive
from steer import Servo

def adaptive_cruise_control(uss, bot_drive):
    acc_safe_distance = 20 #set the desired distance to maintain from the vehicle ahead
    acc_speed = 25
    buffer_distance = 45 #tunable
    delta_speed = 5
    i = 0

    while True:
        
        #Set steering to straight in every cycle
        servo.set_SteeringAngle(7)
        
        print("Driving forward", i)
        i+=1
        front_distance = uss.getDistance('front')
#         back_distance = uss.getDistance('rear')
#         left_distance = uss.getDistance('left')
#         right_distance = uss.getDistance('right')
        print("Front distance :", front_distance, "cm" )
#         print("Back distance :", back_distance, "cm" )
#         print("Left distance :", left_distance, "cm" )
#         print("Right distance :", right_distance, "cm" )
        
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
        servo.set_PanAngle(-15)
        sensor.camera.color_detect_green.main()
        
        #green is detected
        servo.set_PanAngle(25)
        
        servo.set_SteeringAngle(0)
        
        #run the acc
        adaptive_cruise_control(uss, bot_drive)
#         print("ACC is running")
    
    except KeyboardInterrupt:
        print("Program end")
        
    finally:
        print("finally part")
        del bot_drive
        del uss
        del servo
