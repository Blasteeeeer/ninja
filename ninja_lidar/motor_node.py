# subscriber_node.py
import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
from time import sleep
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..')) # including parent directory for importing
from drive import drive

def motor_callback(data):
    acc_safe_distance = 20 #set the desired distance to maintain from the vehicle ahead
    acc_speed = 50
    buffer_distance = 5 #tunable
    delta_speed = 5
    i = 0

    while True:
        print("Driving forward", i)
        i+=1
        front_distance = data.data('front')
#         back_distance = data.data('rear')
#         left_distance = data.data('left')
#         right_distance = data.data('right')
        print("Front distance :", front_distance, "cm" )
#         print("Back distance :", back_distance, "cm" )
#         print("Left distance :", left_distance, "cm" )
#         print("Right distance :", right_distance, "cm" )
        
        #speed from encoder needs to be read her*****
        bot_speed = bot_drive.get_veh_speed()
        
        if front_distance > 10:
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
             
        print()     
        sleep(0.1)

def subscriber_node():
    # rospy.init_node('obstacle_distance_subscriber')
    # rospy.Subscriber('/obstacle_distance', Float32, motor_callback)
    # rospy.spin()  # Keep the node running
    pass

if __name__ == '__main__':

	bot_drive = drive(1)
    # try:
    #     subscriber_node()
    # except rospy.ROSInterruptException:
    #     pass
