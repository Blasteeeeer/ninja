from rpi_hardware_pwm import HardwarePWM
import time
import RPi.GPIO as GPIO

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..')) # including parent directory for importing
from drive import drive
import sensor

# channel 0 = GPIO12
# channel 1 = GPIO19
pwm = HardwarePWM(pwm_channel=1, hz=60, chip=0)
pwm.start(0)

# Set PWM parameters
frequency = 60  # Hz (typical servo motor frequency)
duty_cycle_min = 2.5  # Duty cycle for minimum position (in percentage)
duty_cycle_max = 12.5  # Duty cycle for maximum position (in percentage)
angle_min = -90  # Minimum angle in degrees
angle_max = 90  # Maximum angle in degrees

# Function to set servo angle with smoothing
def set_angle_smooth(angle):
    current_angle = 0
    step = 1 if abs(angle - current_angle) > 10 else 1  # Adjust step size based on angle difference
    while current_angle != angle:
        if current_angle < angle:
            current_angle += step
            if current_angle > angle:
                current_angle = angle
        else:
            current_angle -= step
            if current_angle < angle:
                current_angle = angle
        set_angle(current_angle)
        time.sleep(0.01)  # Adjust this value as needed

# Function to set servo angle
def set_angle(angle):
    duty_cycle = ((angle - angle_min) / (angle_max - angle_min)) * (duty_cycle_max - duty_cycle_min) + duty_cycle_min
    pwm.change_duty_cycle(duty_cycle)
    time.sleep(0.01)  # Adjust this value as needed


if __name__ == "__main__":
    # Main program

    try:
            # Set servo to 0 degrees (center) with smoothing
        print("set to 0")
        set_angle(0)
        time.sleep(1)  # Adjust this value as needed
        
        step = 3

        # declaring object for USS
        uss = sensor.uss.uss()
        bot_drive = drive(1) #single motor drive
        
        while True:
            bot_drive.forward(10)


            #################################################
            ########### for right side steering ##################
            # Set servo to -90 degrees (left) with smoothing
    #         time.sleep(3)
            #print("set to 30")
            for i in range(0,31,step):
                set_angle(i)
                time.sleep(0.05)  # Adjust this value as needed
            
    #         time.sleep(3)
            # Set servo to 90 decgrees (right) with smoothing
            #print("set to 0")
            for i in range(30,-1,-step):
                set_angle(i)
                time.sleep(0.05)  # Adjust this value as needed
            
            ################################################################
            ########### for left side steering ##################
    #         time.sleep(3)
            #print("set to -30")
            for i in range(0,-31,-step):
                set_angle(i)
                time.sleep(0.05)  # Adjust this value as needed
                
            
            # Set servo to 90 degrees (right) with smoothing
    #         time.sleep(3)
            #print("set to 0")
            for i in range(-30,1,step):
                set_angle(i)
                time.sleep(0.05)  # Adjust this value as needed

            print("Front USS Distance = ", uss.getDistance('front'))
            print("Rear USS Distance = ", uss.getDistance('rear'))

            bot_speed = bot_drive.get_veh_speed()
            print("Bot Speed :", bot_speed, "cm/s" )
            time.sleep(0.1)
                
    except KeyboardInterrupt:
        print("Program terminated by user")

    finally:
        pwm.stop() # stop the current for PWM
        del bot_drive
#         bot_drive.right_motor.stop_motor() # stop the motor
        set_angle(0) # set the angle to 0
        GPIO.cleanup() # Clean up GPIO
