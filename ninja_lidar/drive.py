import RPi.GPIO as GPIO
import time
import os

from encoderMotor_driver import encoderMotor

drive_type = None #single or dual motor drive

class drive:
    def __init__(self,dr_type=1):
         
        global drive_type
        drive_type = dr_type
        self.DR_type = dr_type
        if dr_type == 2:
            ### encoders Motors for dual motor
            self.left_motor = encoderMotor(6)
            self.right_motor = encoderMotor(7)
            self.left_motor.set_lines_per_rot(13)
            self.right_motor.set_lines_per_rot(13)
            self.left_motor.reset_encoder()
            self.right_motor.reset_encoder()
            
        elif dr_type == 1:
            ### encoders Motors for single motor
            self.right_motor = encoderMotor(6)
            self.right_motor.set_lines_per_rot(13)
            self.right_motor.reset_encoder()
        else:
            print("Invalid drive type. Only single or dual motor drive allowed.")


    def forward(self, speed_cmps):
        distance_cm_per_rot = 22
        speed_rpm = (speed_cmps*60)/distance_cm_per_rot
        #print("Set Speed cmps:",speed_cmps)
        #print("Set Speed rpm:",speed_rpm)
        speed_rpm = round(speed_rpm) 
        if speed_rpm < 10 :
            if drive_type == 2: #dual motor drive
                print("Dual motor drive -brake")
                self.left_motor.brake_motor("reverse")
                self.right_motor.brake_motor("forward")
            else: #single motor drive
                self.right_motor.brake_motor("forward")
        else:
            if drive_type == 2: #dual motor drive
                print("Dual motor drive - forward")
                self.left_motor.reverse(speed_rpm)
                self.right_motor.forward(speed_rpm)
            else: #single motor drive
                self.right_motor.forward(speed_rpm)
                
    def reverse(self, speed_cmps):
        distance_cm_per_rot = 22
        speed_rpm = (speed_cmps*60)/distance_cm_per_rot
        speed_rpm = round(speed_rpm) 
        if speed_rpm < 10:
            if self.drive_type == 2: #dual motor drive
                self.left_motor.brake_motor("forward")
                self.right_motor.brake_motor("reverse")
            else: #single motor drive
                self.right_motor.brake_motor("reverse")
        else:
            if self.drive_type == 2: #dual motor drive
                print("Dual motor drive - reverse")
                self.left_motor.forward(speed_rpm)
                self.right_motor.reverse(speed_rpm)
            else: #single motor drive
                self.right_motor.reverse(speed_rpm)
            
    def get_veh_distance(self):
        return self.right_motor.get_position()
    
    def get_veh_speed(self):
        speed_rpm = self.right_motor.get_speed()
        distance_cm_per_rot = 22
        speed_cmps = distance_cm_per_rot*(speed_rpm/60)
        #print("get Speed cmps:",speed_cmps)
        #print("get Speed rpm:",speed_rpm)
        return speed_cmps
    
    def __del__(self):
        if self.DR_type  == 2:
            ### encoders Motors for dual motor
            self.right_motor.stop_motor()
            self.left_motor.stop_motor()

            
        elif self.DR_type  == 1:
            ### encoders Motors for single motor
            self.right_motor.stop_motor()
    
    

if __name__ == "__main__":
    bot_drive = drive(1) #single motor drive
    #bot_drive = drive(2) #dual motor drive

    try:
        bot_drive.forward(10)
        while True:
            bot_speed = bot_drive.get_veh_speed()
            print("Bot Speed :", bot_speed, "cm/s" )
            time.sleep(0.1)
        
    except KeyboardInterrupt:
        # when "control C" is pressed, clean up the GPIO from I/O
        print("Exitted")
        #bot_drive.left_motor.stop_motor()
        bot_drive.right_motor.stop_motor()
        #GPIO.cleanup()
