from FileManagement import fileDB
from rpi_hardware_pwm import HardwarePWM
import time
import os



class Servo:
    def __init__(self):
        self.pwm1 = HardwarePWM(pwm_channel=1, hz=50, chip=0)
        self.pwm1.start(0)
        
        self.pwm0 = HardwarePWM(pwm_channel=0, hz=50, chip=0)
        self.pwm0.start(0)

        # Set PWM parameters
        self.duty_cycle_min = 2.5  # Duty cycle for minimum position (in percentage)
        self.duty_cycle_max = 12.5  # Duty cycle for maximum position (in percentage)
        self.angle_min = -90  # Minimum angle in degrees
        self.angle_max = 90  # Maximum angle in degrees
        
        # fetch the value from the MotorsConfiguration
        self.config_file = fileDB('/home/raspberrypi/Documents/Ninja_v2/steer/MotorsConfiguration.txt')
#         self.steer_servo_conf_val = float(self.config_file.get("steer_servo", default_value = 0))
#         self.pan_servo_conf_val = float(self.config_file.get("pan_servo", default_value = 0))
        self.steer_servo_conf_val = -10
        self.pan_servo_conf_val = 0

#         print("steer_servo=", self.steer_servo_conf_val)
        self.current_angle = self.steer_servo_conf_val
        
#         self.set_SteeringAngle(self.steer_servo_conf_val)
        
        
    # Should be called by the main function.
    # adding with the configurated value to remove error
    def set_SteeringAngle(self, value, Smoothnes = True):
        value = value + self.steer_servo_conf_val
        
        if Smoothnes:
            self.set_angle_smooth(value)
        else:
            self.set_angle(value, pwm = 1)
            
    def set_PanAngle(self, value):
#         value = value + self.pan_servo_conf_val
        self.set_angle(value, pwm = 0)
        

    # used by Servo_configuration to save the info in MotorsConfiguration
    def set_servo_conf(self, value):
        self.steer_servo_conf_val = value[0]
        self.config_file.set("steer_servo", "%s"%value)
        self.set_SteeringAngle(self.steer_servo_conf_val)
        
        self.pan_servo_conf_val = value[1]
        self.config_file.set("pan_servo", "%s"%value)
        self.set_SteeringAngle(self.pan_servo_conf_val)
        
        
    # Function to set servo angle with smoothing
    # not called anywhere
    def set_angle_smooth(self, angle):
        step = 6
        
        while self.current_angle != angle:
            if self.current_angle < angle:
                self.current_angle += step
                if self.current_angle > angle:
                    self.current_angle = angle
            else:
                self.current_angle -= step
                if self.current_angle < angle:
                    self.current_angle = angle
            self.set_angle(self.current_angle, pwm = 1)
            time.sleep(0.01)  # Adjust this value as needed
                
    # Function to set servo angle
    def set_angle(self, angle, pwm):
        duty_cycle = ((angle - self.angle_min) / (self.angle_max - self.angle_min)) * (self.duty_cycle_max - self.duty_cycle_min) + self.duty_cycle_min
        if duty_cycle > 100:
            duty_cycle = 100
        elif duty_cycle < 0:
            duty_cycle = 0
            
        if(pwm == 0):
            self.pwm0.change_duty_cycle(duty_cycle)
        elif(pwm == 1):
            self.pwm1.change_duty_cycle(duty_cycle)
        
        self.current_angle = angle
        time.sleep(0.01)  # Adjust this value as needed
        
    def kill_pan_servo(self):
        self.pwm0.stop()
        
    def __del__(self):
        self.set_angle(self.steer_servo_conf_val, 1)
        self.set_angle(self.pan_servo_conf_val, 0)
        time.sleep(5)
        
        self.pwm1.stop()
        self.pwm0.stop()

if __name__ == "__main__":
    steer = Servo()

    try:
        i = 0
        while i<1:
            print("10")
            steer.set_PanAngle(0)
#             steer.set_angle(0, 1)
#             steer.set_SteeringAngle(0, False)
            time.sleep(1)
            
            print("0")
            steer.set_PanAngle(20)
#             steer.set_angle(-10, 1)
#             steer.set_SteeringAngle(10, False)
            time.sleep(1)
            
            i+=1
        
#         steer.set_SteeringAngle(0, False)
#         steer.kill_pan_servo()
        
        while True:
#             steer.set_SteeringAngle(30, False )
#             time.sleep(1)
#             steer.set_SteeringAngle(0, False )
            time.sleep(1)
    #     steer.set_SteeringAngle(-20)
    #     time.sleep(1)
    #     steer.set_SteeringAngle(40)
    #     time.sleep(1)
    except KeyboardInterrupt:
        print("Program terminated by user")

    finally:
        # Clean up GPIO
        del steer
    