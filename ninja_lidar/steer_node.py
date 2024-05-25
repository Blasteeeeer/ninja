#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from FileManagement import fileDB
from rpi_hardware_pwm import HardwarePWM
import time
import os

class Servo:
    def __init__(self):
        self.pwm = HardwarePWM(pwm_channel=1, hz=60, chip=0)
        self.pwm.start(0)

        # Set PWM parameters
        self.duty_cycle_min = 2.5  # Duty cycle for minimum position (in percentage)
        self.duty_cycle_max = 12.5  # Duty cycle for maximum position (in percentage)
        self.angle_min = -90  # Minimum angle in degrees
        self.angle_max = 90  # Maximum angle in degrees
        
        # Fetch the value from the MotorsConfiguration
        self.config_file = fileDB('/home/ninja/Desktop/Ninja_v2/steer/MotorsConfiguration.txt')
        self.steer_servo_conf_val = float(self.config_file.get("steer_servo", default_value=0))
        print("steer_servo=", self.steer_servo_conf_val)
        self.current_angle = self.steer_servo_conf_val
        
    def set_SteeringAngle(self, value, Smoothnes=True):
        value = value + self.steer_servo_conf_val
        
        if Smoothnes:
            self.set_angle_smooth(value)
        else:
            self.set_angle(value)

    def set_servo_conf(self, value):
        self.steer_servo_conf_val = value
        self.config_file.set("steer_servo", "%s" % value)
        self.set_SteeringAngle(self.steer_servo_conf_val)
        
    def set_angle_smooth(self, angle):
        step = 3
        
        while self.current_angle != angle:
            if self.current_angle < angle:
                self.current_angle += step
                if self.current_angle > angle:
                    self.current_angle = angle
            else:
                self.current_angle -= step
                if self.current_angle < angle:
                    self.current_angle = angle
            self.set_angle(self.current_angle)
            time.sleep(0.01)
                
    def set_angle(self, angle):
        duty_cycle = ((angle - self.angle_min) / (self.angle_max - self.angle_min)) * (self.duty_cycle_max - self.duty_cycle_min) + self.duty_cycle_min
        self.pwm.change_duty_cycle(duty_cycle)
        
        self.current_angle = angle
        time.sleep(0.01)
        
    def __del__(self):
        self.pwm.stop()

def obstacle_distance_callback(msg):
    obstacle_distance = msg.data
    rospy.loginfo(f"Received obstacle distance: {obstacle_distance}")

    if obstacle_distance < 1.0:
        servo.set_SteeringAngle(90)  # Turn servo to 90 degrees when an obstacle is close
    else:
        servo.set_SteeringAngle(0)   # Turn servo to 0 degrees when no obstacle is close

def servo_control_node():
    # rospy.init_node('servo_control')
    # rospy.Subscriber('/obstacle_distance', Float32, obstacle_distance_callback)
    # rospy.spin()
    pass

if __name__ == '__main__':
    servo = Servo()
    servo_control_node()
    # try:
    #     servo_control_node()
    # except rospy.ROSInterruptException:
    #     pass
    # finally:
    #     del servo
