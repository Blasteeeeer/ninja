#!/usr/bin/env python3

import rospy
import geometry_msgs.msg import Twist
from drive import drive
from Servo import Servo

class drivebot:
    def __init__(self,drive, steer):
        rospy.init_node('drive_node', anonymous=True)
        self.wheelbase = 0.5 # Update the bot wheelbase here. (Distance between left and right wheel along axle)
        self.drive = drive
        self.steer = steer

        # Subscribe to /cmd_vel topic from navigation node
        rospy.Subscriber('/cmd_vel', Twist, self.drivebot_callback)

    def drivebot_callback(self, msg):
        #Calculate the drive velocity and steering angle based on the subscribed velocity commands
        linear_velocity = msg.liner.x
        angular_velocity = msg.angular.z

        forward_drive_velocity = linear_velocity
        steering_angle = (angular_velocity * self.wheelbase) / 2.0

        # Log the data
        rospy.loginfo('Drive velocity : %f, Steering Angle : %f', forward_drive_velocity,steering_angle)

        self.drive.forward(forward_drive_velocity)
        self.steer.set_SteeringAngle(steering_angle)


if __name__ == '__main__':

    drive_motor = drive(1)
    steer_servo = Servo()
    drive_bot = drivebot(drive_motor, steer_servo)
    rospy.spin()

    