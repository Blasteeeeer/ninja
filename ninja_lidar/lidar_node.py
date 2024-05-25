#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

def lidar_callback(msg):
	#Process the lidar data
	distances = msg.ranges

	#Extract obstacle information
	obstacles = []
	for i in range(len(distances)):
		if distances[i] <1.0:   #Assuming 1.0 meter as the threshold to detect obstacle
			obstacles.append(distances[i])


	# Publish the distance data as ROS messages
	obstacle_distance = min(obstacles) if obstacles else float('inf')  # Minimum distance to closest obstacle
	#obstacle_distance_msg = float(data=obstacle_distance)
	obstacle_distance_pub.publish(obstacle_distance)

	
def lidar_node():
	rospy.init_node('lidar_node', anonymous=True)
	rospy.Subscriber('/scan',LaserScan, lidar_callback)
	rospy.spin()
	
if __name__ == '__main__':
	obstacle_distance_pub = rospy.Publisher('/obstacle_distance',Float32,queue_size=10)
	lidar_node()
