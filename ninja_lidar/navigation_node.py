#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid
import rospkg

def map_callback(msg):
    # Once the map is received, set the start and destination positions and initiate navigation
    set_start_and_destination(msg)
    
def set_start_and_destination(map_msg):
    map_resolution = map_msg_info.resolution
    map_origin_x = map_msg.info.origin.position.x
    map_origin_y = map_msg.info.origin.position.y
    
    # Update the start position x & y co-ordinates
    start_x = 1.0
    start_y = 1.0
    start_orientation = 0.0
    
    # Update the destination position x & y co-ordinates
    dest_x = 5.0
    dest_y = 5.0
    dest_orientation = 0.0
    
    #Publish the start and destination positions as ROS messages
    goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
    
    start_pose = PoseStamped()
    start_pose.header.frame_id = "map"
    start_pose.pose.position.x = start_x
    start_pose.pose.position.y = start_y
    start_pose.pose.orientation.z = start_orientation
    
    dest_pose = PoseStamped()
    dest_pose.header.frame_id = "map"
    dest_pose.pose.position.x = dest_x
    dest_pose.pose.position.y = dest_y
    dest_pose.pose.orientation.z = dest_orientation
    
    goal_msg = MoveBaseActionGoal()
    goal_msg.goal.target_pose = dest_pose
    
    goal_pub.publish(goal_msg)
    
def main():
    rospy.init_node('autonomous_navigation_node', anonymous = True)
    
    #Subscribe to the map topic to receive the map in PGM format
    rospy.Subscriber('/map',OccupancyGrid, map_callback)
    
    rospy.spin()
    
if __name__ == '__main__':
    main()