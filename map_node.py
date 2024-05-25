#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
import rospkg

def publish_map():
    rospy.init_node('map_publisher_node', anonymous=True)
    
    map_file = "path_to_pgn_file"
    map_data = open(map_file, "r").read()
    
    #Create map message
    map_msg = OcccupancyGrid()
    map_msg.header.stamp = rospy.Time.now()
    map_msg.header.frame_id = "map"
    map_msg.info.resolution = 0.05 #map resolution

    map_msg.info.width = 400 #map width
    map_msg.info.height = 400 #map heigth

    map_msg.info.origin.position.x = 0.0
    map_msg.info.origin.position.y = 0.0
    map_msg.info.origin.position.z = 0.0


    map_msg.data = list(map(ord,map_data)) # This is to set the map data

    #Publish the map
    map_pub = rospy.Publisher('/map' ,OccupancyGrid, queue_size= 10)
    map_pub.publish(map_msg)


if __name__ == '__main__':
    try:
        publish_map()
    except rospy.ROSInterruptException:
        pass