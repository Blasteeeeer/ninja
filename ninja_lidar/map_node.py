#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
import yaml


def load_map_data():
    map_data = OccupancyGrid()
    # with open('/home/ninja/catkin_ws/src/maps/take_2.yaml','r') as yaml_file:
    #     map_info = yaml_file.safe_load(yaml_file)
    #     map_data.info.resolution = map_info['resolution']
    #     map_data.info.origin.position.x  = map_info['origin'][0]
    #     map_data.info.origin.position.y  = map_info['origin'][1]
    #     map_data.info.origin.position.z  = 0.0
    #     map_data.info.origin.orientation.x  = 0.0
    #     map_data.info.origin.orientation.y  = 0.0
    #     map_data.info.origin.orientation.z  = map_info['origin'][2]
    #     map_data.info.origin.orientation.z  = map_info['origin'][3]

    map_data.info.resolution = 0.050000
    map_data.info.origin.position.x  = -51.224998
    map_data.info.origin.position.y  = -51.224998
    map_data.info.origin.position.z  = 0.0
    map_data.info.origin.orientation.x  = 0.0
    map_data.info.origin.orientation.y  = 0.0
    map_data.info.origin.orientation.z  = 0.0
    map_data.info.origin.orientation.w  = 0.0

    with open('/home/ninja/catkin_ws/src/maps/take_2.pgm','rb') as pgm_file:
        map_data.data = list(pgm_file.read())

    return map_data

def map_publisher_node():
    rospy.init_node('map_node', anonymous=True)
    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
    rate = rospy.Rate(1)  # Publish rate

    while not rospy.is_shutdown():
        map_data = load_map_data()
        map_pub.publish(map_data)
        rate.sleep()

if __name__ == '__main__':
    try:
        map_publisher_node()
    except rospy.ROSInterruptException:
        pass

        
# def publish_map():
#     rospy.init_node('map_publisher_node', anonymous=True)
    
#     map_file = "path_to_pgn_file"
#     map_data = open(map_file, "r").read()
    
#     #Create map message
#     map_msg = OcccupancyGrid()
#     map_msg.header.stamp = rospy.Time.now()
#     map_msg.header.frame_id = "map"
#     map_msg.info.resolution = 0.05 #map resolution

#     map_msg.info.width = 400 #map width
#     map_msg.info.height = 400 #map heigth

#     map_msg.info.origin.position.x = 0.0
#     map_msg.info.origin.position.y = 0.0
#     map_msg.info.origin.position.z = 0.0


#     map_msg.data = list(map(ord,map_data)) # This is to set the map data

#     #Publish the map
#     map_pub = rospy.Publisher('/map' ,OccupancyGrid, queue_size= 10)
#     map_pub.publish(map_msg)


# if __name__ == '__main__':
#     try:
#         publish_map()
#     except rospy.ROSInterruptException:
#         pass