#!/usr/bin/env python


import rospy
import actionlib
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geographic_msgs.msg import GeoPoseStamped
from nav_msgs.msg import OccupancyGrid,Odometry
import pdb

#move_base_msgs
from move_base_msgs.msg import *


def simple_move(msg):
	global bottom_left,top_right,movements,geo_width,geo_height,pixel_width,pixel_height

	#convert to pixel space
	# pdb.set_trace()
	width_percent_geo=(msg.pose.position.longitude-bottom_left[1])/float(geo_width)
	pixel_x = pixel_width*width_percent_geo

	height_percent_geo = (msg.pose.position.latitude-bottom_left[0])/float(geo_height)
	pixel_y = pixel_height*height_percent_geo

	#add to movements
	pixel_x,pixel_y=pixel_x/10.,pixel_y/10.
	movements.append([pixel_x,pixel_y])


	pub_markers(marker_pub)

		

def pub_markers(marker_pub):
	global movements
	if(len(movements)>15):
		# print "here"
		markerArray = MarkerArray()

		for i,m in enumerate(movements):
			marker = Marker()
			marker.header.frame_id = "/map"
			marker.type = marker.SPHERE
			marker.action = marker.ADD
			marker.scale.x = 0.5
			marker.scale.y = 0.5
			marker.scale.z = 0.5
			marker.color.a = 1.0
			
			marker.color.r = 1.
			marker.color.g = 0.0
			marker.color.b = 0.0
			# else:
			# 	marker.color.r = 0.
			# 	marker.color.g = 1.0
			# 	marker.color.b = 0.0
			marker.pose.orientation.w = 1.0
			marker.pose.position.x = m[0]
			marker.pose.position.y = m[1] 
			marker.pose.position.z = 0.
			marker.id=i
			markerArray.markers.append(marker)

		marker_pub.publish(markerArray)



if __name__ == '__main__':
	global bottom_left,top_right,movements,geo_width,geo_height,pixel_width,pixel_height
	rospy.init_node('simple_move')

	bottom_left=[44.566881, -123.275972]
	top_right = [44.568019, -123.274166]

	geo_width=top_right[1]-bottom_left[1]
	geo_height=top_right[0]-bottom_left[0]

	pixel_width=968
	pixel_height=834

	# print lat_width,_height

	movements=[]

	marker_pub = rospy.Publisher('vis_m_array', MarkerArray,queue_size=1)
	robot_pub = rospy.Publisher('/odom', Odometry,queue_size=1)

	point_sub = rospy.Subscriber('/lat_lon_pose',GeoPoseStamped,simple_move)

	rate = rospy.Rate(10)
	rospy.sleep(0.5)
	while not rospy.is_shutdown():
		rate.sleep()
	# map_sub = rospy.Subscriber('/map',OccupancyGrid,update_map)

	# simple_move()