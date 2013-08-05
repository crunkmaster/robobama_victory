#!/usr/bin/env python
import roslib; roslib.load_manifest('robobama_victory')
import rospy
import sys

from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

trajectory = Twist()

def navdataCallback(msg):
	global trajectory

	drone_state = msg.state
	tags_count = msg.tags_count
	tags_xc = msg.tags_xc
	tags_yc = msg.tags_yc
	tags_distance = msg.tags_distance

	thresh = 10
	distance_thresh = 10
	
	# basically hover if you don't see the target
	if tags_count != 1:
		trajectory.linear.y = 0
		trajectory.linear.x = 0
		trajectory.linear.z = 0
		flight_pub.publish(trajectory)

	try:
		x_pos = tags_xc[0] * .64 
		y_pos = tags_yc[0] * .36
		tags_distance = tags_distance[0]
		print "tags_distance: {0}".format(tags_distance)

		# these are distances from the middle of the camera view
		x_distance = abs(320 - x_pos)
		y_distance = abs(230 - y_pos)
		# this is keeping it 100 units away from the target
		tag_offset = abs(80 - tags_distance)

		# centering the target in the left / right direction
		if x_distance > thresh:
			if x_pos > 320:
				trajectory.linear.y = -0.1
				print "going right"
			if x_pos < 320:
				trajectory.linear.y = 0.1
				print "going left"
		else:
			trajectory.linear.y = 0.0
			print "centered left / right"
		
		# #centering the target in the up / down direction
		# if y_distance > thresh:
		#     if y_pos > 240:
		#         trajectory.linear.z = -0.2
		#         print "going up"
		#     if y_pos < 240:
		#         trajectory.linear.z = 0.3
		#         print "going down"
		# else:
		#     trajectory.linear.z = 0.0
		#     print "centered up / down"
		
		# maintaining proper distance from the target
		if tag_offset > distance_thresh:
		    if tags_distance < 80:
		        trajectory.linear.x = -0.2
		        print "moving away"
		    if tags_distance > 80:
		        trajectory.linear.x = 0.2
		        print "moving closer"
		else:
		    trajectory.linear.x = 0.0

	except IndexError:
		print "can't see target"
		pass

	flight_pub.publish(trajectory)

if __name__ == "__main__":
	rospy.init_node("drone_controller")

	# create the takeoff and landing commands
	takeoff = rospy.Publisher('/ardrone/takeoff', Empty).publish
	land = rospy.Publisher('/ardrone/land', Empty).publish

	flight_pub = rospy.Publisher('/cmd_vel', Twist)
	
	rospy.sleep(4)
	#takeoff()
	print "taking off"

	# this atexit stuff doesn't even work, so don't bother.
	import atexit
	atexit.register(land) # make sure on exit, the drone will land

	# start target tracking using navdata callback function
	navdata_sub = rospy.Subscriber('/ardrone/navdata', Navdata,
								   navdataCallback)
	rospy.sleep(120)
	print "landing"
	land()
