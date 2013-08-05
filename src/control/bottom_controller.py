#!/usr/bin/env python

import roslib; roslib.load_manifest('uav_target_tracking')
import rospy

from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

print "imports done"

trajectory = Twist()

def targetposCallback(msg):
    global trajectory

    target_pos = msg.pose.pose.position

    # all target position data in mm

    # For trajectory, +x forwards, -x backwards
    # for target location data, +y forwards, -y backwards
    if abs(target_pos.y) > 500:
        trajectory.linear.x = 0.8 if target_pos.y < 0 else -0.8
    else:
        trajectory.linear.x = -target_pos.y/600.

    # for trajectory, +y left, -y right
    # for loc, +x right, -x left
    if abs(target_pos.x) > 500:
        trajectory.linear.y = -0.8 if target_pos.x < 0 else 0.8
    else:
        trajectory.linear.y = target_pos.x/600.

    # For both trajectory and loc, +z up, -z down 
    zthresh = 700.
    z_offset = abs(zthresh - target_pos.z)
    if z_offset < 100:
        trajectory.linear.z = 0
    elif z_offset < 1200:
        trajectory.linear.z = (zthresh - target_pos.z)/zthresh
    else:
        trajectory.linear.z = 1 if target_pos.z < zthresh else -1

    print "trajectory.linear.x:", trajectory.linear.x
    print "trajectory.linear.y:", trajectory.linear.y
    print "trajectory.linear.z:", trajectory.linear.z

    controller_pub.publish(trajectory)


def navdataCallback(msg):
    global trajectory

    thresh = 600.
    z_offset = abs(thresh - msg.altd)
    if z_offset < 100:
           trajectory.linear.z = 0
    elif z_offset < 1200:
        trajectory.linear.z = (thresh - msg.altd)/thresh
    else:
        trajectory.linear.z = 1 if msg.altd < thresh else -1

    trajectory.linear.z = 0.0 + (thresh - msg.altd)/300.

    #trajectory.linear.z = (-msg.altd + thresh)/thresh

    #controller_pub.publish(trajectory)


if __name__ == "__main__":
    rospy.init_node("squaretarget_ardrone_controller")

    takeoff = rospy.Publisher('/ardrone/takeoff', Empty).publish
    land = rospy.Publisher('/ardrone/land', Empty).publish

    controller_pub = rospy.Publisher('/cmd_vel', Twist)

    rospy.sleep(4)

    print "pubs, subs created"

    takeoff()
    print "takeoff"

    import atexit
    atexit.register(land)

    traj = Twist()
    traj.linear.x = 0
    traj.linear.y = 0
    traj.linear.z = 0.0
    traj.angular.z = 0

    controller_pub.publish(traj)
    rospy.sleep(1)

    controller_pub.publish(traj)
    rospy.sleep(1)

    targetpos_sub = rospy.Subscriber('/ardrone_target', Odometry, targetposCallback)
    navdata_sub = rospy.Subscriber('/ardrone/navdata', Navdata, navdataCallback)

    #rospy.spin()
    rospy.sleep(120)

    land()
    print "landing"
