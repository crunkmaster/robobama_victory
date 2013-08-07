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

        thresh = 20
        distance_thresh = 20

        # initialize all trajectories to zero
        # this theoretically will make it easy to change only
        # one variable of motion at a time.
        trajectory.linear.x = 0.0
        trajectory.linear.y = 0.0
        trajectory.linear.z = 0.0

        # basically hover if you don't see the target
        if tags_count != 1:
                flight_pub.publish(trajectory)
                return

        # multiplied by a constant to translate target_coordinates
        # from 1000x1000 frame to 640x360 frame
        x_pos = tags_xc[0] * .64
        y_pos = tags_yc[0] * .36

        tags_distance = tags_distance[0]

        print "tags_distance: {0}".format(tags_distance)
        # these are distances from the middle of the camera's
        # 640x360 view
        x_distance = abs(320 - x_pos)
        y_distance = abs(180 - y_pos)
        
        tag_offset = abs(80 - tags_distance)
        distances = {'x': x_distance, 'y': y_distance, 
                     'tag': tags_distance}
        
        ## essentially, what this code does is select which variable
        ## needs the greatest adjustment, and then only generate a trajectory
        ## based on that variable...
        ## this adjustment variable is determined at each call of navdataCallback
        ## or per each image message received from the drone...
        adjustment_variable = max(distance, key = distance.get)

        if adjustment_variable == 'x' and x_distance > thresh:
                if x_pos > 320:
                        trajectory.linear.y = -0.1
                if x_pos < 320:
                        trajectory.linear.y = 0.1
        elif adjustment_variable == 'y' and y_distance > thresh:
                if y_pos > 180:
                        trajectory.linear.z = -0.2
                if y_pos < 180:
                        trajectory.linear.z = 0.3
        elif adjustment_variable == 'tag' and tag_offset > distance_thresh:
                if tags_distance > 80:
                        trajectory.linear.x = 0.1
                if tags_distance < 80:
                        trajectory.linear.x = -0.1
                        
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

        # this should hopefully make it so a keyboard interrupt will
        # land the drone...
        try:
                # start target tracking using navdata callback function
                navdata_sub = rospy.Subscriber('/ardrone/navdata', Navdata,
                                               navdataCallback)
                rospy.spin()
        except KeyboardInterrupt:
                print "landing"
                land()
                
