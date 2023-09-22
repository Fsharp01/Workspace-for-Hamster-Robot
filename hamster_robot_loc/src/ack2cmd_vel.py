#!/usr/bin/env python3

import rospy
import math
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TwistStamped

def cmd_callback(msg, pub_cmd):
	twist_msg = TwistStamped()
	twist_msg.header = msg.header
	twist_msg.twist.linear.x = msg.drive.speed*math.cos(msg.drive.steering_angle)
	twist_msg.twist.linear.y = msg.drive.speed*math.sin(msg.drive.steering_angle)
	twist_msg.twist.angular.z = math.atan2(twist_msg.twist.linear.y,twist_msg.twist.linear.x)
	
	pub_cmd.publish(twist_msg)

def talker():
	rospy.init_node('ack2cmd_vel')
	ackermann_topic_name = rospy.get_param("~ackermann_topic_name", '/ackermann_cmd')
	cmd_vel_topic_name = rospy.get_param("~cmd_vel_topic_name", '/cmd_vel')
	
	# publisher def
	pub_cmd = rospy.Publisher(cmd_vel_topic_name, TwistStamped, queue_size=10)
	sub_ack = rospy.Subscriber(ackermann_topic_name, AckermannDriveStamped, cmd_callback, pub_cmd)
	
	rospy.spin()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
