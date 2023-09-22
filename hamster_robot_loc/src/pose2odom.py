#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

def pose_callback(msg, pub_odom):
	odom_msg = Odometry()
	odom_msg.header = msg.header
	odom_msg.pose = msg.pose
	pub_odom.publish(odom_msg)

def talker():
	rospy.init_node('pose2odom')
	pose_topic_name = rospy.get_param("~pose_topic_name", '/pose')
	odom_topic_name = rospy.get_param("~odom_topic_name", '/odom')
	
	# publisher def
	pub_odom = rospy.Publisher(odom_topic_name, Odometry, queue_size=10)
	sub_command = rospy.Subscriber(pose_topic_name, PoseWithCovarianceStamped, pose_callback, pub_odom)
	
	rospy.spin()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
