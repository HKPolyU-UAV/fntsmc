#! /usr/bin/python3
import os

import numpy as np
import rospy
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
import tf
from tf.transformations import quaternion_matrix


current_state = State()
pose = PoseStamped()  # position publish
uav_state = PoseStamped()  # position subscribe
uav_odom = Odometry()
ctrl_cmd = AttitudeTarget()
voltage = 11.4


def state_cb(msg):
	global current_state
	current_state = msg


def uav_odom_cb(msg: Odometry):
	uav_odom.pose.pose.position.x = msg.pose.pose.position.x
	uav_odom.pose.pose.position.y = msg.pose.pose.position.y
	uav_odom.pose.pose.position.z = msg.pose.pose.position.z
	uav_odom.pose.pose.orientation.x = msg.pose.pose.orientation.x
	uav_odom.pose.pose.orientation.y = msg.pose.pose.orientation.y
	uav_odom.pose.pose.orientation.z = msg.pose.pose.orientation.z
	uav_odom.pose.pose.orientation.w = msg.pose.pose.orientation.w

	uav_odom.twist.twist.linear.x = msg.twist.twist.linear.x
	uav_odom.twist.twist.linear.y = msg.twist.twist.linear.y
	uav_odom.twist.twist.linear.z = msg.twist.twist.linear.z
	uav_odom.twist.twist.angular.x = msg.twist.twist.angular.x
	uav_odom.twist.twist.angular.y = msg.twist.twist.angular.y
	uav_odom.twist.twist.angular.z = msg.twist.twist.angular.z


def uav_battery_cb(msg: BatteryState):
	global voltage
	voltage = msg.voltage
	# print(voltage)


if __name__ == "__main__":
	rospy.init_node("offb_node_py")  # 初始化一个节点

	'''topic subscribe'''
	# 订阅回来 uav 的 state 信息，包括连接状态，工作模式，电机的解锁状态
	state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)

	# subscribe the odom of the UAV
	uav_vel_sub = rospy.Subscriber("mavros/local_position/odom", Odometry, callback=uav_odom_cb)
	uav_battery_sub = rospy.Subscriber("mavros/battery", BatteryState, callback=uav_battery_cb)
	'''topic subscribe'''

	frequency = 100
	rate = rospy.Rate(frequency)

	while not rospy.is_shutdown():
		print('hhh', voltage)
		rate.sleep()
