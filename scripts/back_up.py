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

from smc_ctrl.uav_ROS import UAV_ROS
from smc_ctrl.controller import ctrl_out, data_collector, ctrl_in2
from smc_ctrl.NESO import neso_in, neso_out
from smc_ctrl.ref_cmd import *
from smc_ctrl.utils import *


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


def uav_odom_2_uav_state(odom: Odometry) -> np.ndarray:
	_orientation = odom.pose.pose.orientation
	_w = _orientation.w
	_x = _orientation.x
	_y = _orientation.y
	_z = _orientation.z
	rpy = tf.transformations.euler_from_quaternion([_x, _y, _z, _w])
	_uav_state = np.array([
		odom.pose.pose.position.x,  # x
		odom.pose.pose.position.y,  # y
		odom.pose.pose.position.z,  # z
		odom.twist.twist.linear.x,  # vx
		odom.twist.twist.linear.y,  # vy
		odom.twist.twist.linear.z,  # vz
		rpy[0],  # phi
		rpy[1],  # theta
		rpy[2],  # psi
		odom.twist.twist.angular.x,  # p
		odom.twist.twist.angular.y,  # q
		odom.twist.twist.angular.z  # r
	])
	return _uav_state


def thrust_2_throttle(thrust: float):
	"""

	"""
	# m = np.array([0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2, 3.4, 3.6, 3.7])
	# y = np.array([0.257, 0.330, 0.391, 0.446, 0.497, 0.543, 0.585, 0.626, 0.665, 0.703, 0.736, 0.771, 0.803, 0.835, 0.865, 0.894, 0.921, 0.937])

	'''gazebo 三次曲线模型'''
	# a = 0.1150
	# b = 0.0405
	# c = -7.8859e-4
	# d = 8.2424e-06
	# _throttle = a + b * thrust + c * thrust ** 2 + d * thrust ** 3
	# _throttle = max(min(_throttle, 0.95), 0.10)
	'''gazebo 三次曲线模型'''

	'''gazebo 线性模型'''
	k = 0.06307977736549165
	_throttle = k * thrust
	_throttle = max(min(_throttle, 0.9), 0.10)
	'''gazebo 线性模型'''

	'''姜百伦飞机模型'''
	# _m = 0.453	# uav 质量
	# _g = 9.8
	# # voltage = 12.0
	# _y = -0.0426229508 * voltage + 0.7269726774		# 不同电压下的悬停油门
	# _y = min(max(_y, 0.21), 0.27)
	# k = _y / (_m * _g)
	# _throttle = k * thrust
	# _throttle = max(min(_throttle, 0.4), 0.10)
	'''姜百伦飞机模型'''
	return _throttle


current_state = State()
pose = PoseStamped()  # position publish
uav_state = PoseStamped()  # position subscribe
uav_odom = Odometry()
ctrl_cmd = AttitudeTarget()
voltage = 11.4
global_flag = 0


if __name__ == "__main__":
	rospy.init_node("offb_node_py")  # 初始化一个节点

	'''topic subscribe'''
	# 订阅回来 uav 的 state 信息，包括连接状态，工作模式，电机的解锁状态
	state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)

	# # subscribe the position of the UAV
	# uav_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=uav_pos_cb)

	# subscribe the odom of the UAV
	uav_vel_sub = rospy.Subscriber("mavros/local_position/odom", Odometry, callback=uav_odom_cb)
	uav_battery_sub = rospy.Subscriber("mavros/battery", BatteryState, callback=uav_battery_cb)
	'''topic subscribe'''

	local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
	uav_att_throttle_pub = rospy.Publisher("mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)
	'''Publish 位置指令给 UAV'''

	'''arming service'''
	rospy.wait_for_service("/mavros/cmd/arming")  # 等待解锁电机的 service 建立
	arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

	'''working mode service'''
	rospy.wait_for_service("/mavros/set_mode")  # 等待设置 UAV 工作模式的 service 建立
	set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

	frequency = 100
	rate = rospy.Rate(frequency)

	'''如果没有连接上，就等待'''
	while (not rospy.is_shutdown()) and (not current_state.connected):
		rate.sleep()

	pose.pose.position.x = uav_odom.pose.pose.position.x
	pose.pose.position.y = uav_odom.pose.pose.position.y
	pose.pose.position.z = uav_odom.pose.pose.position.z

	for i in range(100):
		if rospy.is_shutdown():
			break
		local_pos_pub.publish(pose)
		rate.sleep()

	offb_set_mode = SetModeRequest()  # 先设置工作模式为 offboard
	offb_set_mode.custom_mode = 'OFFBOARD'

	arm_cmd = CommandBoolRequest()
	arm_cmd.value = True  # 通过指令将电机解锁

	while (current_state.mode != "OFFBOARD") and (not rospy.is_shutdown()):  # 等待
		if set_mode_client.call(offb_set_mode).mode_sent:
			print('Switching to OFFBOARD mode is available...waiting for 1 seconds')
			break
		local_pos_pub.publish(pose)
		rate.sleep()

	t_now = rospy.Time.now()

	while rospy.Time.now() - t_now < rospy.Duration(int(1.0)):
		local_pos_pub.publish(pose)
		rate.sleep()

	while (not current_state.armed) and (not rospy.is_shutdown()):
		if arming_client.call(arm_cmd).success:
			print('UAV is armed now...waiting for 1 seconds')
			break
		local_pos_pub.publish(pose)
		rate.sleep()

	t_now = rospy.Time.now()

	while rospy.Time.now() - t_now < rospy.Duration(int(1.0)):
		local_pos_pub.publish(pose)
		rate.sleep()

	print('Start......')

	# ref_amplitude = np.array([1.5, 1.5, 0.5, 0])  # xd yd zd psid 振幅
	ref_amplitude = np.array([0, 0, 0.5, 0])  # xd yd zd psid 振幅
	ref_period = np.array([5, 5, 5, 10])  # xd yd zd psid 周期
	ref_bias_a = np.array([1, 1, 1, 0])  # xd yd zd psid 幅值偏移
	ref_bias_phase = np.array([np.pi / 2, 0, 0, 0])  # xd yd zd psid 相位偏移

	'''無人機先定點'''
	print('Position...')
	READY = False
	cnt = 0
	ref, _, _, _ = ref_uav(0., ref_amplitude, ref_period, ref_bias_a, ref_bias_phase)
	while (not READY) and (not rospy.is_shutdown()):
		pose.pose.position.x = 0  # ref[0]
		pose.pose.position.y = 0  # ref[1]
		pose.pose.position.z = 1  # ref[2]
		uav_pos = uav_odom_2_uav_state(uav_odom)[0: 3]
		if np.linalg.norm(ref[0: 3] - uav_pos) < 0.2:
			cnt += 1
		else:
			if cnt < 200:
				cnt = 0
		if cnt >= 200:
			READY = True
		local_pos_pub.publish(pose)
		rate.sleep()
	print('OFFBOARD...')
	'''無人機先定點'''

	'''initialization of the controller'''
	t0 = rospy.Time.now().to_sec()  # 这是整体仿真的开始时刻
	uav_ros = UAV_ROS(m=0.797, g=9.8, kt=1e-3, dt=1 / frequency)  # 作为虚拟的模型类，不参与运算，只是提供控制器中必要的数据

	ctrl_out = ctrl_out(k1=np.array([0.6, 0.6]),  # 1.2, 0.8
						k2=np.array([0.3, 0.3]),  # 0.4, 0.6
						k3=np.array([0.05, 0.05]),
						alpha=np.array([1.2, 1.2]),
						beta=np.array([0.7, 0.7]),  # 0.3, 0.3 從0.3改成0.7之後，效果明顯提升
						gamma=np.array([0.2, 0.2]),
						lmd=np.array([1.0, 1.0]),  # 2, 2
						dt=uav_ros.dt)  # 外环控制器 x y

	ctrl_in = ctrl_in2(ctrl0=uav_ros.m * uav_ros.g,
					   k1=0.4,
					   k2=0.4,
					   alpha=1.2,
					   beta=0.7,
					   gamma=0.2,
					   lmd=2.0,
					   dt=1 / frequency)

	obs_out = neso_out(l1=np.array([3., 3.]),
					   l2=np.array([3., 3.]),
					   l3=np.array([1., 1.]),
					   r=np.array([20., 20.]),  # 20， 20
					   k1=np.array([0.7, 0.7]),
					   k2=np.array([0.001, 0.001]))  # x y
	obs_in = neso_in(l1=3., l2=3., l3=1., r=5., k1=0.7, k2=0.001)  # z

	data_record = data_collector(N=int(uav_ros.time_max / uav_ros.dt) + 1)

	ref, dot_ref, _, _ = ref_uav(rospy.Time.now().to_sec() - t0, ref_amplitude, ref_period, ref_bias_a, ref_bias_phase)  # 整体参考信号 xd yd zd psid
	phi_d = 0.
	theta_d = 0.
	rhod = ref[2]  # zd
	dot_rhod = dot_ref[2]  # dot_zd
	uav_ros.rk44(action=ctrl_in.control, uav_state=uav_odom_2_uav_state(uav_odom))  # 将 ROS 中读到的 UAV 状态传入 uav_ros 类中
	ei = uav_ros.rho1() - rhod
	dei = uav_ros.dot_rho1() - dot_rhod
	syst_dynamic0 = (uav_ros.dot_f1_rho1() * uav_ros.rho2() +
					 uav_ros.f1_rho1() * uav_ros.f2_rho2() +
					 uav_ros.f1_rho1() * uav_ros.h_rho1() * ctrl_in.control)
	obs_in.set_init(x0=ei, dx0=dei, syst_dynamic=syst_dynamic0)
	obs_out.set_init(x0=uav_ros.eta(), dx0=uav_ros.dot_eta(), syst_dynamic=ctrl_out.ctrl)

	data_block = {'time': uav_ros.time,  # time
				  'ctrl_in': ctrl_in.control,  # thrust
				  'ref_angle': np.array([phi_d, theta_d, ref[3]]),  # phid thetad psid
				  'ref_pos': ref[0: 3],  # xd yd zd
				  'd_in_obs': np.array([0.0]),  # dz
				  'd_out_obs': np.array([0., 0.]),  # dx dy
				  'state': uav_ros.uav_state_call_back()}  # 12 状态
	data_record.record(data=data_block)
	'''initialization of the controller'''

	while not rospy.is_shutdown():
		t_now = round(rospy.Time.now().to_sec() - t0, 4)
		if uav_ros.n % 1000 == 0:
			print('time: ', t_now)

		'''1. generate reference command and uncertainty'''
		ref, dot_ref, dot2_ref, dot3_ref = ref_uav(uav_ros.time, ref_amplitude, ref_period, ref_bias_a, ref_bias_phase)  # 整体参考信号 xd yd zd psid

		'''2. generate outer-loop reference signal 'eta_d' and its 1st, 2nd, and 3rd-order derivatives'''
		eta_d = np.array([ref[0], ref[1]])  # 外环参考
		dot_eta_d = np.array([dot_ref[0], dot_ref[1]])  # 外环参考一阶导
		dot2_eta_d = np.array([dot2_ref[0], dot2_ref[1]])  # 外环参考二阶导
		dot3_eta_d = np.array([dot3_ref[0], dot3_ref[1]])  # 外环参考三阶导

		'''3. generate inner-loop reference signal 'rho_d' and its 1st and 2nd-order derivatives'''
		phi_d_old = phi_d
		theta_d_old = theta_d

		phi_d, theta_d = uo_2_ref_angle(ctrl_out.ctrl, ref[3], uav_ros.m, ctrl_in.control, np.pi / 5)

		rhod = ref[2]  # zd
		dot_rhod = dot_ref[2]  # zd 的一阶导数
		dot2_rhod = dot2_ref[2]  # zd的二阶导数

		'''4. compute the error 'ei' and its 1st and 2nd-order derivatives'''
		ei = uav_ros.rho1() - rhod
		dei = uav_ros.dot_rho1() - dot_rhod

		OBSERVER_IN = True
		OBSERVER_OUT = False

		if OBSERVER_IN:
			syst_dynamic = uav_ros.dot_f1_rho1() * uav_ros.rho2() + \
						   uav_ros.f1_rho1() * uav_ros.f2_rho2() + \
						   uav_ros.f1_rho1() * uav_ros.h_rho1() * ctrl_in.control - dot2_rhod  # '-dot2_rhod' 这一项是ROS程序特有的
			delta_inner_obs, _ = obs_in.observe(dt=uav_ros.dt, x=ei, syst_dynamic=syst_dynamic)  #
		else:
			delta_inner_obs = -dot2_rhod

		ddei = (uav_ros.dot_f1_rho1() * uav_ros.rho2() +
				uav_ros.f1_rho1() * (uav_ros.f2_rho2() + uav_ros.h_rho1() * ctrl_in.control) +
				delta_inner_obs)

		# '''5. compute the derivative of the inner-loop controller'''
		# ctrl_in.dot_control(ei, dei, ddei,
		# 					uav_ros.f1_rho1(), uav_ros.f2_rho2(), uav_ros.rho2(), uav_ros.h_rho1(), uav_ros.dot_f1_rho1(), uav_ros.dot_Frho2_f1f2(), uav_ros.dot_f1g(),
		# 					delta_inner_obs)

		'''6. get new uav states from Gazebo'''
		uav_ros.rk44(action=ctrl_in.control, uav_state=uav_odom_2_uav_state(uav_odom))

		'''7. compute control output of the inner-loop subsystem for next time step'''
		ctrl_in.control_update(m=uav_ros.m,
							   g=uav_ros.g,
							   phi=uav_ros.phi,
							   theta=uav_ros.theta,
							   kp=uav_ros.kt,
							   dz=uav_ros.vz,
							   dot2_zd=dot2_rhod,
							   e=ei,
							   de=dei,
							   delta_obs=delta_inner_obs)

		'''8. compute the virtual control output of the outer-loop subsystem'''
		if OBSERVER_OUT:
			syst_dynamic = -uav_ros.kt / uav_ros.m * uav_ros.dot_eta() + uav_ros.A()
			delta_outer_obs, dot_delta_outer_obs = obs_out.observe(dt=uav_ros.dt, x=uav_ros.eta(), syst_dynamic=syst_dynamic)
		else:
			delta_outer_obs = np.array([0., 0.])
		dot_eta = uav_ros.dot_eta()
		e_o = uav_ros.eta() - eta_d
		dot_e_o = dot_eta - dot_eta_d
		ctrl_out.control(e=e_o, de=dot_e_o, dd_ref=dot2_eta_d, obs=delta_outer_obs)

		'''9. data storage'''
		data_block = {'time': np.round(uav_ros.time, 3),  # time
					  'ctrl_in': ctrl_in.control,  # thrust
					  'ref_angle': np.array([phi_d, theta_d, ref[3]]),  # phid thetad psid
					  'ref_pos': ref[0: 3],  # xd yd zd
					  'd_in_obs': delta_inner_obs,  # dz
					  'd_out_obs': delta_outer_obs,  # dx dy
					  'state': uav_ros.uav_state_call_back()}  # 12 状态
		data_record.record(data=data_block)

		print('==========START==========')
		print('time: %.3f' % uav_ros.time)
		print(' x_REF    x    || Theta_ref')
		print(' %.2f   %.2f  ||  %.2f' % (ref[0], uav_ros.x, theta_d * 180 / np.pi))
		print(' y_REF    y    ||  Phi_ref')
		print(' %.2f   %.2f  ||  %.2f' % (ref[1], uav_ros.y, phi_d * 180 / np.pi))
		print(' z_REF    z    || thrust   PWM')
		print(' %.2f   %.2f   ||  %.2f   %.2f' % (ref[2], uav_ros.z, ctrl_in.control, thrust_2_throttle(ctrl_in.control)))
		print('===========END===========\n')

		'''10. 将期望姿态角和油门推力转换到 ROS topic 下边'''
		ctrl_cmd.header.stamp = rospy.Time.now()
		ctrl_cmd.type_mask = AttitudeTarget.IGNORE_ROLL_RATE + AttitudeTarget.IGNORE_PITCH_RATE + AttitudeTarget.IGNORE_YAW_RATE
		# phi_d = 0.
		# theta_d = 0.
		# ref[3] = np.pi / 2 * np.sin()
		cmd_q = tf.transformations.quaternion_from_euler(phi_d, theta_d, ref[3])  # x y z w
		# cmd_q = np.array([0., 0., 0., 1.])	# x y z w
		ctrl_cmd.orientation.x = cmd_q[0]
		ctrl_cmd.orientation.y = cmd_q[1]
		ctrl_cmd.orientation.z = cmd_q[2]
		ctrl_cmd.orientation.w = cmd_q[3]
		ctrl_cmd.thrust = thrust_2_throttle(ctrl_in.control)
		# ctrl_cmd.thrust = 0.3

		# pose.pose.position.x = 0
		# pose.pose.position.y = 0
		# pose.pose.position.z = 3
		# local_pos_pub.publish(pose)		# 重点

		uav_att_throttle_pub.publish(ctrl_cmd)
		rate.sleep()

		if data_record.index == data_record.N:
			print('Data collection finish. Exit...')
			data_record.package2file(path=os.getcwd() + '/src/control/scripts/datasave/')
			break
	print('Simulation termination...')
