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
from smc_ctrl.controller import ctrl_out, data_collector, ctrl_in ,ctrl_in2, ctrl_in3
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
	'''gazebo 线性模型'''
	# k = 0.391 / 0.797 / 9.8
	# _throttle = k * thrust
	# _throttle = max(min(_throttle, 0.9), 0.10)
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

	'''330飞机模型'''
	_m = 0.797	# uav 质量
	_g = 9.8
	_y = -0.0181 * voltage + 0.4854		# 不同电压下的悬停油门
	_y = min(max(_y, 0.19), 0.23)
	k = _y / (_m * _g)
	_throttle = k * thrust
	_throttle = max(min(_throttle, 0.6), 0.10)
	'''330飞机模型'''
	return _throttle


def approaching(_t: float, ap_flag: bool, threshold: float):
	ref_amplitude = np.array([0., 0., 0., 0.])
	ref, _, _, _ = ref_uav(0., ref_amplitude, ref_period, ref_bias_a, ref_bias_phase)
	pose.pose.position.x = ref[0]
	pose.pose.position.y = ref[1]
	pose.pose.position.z = ref[2]
	uav_pos = uav_odom_2_uav_state(uav_odom)[0: 3]
	local_pos_pub.publish(pose)
	_bool = False
	if np.linalg.norm(ref[0: 3] - uav_pos) < 0.25:
		if ap_flag:
			_bool = True if rospy.Time.now().to_sec() - _t >= threshold else False
		else:
			ap_flag = True
	else:
		_bool = False
		ap_flag = False
	return ap_flag, _bool


'''Some pre-defined parameters'''
current_state = State()			# monitor uav status
pose = PoseStamped()			# publish offboard [x_d y_d z_d] cmd
uav_odom = Odometry()			# subscribe uav state x y z vx vy vz phi theta psi p q r
ctrl_cmd = AttitudeTarget()		# publish offboard expected [phi_d theta_d psi_d throttle] cmd
voltage = 11.4					# subscribe voltage from the battery
global_flag = 0					# UAV working mode monitoring
# UAV working mode
# 0: connect to onboard computer, arm, load parameters, prepare
# 1: approaching and initialization
# 2: control by SMC ([phi_d theta_d psi_d throttle])
# 3: finish and switch to OFFBOARD-position
'''Some pre-defined parameters'''


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

	t0 = rospy.Time.now().to_sec()

	while rospy.Time.now().to_sec() - t0 < 1.0:
		local_pos_pub.publish(pose)
		rate.sleep()

	while (not current_state.armed) and (not rospy.is_shutdown()):
		if arming_client.call(arm_cmd).success:
			print('UAV is armed now...waiting for 1 seconds')
			break
		local_pos_pub.publish(pose)
		rate.sleep()

	t0 = rospy.Time.now().to_sec()

	while rospy.Time.now().to_sec() - t0 < 1.0:	# OK
		local_pos_pub.publish(pose)
		rate.sleep()

	print('Start......')
	print('Approaching...')
	global_flag = 1

	'''generate reference command'''
	# ref_amplitude = np.array([0., 0., 0., 0])  # xd yd zd psid 振幅
	ref_amplitude = np.array([1.5, 1.5, 0.3, 0])  # xd yd zd psid 振幅
	ref_period = np.array([6, 6, 10, 10])  # xd yd zd psid 周期
	ref_bias_a = np.array([0, 0, 1.0, 0])  # xd yd zd psid 幅值偏移
	ref_bias_phase = np.array([np.pi / 2, 0, 0, 0])  # xd yd zd psid 相位偏移
	'''generate reference command'''

	t0 = rospy.Time.now().to_sec()
	approaching_flag = False

	'''define controllers and observers'''
	uav_ros = None
	c_out = None
	c_in = None
	obs_out = None
	obs_in = None
	data_record = None
	'''define controllers and observers'''
	# global_flag = 2
	while not rospy.is_shutdown():
		t = rospy.Time.now().to_sec()
		if global_flag == 1:		# approaching
			approaching_flag, ok = approaching(t0, approaching_flag, 10.0)
			# ok = True
			if ok:
				print('OFFBOARD, start to initialize...')
				uav_ros = UAV_ROS(m=0.797, g=9.8, kt=1e-3, dt=1 / frequency)
				c_out = ctrl_out(k1=np.array([0.7, 0.7]),  # 越大，角度响应越猛烈
								 k2=np.array([0.6, 0.6]),  # 
								 k3=np.array([0.05, 0.05]),
								 alpha=np.array([1.2, 1.2]),
								 beta=np.array([0.7, 0.7]),  # 越大，角度相应越猛烈
								 gamma=np.array([1.0, 1.0]),
								 lmd=np.array([1.3, 1.3]),  # 2, 2 TODO 积分太大了
								 dt=uav_ros.dt)  # 外环控制器 x y

				c_in = ctrl_in2(ctrl0=uav_ros.m * uav_ros.g,
								k1=4.0,
								k2=0.8,
								alpha=1.2,
								beta=0.5,
								gamma=0.6,
								lmd=1.2,
								dt=1 / frequency)
				# c_in = ctrl_in3(ctrl0=uav_ros.m * uav_ros.g,
				# 				k=4,
				# 				k0=0,
				# 				beta=1.0,
				# 				c=2,
				# 				p=17,
				# 				q=13,
				# 				m=5,
				# 				n=5,
				# 				dt=1 / frequency)

				obs_out = neso_out(l1=np.array([3., 3.]),
								   l2=np.array([3., 3.]),
								   l3=np.array([1., 1.]),
								   r=np.array([20., 20.]),  # 20， 20
								   k1=np.array([0.7, 0.7]),
								   k2=np.array([0.001, 0.001]))  # x y
				obs_in = neso_in(l1=3., l2=3., l3=1., r=5., k1=0.7, k2=0.001)  # z
				data_record = data_collector(N=int(uav_ros.time_max * frequency) + 1)
				# print(data_record.N)
				print('Control...')
				t0 = rospy.Time.now().to_sec()
				global_flag = 2
		elif global_flag == 2:		# control
			t_now = round(t - t0, 4)
			if uav_ros.n % 100 == 0:
				print('time: ', t_now, data_record.index)

			'''1. generate reference command and uncertainty'''
			rax = max(min(0.5 * t_now, 1.5), 0.0)
			ray = max(min(0.5 * t_now, 1.5), 0.0)
			raz = max(min(0.06 * t_now, 0.0), 0.0)
			rapsi = max(min(0.5 * t_now, 0), 0.0)
			ref_amplitude = np.array([rax, ray, ray, rapsi])
			ref, dot_ref, dot2_ref, dot3_ref = ref_uav(t_now,
													   ref_amplitude,
													   ref_period,
													   ref_bias_a,
													   ref_bias_phase)

			'''2. generate outer-loop reference signal 'eta_d' and its 1st, 2nd, and 3rd-order derivatives'''
			eta_d = np.array([ref[0], ref[1]])  # 外环参考
			dot_eta_d = np.array([dot_ref[0], dot_ref[1]])  # 外环参考一阶导
			dot2_eta_d = np.array([dot2_ref[0], dot2_ref[1]])  # 外环参考二阶导
			# dot3_eta_d = np.array([dot3_ref[0], dot3_ref[1]])  # 外环参考三阶导

			'''3. generate inner-loop reference signal 'rho_d' and its 1st and 2nd-order derivatives'''
			phi_d, theta_d = uo_2_ref_angle(c_out.ctrl, ref[3], uav_ros.m, c_in.control, np.pi / 4)
			rhod = ref[2]  # zd
			dot_rhod = dot_ref[2]  # zd 的一阶导数
			dot2_rhod = dot2_ref[2]  # zd的二阶导数

			ctrl_cmd.header.stamp = rospy.Time.now()
			ctrl_cmd.type_mask = AttitudeTarget.IGNORE_ROLL_RATE + AttitudeTarget.IGNORE_PITCH_RATE + AttitudeTarget.IGNORE_YAW_RATE

			cmd_q = tf.transformations.quaternion_from_euler(phi_d, theta_d, ref[3])  # x y z w

			'''4. compute the error 'ei' and its 1st and 2nd-order derivatives'''
			ei = uav_ros.rho1() - rhod
			dei = uav_ros.dot_rho1() - dot_rhod

			OBSERVER_IN = False
			OBSERVER_OUT = False

			if OBSERVER_IN:
				syst_dynamic = uav_ros.dot_f1_rho1() * uav_ros.rho2() + \
							   uav_ros.f1_rho1() * uav_ros.f2_rho2() + \
							   uav_ros.f1_rho1() * uav_ros.h_rho1() * c_in.control - dot2_rhod
				delta_inner_obs, _ = obs_in.observe(dt=uav_ros.dt, x=ei, syst_dynamic=syst_dynamic)  #
			else:
				delta_inner_obs = 0.

			ddei = (uav_ros.dot_f1_rho1() * uav_ros.rho2() +
					uav_ros.f1_rho1() * (uav_ros.f2_rho2() + uav_ros.h_rho1() * c_in.control) +
					delta_inner_obs)

			# c_in.c = c_in.c0 * np.tanh(np.fabs(30 * ei))
			# c_in.beta = c_in.beta0 * np.tanh(np.fabs(5 * dei))

			c_in.control_update(m=uav_ros.m,
								g=uav_ros.g,
								phi=uav_ros.phi,
								theta=uav_ros.theta,
								kp=uav_ros.kt,
								dz=uav_ros.vz,
								dot2_zd=dot2_rhod,
								dot_zd=dot_rhod,
								e=ei,
								de=dei,
								delta_obs=delta_inner_obs)
			k_compensate_z_acc = 0.9
			c_in.control += k_compensate_z_acc * dot2_ref[2]

			ctrl_cmd.orientation.x = cmd_q[0]
			ctrl_cmd.orientation.y = cmd_q[1]
			ctrl_cmd.orientation.z = cmd_q[2]
			ctrl_cmd.orientation.w = cmd_q[3]
			ctrl_cmd.thrust = thrust_2_throttle(c_in.control)
			uav_att_throttle_pub.publish(ctrl_cmd)

			# pose.pose.position.x = ref[0]
			# pose.pose.position.y = ref[1]
			# pose.pose.position.z = ref[2]
			# local_pos_pub.publish(pose)

			'''5. get new uav states from Gazebo'''
			uav_ros.rk44(action=c_in.control, uav_state=uav_odom_2_uav_state(uav_odom))

			'''7. compute the virtual control output of the outer-loop subsystem'''
			if OBSERVER_OUT:
				syst_dynamic = -uav_ros.kt / uav_ros.m * uav_ros.dot_eta() + uav_ros.A()
				delta_outer_obs, dot_delta_outer_obs = obs_out.observe(dt=uav_ros.dt, x=uav_ros.eta(), syst_dynamic=syst_dynamic)
			else:
				delta_outer_obs = np.array([0., 0.])
			dot_eta = uav_ros.dot_eta()
			e_o = uav_ros.eta() - eta_d
			dot_e_o = dot_eta - dot_eta_d
			dot2_e_o = uav_ros.dot2_eta(delta_outer_obs) - dot2_eta_d
			c_out.control(e=e_o, de=dot_e_o, dd_ref=dot2_eta_d, d_ref=dot_eta_d, obs=delta_outer_obs)

			k_compensate_xy = np.array([0., 0.])
			c_out.ctrl += k_compensate_xy * dot2_e_o[0: 2]

			# k_compensate_vxy = np.array([0.6, 0.6])
			# c_out.ctrl += k_compensate_vxy * dot_e_o[0: 2]

			'''8. display'''
			# print('==========START==========')
			# print('time: %.3f' % uav_ros.time)
			# print(' x_REF    x    || Theta_ref')
			# print(' %.2f   %.2f  ||  %.2f' % (ref[0], uav_ros.x, theta_d * 180 / np.pi))
			# print(' y_REF    y    ||  Phi_ref')
			# print(' %.2f   %.2f  ||  %.2f' % (ref[1], uav_ros.y, phi_d * 180 / np.pi))
			# print(' z_REF    z    || thrust   PWM')
			# print(' %.2f   %.2f   ||  %.2f   %.2f' % (ref[2], uav_ros.z, c_in.control, thrust_2_throttle(c_in.control)))
			# print('===========END===========\n')

			'''10. data storage'''
			data_block = {'time': np.round(uav_ros.time, 3),  # time
						  'ctrl_in': np.array([c_in.control, ctrl_cmd.thrust]),  # thrust
						  'ref_angle': np.array([phi_d, theta_d, ref[3]]),  # phid thetad psid
						  'ref_pos': ref[0: 3],  # xd yd zd
						  'd_in_obs': delta_inner_obs,  # dz
						  'd_out_obs': delta_outer_obs,  # dx dy
						  'state': uav_ros.uav_state_call_back()}  # 12 状态
			data_record.record(data=data_block)

			if data_record.index == data_record.N:
				print('Data collection finish. Switching offboard position...')
				data_record.package2file(path=os.getcwd() + '/src/acc_2024_ros/scripts/datasave/')
				global_flag = 3
		elif global_flag == 3:		# finish, back to offboard position
			pose.pose.position.x = 0
			pose.pose.position.y = 0
			pose.pose.position.z = 0.5
			local_pos_pub.publish(pose)
		else:
			pose.pose.position.x = 0
			pose.pose.position.y = 0
			pose.pose.position.z = 0.5
			local_pos_pub.publish(pose)
			print('WORKING MODE ERROR...')
		rate.sleep()
