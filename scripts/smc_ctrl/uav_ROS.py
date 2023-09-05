import numpy as np
import os, sys

sys.path.append(os.getcwd() + '/src/acc_2024_ros/scripts/')
sys.path.append(os.getcwd() + '/src/acc_2024_ros/scripts/smc_ctrl/')

from utils import *


class UAV_ROS:
	def __init__(self, m: float = 1.5, g: float = 9.8, kt: float = 1e-3, dt:float = 0.01):
		self.m = m  # 无人机质量
		self.g = g  # 重力加速度
		self.kt = kt  # 平移阻尼系数

		self.x = 0.
		self.y = 0.
		self.z = 0.

		self.vx = 0.
		self.vy = 0.
		self.vz = 0.

		self.phi = 0.
		self.theta = 0.
		self.psi = 0.

		self.p = 0.
		self.q = 0.
		self.r = 0.

		self.dt = dt
		self.n = 0  # 记录走过的拍数
		self.time = 0.  # 当前时间
		self.time_max = 30  # 每回合最大时间

		'''control'''
		self.control = self.m * self.g  # 油门
		'''control'''

		'''other'''
		self.f1g_old = self.f1_rho1() * self.h_rho1()
		self.f1g_new = self.f1_rho1() * self.h_rho1()
		self.Frho2_f1f2_old = self.dot_f1_rho1() * self.rho2() + self.f1_rho1() * self.f2_rho2()
		self.Frho2_f1f2_new = self.dot_f1_rho1() * self.rho2() + self.f1_rho1() * self.f2_rho2()
		'''other'''

	def rk44(self, action: float, uav_state: np.ndarray):
		self.control = action

		'''时间更新之前，先把需要求导的保存'''
		self.f1g_old = self.f1g_new
		self.Frho2_f1f2_old = self.Frho2_f1f2_new
		'''时间更新之前，先把需要求导的保存'''

		self.x = uav_state[0]
		self.y = uav_state[1]
		self.z = uav_state[2]

		self.vx = uav_state[3]
		self.vy = uav_state[4]
		self.vz = uav_state[5]

		self.phi = uav_state[6]
		self.theta = uav_state[7]
		self.psi = uav_state[8]

		self.p = uav_state[9]
		self.q = uav_state[10]
		self.r = uav_state[11]

		self.n += 1  # 拍数 +1
		self.time += self.dt

		'''需要求导的变量更新'''
		self.f1g_new = self.f1_rho1() * self.h_rho1()
		self.Frho2_f1f2_new = self.dot_f1_rho1() * self.rho2() + self.f1_rho1() * self.f2_rho2()
		'''需要求导的变量更新'''

	def uav_state_call_back(self):
		return np.array([self.x, self.y, self.z, self.vx, self.vy, self.vz, self.phi, self.theta, self.psi, self.p, self.q, self.r])

	'''内环控制器相关的'''
	def f1_rho1(self):
		"""
		:brief:  [1  0          0                     0          ]
				 [0  1  sin(phi)tan(theta)            0          ]
				 [0  0       cos(phi)               -sin(phi)    ]
				 [0  0  sin(phi)/cos(theta)  -cos(phi)/cos(theta)]
		:return: f1(rho_1)
		"""
		return 1.0

	def f2_rho2(self):
		"""
		:brief:  [       -(mg + kt * dz) / m       ]
				 [(kr * p + qr * (Iyy - Izz)) / Ixx]
				 [(kr * q + pr * (Izz - Ixx)) / Iyy]
				 [(kr * r + pq * (Ixx - Iyy)) / Izz]
		:return: f2(rho_2)
		"""
		_f2_rho2 = -(self.m * self.g + self.kt * self.vz) / self.m
		return _f2_rho2

	def h_rho1(self):
		"""
		:brief:  [cos(phi)cos(theta)/m  0      0       0 ]
				 [         0          1/Jxx    0       0 ]
				 [         0            0    1/Jyy     0 ]
				 [         0            0      0    1/Jzz]
		:return: h(rho_1)
		"""
		_g = np.cos(self.phi) * np.cos(self.theta) / self.m
		return _g

	def rho1(self):
		return self.z

	def rho2(self):
		return self.vz

	def dot_rho1(self):
		return self.f1_rho1() * self.rho2()

	def dot_f1_rho1(self):
		return 0.

	def dot_f1g(self):
		return (self.f1g_new - self.f1g_old) / self.dt

	def dot_Frho2_f1f2(self):
		return (self.Frho2_f1f2_new - self.Frho2_f1f2_old) / self.dt
	'''内环控制器相关的'''

	'''外环控制器相关的'''
	def eta(self):
		return np.array([self.x, self.y])

	def dot_eta(self):
		return np.array([self.vx, self.vy])

	def dot2_eta(self, obs: np.ndarray):
		return -self.kt / self.m * self.dot_eta() + self.A() + obs

	def A(self):
		return self.control / self.m * np.array([C(self.phi) * C(self.psi) * S(self.theta) + S(self.phi) * S(self.psi),
												 C(self.phi) * S(self.psi) * S(self.theta) - S(self.phi) * C(self.psi)]).astype(float)

	def eta_0(self):
		return self.control / self.m * np.array([C(self.psi) * C(self.phi) * S(self.theta) + S(self.phi) * S(self.psi),
												 S(self.psi) * C(self.phi) * S(self.theta) - S(self.phi) * C(self.psi)]).astype(float)
	'''外环控制器相关的'''
