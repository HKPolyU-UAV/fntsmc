import numpy as np
import pandas as pd
import platform


class ctrl_in:
	def __init__(self, thrust: float, dt: float, c: float = 3., lmd: float = 8., k0: float = 15.):
		self.c = c  # 3.    z channel, the bigger c is, the larger control error is.
		self.lmd = lmd  # 3.   too small -> large error, too big: diverge
		self.k0 = k0
		self.s = 0.
		self.ds = 0.
		self.sigma = 0.
		self.control = thrust
		self.du = 0.
		self.dt = dt

	def dot_control(self,
					e: float, de: float, dde: float,
					f1: float, f2: float, rho2: float, h: float, F: float, dot_Frho2_f1f2: float, dot_f1h: float,
					delta_obs: float):
		syst_dynamic1 = np.dot(F, rho2) + np.dot(f1, f2)
		syst_dynamic2 = dot_Frho2_f1f2

		self.s = self.c * e + de
		self.ds = self.c * de + dde
		self.sigma = self.ds + self.lmd * self.s

		du1 = self.lmd * self.c * de + (self.c + self.lmd) * syst_dynamic1  # + syst_dynamic2
		du2 = ((self.c + self.lmd) * f1 * h + dot_f1h) * self.control
		V_dis = (self.c + self.lmd) * delta_obs  # + dot_delta_obs
		du3 = (np.fabs(V_dis) + self.k0) * np.tanh(5 * self.sigma)
		self.du = -(du1 + du2 + du3) / (f1 * h)
		# self.du = -np.dot(np.linalg.inv(np.dot(f1, h)), du1 + du2 + du3)

	def control_update(self):
		self.control += self.du * self.dt


class ctrl_in2:
	def __init__(self,
				 ctrl0: float = 0.5,
				 k1: float = 0.4,
				 k2: float = 0.4,
				 alpha: float = 1.2,
				 beta: float = 0.7,
				 gamma: float = 0.2,
				 lmd: float = 2.0,
				 dt: float = 0.01):
		self.k1 = k1
		self.k2 = k2
		self.alpha = alpha
		self.beta = beta
		self.gamma = gamma
		self.lmd = lmd
		self.dt = dt
		self.ki = 0.
		self.e_integration = 0.

		self.cnt = 0

		self.s = 0.
		self.dot_s1 = 0.
		self.s1 = 0.
		self.sigma = self.s + self.lmd * self.s1
		self.control = ctrl0

	def control_update(self,
					   m: float,
					   g: float,
					   phi: float,
					   theta: float,
					   kp: float,
					   dz: float,
					   dot2_zd: float,
					   dot_zd: float,
					   e: float,
					   de: float,
					   delta_obs: float):
		k_tanh_e = 5
		k_tanh_sigma0 = 5

		'''杨烨峰好使的'''
		# self.s = de + self.k1 * e + self.gamma * np.fabs(e) ** self.alpha * np.tanh(k_tanh_e * e)
		# self.dot_s1 = np.fabs(self.s) ** self.beta * np.tanh(k_tanh_sigma0 * self.s)
		# self.s1 += self.dot_s1 * self.dt
		# self.sigma = self.s + self.lmd * self.s1

		# u1 = (m * g + kp * dz) / m + dot2_zd - self.k1 * de - self.gamma * self.alpha * np.fabs(e) ** (self.alpha - 1) * de - self.lmd * self.dot_s1
		# u2 = -delta_obs - self.k2 * self.sigma
		# self.control = m * (u1 + u2 - self.ki * self.e_integration) / (np.cos(phi) * np.cos(theta))
		'''杨烨峰好使的'''

		kk_ht = 0.2
		self.s = de + self.k1 * e + self.gamma * np.fabs(e) ** self.alpha * np.tanh(k_tanh_e * e)
		self.dot_s1 = np.fabs(self.s) ** self.beta * np.tanh(k_tanh_sigma0 * self.s)
		self.s1 += self.dot_s1 * self.dt
		self.sigma = self.s + self.lmd * self.s1
		# print('杨烨峰', self.sigma)

		uf0 = (m * g + kp * dz) / m + dot2_zd
		uf11 = -self.k1 * (de - kk_ht * dot_zd) - self.gamma * self.alpha * np.fabs(e) ** (self.alpha - 1) * (de - kk_ht * dot_zd) - self.lmd * self.dot_s1
		if np.fabs(self.sigma) > 1:
			uf12 = -delta_obs - self.k2 * self.sigma
		else:
			uf12 = -delta_obs - 2 * self.k2 * np.sign(self.sigma)
		# uf12 = -delta_obs - self.k2 * self.sigma
		self.control = (uf0 + uf11 + uf12) * m / (np.cos(phi) * np.cos(theta))


class ctrl_in3:
	def __init__(self,
				 ctrl0: float = 0.5,
				 k: float = 0.4,
				 k0: float = 15.,
				 beta: float = 0.7,
				 c: float = 5,
				 p: float = 0.,
				 q: float = 0.,
				 m: float = 5,
				 n: float = 3,
				 dt: float = 0.01):
		self.p = p
		self.q = q
		self.beta = beta
		self.beta0 = beta
		self.c0 = c
		self.c = c
		self.m = m
		self.n = n
		self.k = k
		self.k0 = k0
		self.dt = dt

		self.cnt = 0
		self.ki = 0.06
		self.e_integration = 0.

		self.s = 0.
		self.control = ctrl0

	def control_update(self,
					   m: float,
					   g: float,
					   phi: float,
					   theta: float,
					   kp: float,
					   dz: float,
					   dot2_zd: float,
					   e: float,
					   de: float,
					   delta_obs: float):
		if np.linalg.norm([e]) < 0.3:
			self.c = 1
		else:
			self.c = 1 + (np.linalg.norm([e]) - 0.3) ** 2

		# if np.linalg.norm([de]) < 0.5:
		# 	self.beta = 1e-2
		# else:
		# 	self.beta = 0.5 * np.linalg.norm([de])

		self.s = self.c * e + np.sign(de) * np.fabs(de) ** (self.p / self.q) * self.beta
		u1 = (m * g + kp * dz) / m + dot2_zd
		u2 = -self.c / self.beta * self.q / self.p * np.fabs(de) ** (2 - self.p / self.q)
		# u3 = -(np.fabs(delta_obs) + self.k0) * np.tanh(5 * self.s) - self.k * self.s
		u3 = -self.k * np.sign(self.s) * np.fabs(self.s) ** (self.m / self.n) - delta_obs
		# print(u1, u2, u3)
		self.e_integration += e
		self.control = m * (u1 + u2 + u3 - self.e_integration * self.ki) / (np.cos(phi) * np.cos(theta))


class ctrl_out:
	def __init__(self,
				 k1: np.ndarray = np.array([0.4, 0.4]),  # 1.2, 0.8
				 k2: np.ndarray = np.array([0.4, 0.6]),  # 0.2, 0.6
				 k3: np.ndarray = np.array([0.05, 0.05]),
				 alpha: np.ndarray = np.array([1.2, 1.2]),
				 beta: np.ndarray = np.array([0.7, 0.7]),  # 0.3, 0.3 從0.3改成0.7之後，效果明顯提升
				 gamma: np.ndarray = np.array([0.2, 0.2]),
				 lmd: np.ndarray = np.array([2.0, 2.0]),
				 dt: float = 0.01):
		self.k1 = k1  # 第一维度控制 y ，第二维度控制 x
		self.k2 = k2
		# self.k2 = np.array([1.0, 1.0])
		self.k3 = k3
		self.alpha = alpha
		self.beta = beta
		self.gamma = gamma
		self.lmd = lmd
		self.dt = dt
		self.sigma_o = np.array([0., 0.]).astype(float)
		self.dot_sigma_o1 = np.array([0., 0.]).astype(float)
		self.sigma_o1 = np.array([0., 0.]).astype(float)
		self.so = self.sigma_o + lmd * self.sigma_o1
		self.ctrl = np.array([0., 0.]).astype(float)

	def control(self,
				e: np.ndarray,
				de: np.ndarray,
				dd_ref: np.ndarray,
				d_ref: np.ndarray,
				obs: np.ndarray):
		k_tanh_e = 5
		k_tanh_sigma0 = 5
		kk_ht = 0.25		# 1.2 米，补偿0.3
		self.sigma_o = (de - kk_ht * d_ref) + self.k1 * e + self.gamma * np.fabs(e) ** self.alpha * np.tanh(k_tanh_e * e)
		self.dot_sigma_o1 = np.fabs(self.sigma_o) ** self.beta * np.tanh(k_tanh_sigma0 * self.sigma_o)
		self.sigma_o1 += self.dot_sigma_o1 * self.dt
		self.so = self.sigma_o + self.lmd * self.sigma_o1
		uo1 = dd_ref - self.k1 * (de - kk_ht * d_ref) \
		      - self.gamma * self.alpha * np.fabs(e) ** (self.alpha - 1) * (de - kk_ht * d_ref) \
			  - self.lmd * self.dot_sigma_o1
		uo2 = -self.k2 * self.so - obs
		self.ctrl = uo1 + uo2


class data_collector:
	def __init__(self, N: int = 2):
		self.t = np.zeros((N, 1)).astype(float)
		self.inner_control = np.zeros((N, 2)).astype(float)
		self.ref_angle = np.zeros((N, 3)).astype(float)
		self.ref_pos = np.zeros((N, 3)).astype(float)
		self.d_in_obs = np.zeros((N, 1)).astype(float)
		self.d_out_obs = np.zeros((N, 2)).astype(float)
		self.state = np.zeros((N, 12)).astype(float)
		self.index = 0
		self.name = ['uav_state.csv', 'ref_cmd.csv', 'control.csv', 'observe.csv']
		# self.path = './datasave/'
		self.path = '../scripts/datasave/'
		self.path2Matlab = self.path + 'plot_matlab/'
		self.N = N

	def record(self, data: dict):
		if self.index < self.N:
			self.t[self.index][0] = data['time']
			self.inner_control[self.index] = data['ctrl_in']
			self.ref_angle[self.index] = data['ref_angle']
			self.ref_pos[self.index] = data['ref_pos']
			self.d_in_obs[self.index] = data['d_in_obs']
			self.d_out_obs[self.index] = data['d_out_obs']
			self.state[self.index] = data['state']
			self.index += 1

	def package2file(self, path: str):
		pd.DataFrame(np.hstack((self.t, self.state)),
					 columns=['time', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'phi', 'theta', 'psi', 'p', 'q', 'r']). \
			to_csv(path + self.name[0], sep=',', index=False)

		pd.DataFrame(np.hstack((self.t, self.ref_pos, self.ref_angle)),
					 columns=['time', 'ref_x', 'ref_y', 'ref_z', 'ref_phi', 'ref_theta', 'ref_psi']). \
			to_csv(path + self.name[1], sep=',', index=False)

		pd.DataFrame(np.hstack((self.t, self.inner_control)),
					 columns=['time', 'thrust', 'throttle']). \
			to_csv(path + self.name[2], sep=',', index=False)

		pd.DataFrame(np.hstack((self.t, self.d_out_obs, self.d_in_obs)),
					 columns=['time', 'dx_obs', 'dy_obs', 'dz_obs']). \
			to_csv(path + self.name[3], sep=',', index=False)
