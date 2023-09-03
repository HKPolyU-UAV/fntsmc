import numpy as np

from NESO import neso_in, neso_out
from controller import ctrl_in, ctrl_out, data_collector
from utils import *
from uav import UAV
from ref_cmd import *

OBSERVER = True

if __name__ == '__main__':
	'''inner controller initialization'''
	uav = UAV()

	'''controller initialization'''
	ctrl_out = ctrl_out()
	ctrl_in = ctrl_in(ctrl0=np.array([uav.m * uav.g, 0, 0, 0]).astype(float), dt=uav.dt)

	'''inner observer initialization'''
	obs_in = neso_in(l1=np.array([3., 3., 3., 3.]),
					 l2=np.array([3., 3., 3., 3.]),
					 l3=np.array([1., 1., 1., 1.]),
					 r=np.array([50., 2., 2., 100.]),  # 50
					 k1=np.array([0.7, 0.7, 0.7, 0.7]),
					 k2=np.array([0.001, 0.001, 0.001, 0.001]))  # z phi theta psi

	'''outer observer initialization'''
	obs_out = neso_out(l1=np.array([3., 3.]),
					   l2=np.array([3., 3.]),
					   l3=np.array([1., 1.]),
					   r=np.array([50., 50.]),  # 50
					   k1=np.array([0.7, 0.7]),
					   k2=np.array([0.001, 0.001]))  # x y

	'''data storage initialization'''
	data_record = data_collector(N=int(uav.time_max / uav.dt) + 1)

	'''reference signal initialization'''
	phi_d = theta_d = phi_d_old = theta_d_old = phi_d_old2 = theta_d_old2 = 0.
	dot_phi_d = (phi_d - phi_d_old) / uav.dt
	dot_theta_d = (theta_d - theta_d_old) / uav.dt
	dot2_phi_d = (phi_d - 2 * phi_d_old + phi_d_old2) / uav.dt ** 2
	dot2_theta_d = (theta_d - 2 * theta_d_old + theta_d_old2) / uav.dt ** 2

	ref_amplitude = np.array([2, 2, 1, np.pi / 2])
	ref_period = np.array([5, 5, 4, 5])
	ref_bias_a = np.array([2, 2, 1, 0])
	ref_bias_phase = np.array([np.pi / 2, 0, 0, 0])

	'''observer initialization'''
	ref, dot_ref, _, _ = ref_uav(uav.time, ref_amplitude, ref_period, ref_bias_a, ref_bias_phase)  # 整体参考信号 xd yd zd psid
	rhod = np.array([ref[2], phi_d, theta_d, ref[3]]).astype(float)  # 内环参考信号 z_d phi_d theta_d psi_d
	dot_rhod = np.array([dot_ref[2], dot_phi_d, dot_theta_d, dot_ref[3]]).astype(float)  # 内环参考信号导数
	uncertainty = generate_uncertainty(uav.time, False)

	e_I = uav.rho1() - rhod
	de_I = uav.dot_rho1() - dot_rhod
	syst_dynamic0 = np.dot(uav.dot_f1_rho1(), uav.rho2()) + \
					np.dot(uav.f1_rho1(), uav.f2_rho2()) + \
					np.dot(uav.f1_rho1(), np.dot(uav.g_rho1(), ctrl_in.control))
	obs_in.set_init(x0=e_I, dx0=de_I, syst_dynamic=syst_dynamic0)
	obs_out.set_init(x0=uav.eta(), dx0=uav.dot_eta(), syst_dynamic=ctrl_out.ctrl)

	data_block = {'time': uav.time,
				  'ctrl_in': ctrl_in.control,
				  'ref_angle': rhod[1:4],
				  'ref_pos': ref[0: 3],
				  'd_in': np.dot(uav.f1_rho1(), np.array([uncertainty[2] / uav.m, uncertainty[3], uncertainty[4], uncertainty[5]])),  # -[0,0,0,0]
				  'd_in_obs': np.array([0., 0., 0., 0.]).astype(float),
				  'd_out': np.array([uncertainty[0], uncertainty[1]]) / uav.m,
				  'd_out_obs': np.array([0., 0.]).astype(float),
				  'state': uav.uav_state_call_back()}
	data_record.record(data=data_block)

	'''嗨嗨嗨，开始控制了'''
	while uav.time < uav.time_max:
		if uav.n % 1000 == 0:
			print('time: ', uav.n * uav.dt)

		'''1. generate reference command and uncertainty'''
		ref, dot_ref, dot2_ref, dot3_ref = ref_uav(uav.time, ref_amplitude, ref_period, ref_bias_a, ref_bias_phase)  # 整体参考信号 xd yd zd psid
		uncertainty = generate_uncertainty(uav.time, False)

		'''2. generate outer-loop reference signal 'eta_d' and its 1st, 2nd, and 3rd-order derivatives'''
		eta_d = np.array([ref[0], ref[1]])  # 外环参考
		dot_eta_d = np.array([dot_ref[0], dot_ref[1]])  # 外环参考一阶导
		dot2_eta_d = np.array([dot2_ref[0], dot2_ref[1]])  # 外环参考二阶导
		dot3_eta_d = np.array([dot3_ref[0], dot3_ref[1]])  # 外环参考三阶导

		'''3. generate inner-loop reference signal 'rho_d' and its 1st and 2nd-order derivatives'''
		phi_d_old2 = phi_d_old
		theta_d_old2 = theta_d_old
		phi_d_old = phi_d
		theta_d_old = theta_d

		phi_d, theta_d = uo_2_ref_angle(ctrl_out.ctrl, ref[3], uav.m, ctrl_in.control[0])

		dot_phi_d = (phi_d - phi_d_old) / uav.dt
		dot_theta_d = (theta_d - theta_d_old) / uav.dt
		dot2_phi_d = (phi_d - 2 * phi_d_old + phi_d_old2) / uav.dt ** 2  # 作为参考，不计入计算，写在 excel 里面的
		dot2_theta_d = (theta_d - 2 * theta_d_old + theta_d_old2) / uav.dt ** 2  # 作为参考，不计入计算，写在 excel 里面的

		rhod = np.array([ref[2], phi_d, theta_d, ref[3]]).astype(float)  # z_d phi_d theta_d psi_d
		dot_rhod = np.array([dot_ref[2], dot_phi_d, dot_theta_d, dot_ref[3]]).astype(float)  # z_d phi_d theta_d psi_d 的一阶导数
		dot2_rhod = np.array([dot2_ref[2], dot2_phi_d, dot2_theta_d, dot2_ref[3]]).astype(float)  # z_d phi_d theta_d psi_d 的二阶导数

		'''4. compute the error 'e_I' and its 1st and 2nd-order derivatives'''
		de_I_old = de_I.copy()  # 这个时候 de_I 是上一时刻的
		e_I = uav.rho1() - rhod
		de_I = uav.dot_rho1() - dot_rhod  # 这个时候 de_I 是新时刻的
		if OBSERVER:  # observer for inner-loop subsystem, no matter AFTO or NESO 没啥区别我跟你说
			syst_dynamic = np.dot(uav.dot_f1_rho1(), uav.rho2()) + \
						   np.dot(uav.f1_rho1(), uav.f2_rho2()) + \
						   np.dot(uav.f1_rho1(), np.dot(uav.g_rho1(), ctrl_in.control))
			delta_inner_obs, _ = obs_in.observe(dt=uav.dt, x=e_I, syst_dynamic=syst_dynamic)
		else:
			# delta_inner_obs = np.array([0., 0., 0., 0.])
			delta_inner_obs = -dot2_rhod
		dde_I = np.dot(uav.dot_f1_rho1(), uav.rho2()) + \
				np.dot(uav.f1_rho1(), uav.f2_rho2() + np.dot(uav.g_rho1(), ctrl_in.control)) + \
				delta_inner_obs

		'''5. compute the derivative of the inner-loop controller'''
		ctrl_in.dot_control(e_I, de_I, dde_I,
							uav.f1_rho1(), uav.f2_rho2(), uav.rho2(), uav.g_rho1(), uav.dot_f1_rho1(), uav.dot_Frho2_f1f2(), uav.dot_f1g(),
							delta_inner_obs)

		'''6. ode solve'''
		uav.rk44(action=ctrl_in.control, dis=uncertainty, n=1)

		'''7. compute control output of the inner-loop subsystem for next time step'''
		ctrl_in.control_update()
		# c = np.array([1, 1, 1, 1])
		# k = np.array([1, 1, 1, 1])
		# _s = c * e_I + de_I
		# u_i1 = -c * de_I - np.dot(uav.dot_f1_rho1(), uav.rho2()) - np.dot(uav.f1_rho1(), uav.f2_rho2())
		# u_i2 = -k * _s # - np.fabs(delta_inner_obs) * np.tanh(10 * _s)
		# ctrl_in.control = np.dot(np.linalg.inv(np.dot(uav.f1_rho1(), uav.g_rho1())), u_i1 + u_i2)

		'''8. compute the virtual control output of the outer-loop subsystem'''
		if OBSERVER:
			syst_dynamic = -uav.kt / uav.m * uav.dot_eta() + uav.A()
			delta_outer_obs, dot_delta_outer_obs = obs_out.observe(dt=uav.dt, x=uav.eta(), syst_dynamic=syst_dynamic)
		else:
			delta_outer_obs = np.array([0., 0.])
		dot_eta = uav.dot_eta()
		e_o = uav.eta() - eta_d
		dot_e_o = dot_eta - dot_eta_d
		ctrl_out.control(e=e_o, de=dot_e_o, dd_ref=dot2_eta_d, obs=delta_outer_obs)

		'''10. data storage'''
		data_block = {'time': uav.time,
					  'ctrl_in': ctrl_in.control,
					  'ref_angle': rhod[1:4],
					  'ref_pos': ref[0: 3],
					  'd_in': np.dot(uav.f1_rho1(), np.array([uncertainty[2] / uav.m, uncertainty[3], uncertainty[4], uncertainty[5]])) - dot2_rhod,
					  'd_in_obs': delta_inner_obs,
					  'd_out': np.array([uncertainty[0], uncertainty[1]]) / uav.m,
					  'd_out_obs': delta_outer_obs,
					  'state': uav.uav_state_call_back()}
		data_record.record(data=data_block)

	'''data to file'''
	data_record.package2file(path=data_record.path)
	data_record.package2file(path=data_record.path2Matlab)
