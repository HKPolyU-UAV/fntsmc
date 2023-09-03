import numpy as np
import pandas as pd
from NESO import neso_in
from control.controller import ctrl_in, data_collector
from uav import UAV
from ref_cmd import *

IS_IDEAL = False

if __name__ == '__main__':
    uav = UAV()
    ctrl_in = ctrl_in(ctrl0=np.array([uav.m * uav.g, 0, 0, 0]).astype(float), dt=uav.dt)
    observer = neso_in(l1=np.array([2., 2., 2., 2.]),
                       l2=np.array([2., 2., 2., 2.]),
                       l3=np.array([1., 1., 1., 1.]),
                       r=np.array([20., 20., 20., 20.]),   # 50
                       k1=np.array([0.5, 0.5, 0.5, 0.5]),
                       k2=np.array([0.001, 0.001, 0.001, 0.001]))

    ref_amplitude = np.array([1, np.pi / 3, np.pi / 3, np.pi / 2])
    ref_period = np.array([5, 5, 5, 4])
    ref_bias_a = np.array([2, 0, 0, 0])
    ref_bias_phase = np.array([0, 0, np.pi / 2, 0])

    rhod, dot_rhod, dot2_rhod, dot3_rhod = ref_inner(uav.time, ref_amplitude, ref_period, ref_bias_a, ref_bias_phase)
    e0 = uav.rho1() - rhod
    de0 = uav.dot_rho1() - dot_rhod
    syst_dynamic0 = np.dot(uav.dot_f1_rho1(), uav.rho2()) + \
                    np.dot(uav.f1_rho1(), uav.f2_rho2()) + \
                    np.dot(uav.f1_rho1(), np.dot(uav.g_rho1(), ctrl_in.control))
    observer.set_init(x0=e0, dx0=de0, syst_dynamic=syst_dynamic0)
    '''一些中间变量初始化'''
    de = np.array([0, 0, 0, 0]).astype(float)
    '''一些中间变量初始化'''

    data_record = data_collector(N=int(uav.time_max / uav.dt) + 1)

    while uav.time < uav.time_max:
        if uav.n % 1000 == 0:
            print(uav.n)
        '''1. 计算 tk 时刻参考信号 和 生成不确定性'''
        uncertainty = generate_uncertainty(time=uav.time, is_ideal=False)
        rhod, dot_rhod, dot2_rhod, dot3_rhod = ref_inner(uav.time, ref_amplitude, ref_period, ref_bias_a, ref_bias_phase)
        '''1. 计算 tk 时刻参考信号 和 生成不确定性'''

        '''2. 计算 tk 时刻误差信号'''
        if uav.n == 0:
            e = uav.rho1() - rhod
            de = uav.dot_rho1() - dot_rhod  # 这个时候 de 是新时刻的
            de_old = de.copy()  # 这个时候 de 是上一时刻的
        else:
            de_old = de.copy()  # 这个时候 de 是上一时刻的
            e = uav.rho1() - rhod
            de = uav.dot_rho1() - dot_rhod  # 这个时候 de 是新时刻的

        syst_dynamic = np.dot(uav.dot_f1_rho1(), uav.rho2()) + \
                       np.dot(uav.f1_rho1(), uav.f2_rho2()) + \
                       np.dot(uav.f1_rho1(), np.dot(uav.g_rho1(), ctrl_in.control))
        # delta_obs, dot_delta_obs = observer.observe(dt=uav.dt, x=e, syst_dynamic=syst_dynamic)
        delta_obs = -dot2_rhod

        if IS_IDEAL:
            dde = np.dot(uav.dot_f1_rho1(), uav.rho2()) + \
                    np.dot(uav.f1_rho1(), uav.f2_rho2() + np.dot(uav.g_rho1(), ctrl_in.control))  # - dot2_rhod
        else:
            dde = np.dot(uav.dot_f1_rho1(), uav.rho2()) + \
                    np.dot(uav.f1_rho1(), uav.f2_rho2() + np.dot(uav.g_rho1(), ctrl_in.control)) + \
                    delta_obs
        '''2. 计算 tk 时刻误差信号'''

        ctrl_in.dot_control(e, de, dde,
                            uav.f1_rho1(), uav.f2_rho2(), uav.rho2(), uav.g_rho1(), uav.dot_f1_rho1(), uav.dot_Frho2_f1f2(), uav.dot_f1g(),
                            delta_obs)

        data_block = {'time': uav.time,
                      'ctrl_in': ctrl_in.control,
                      'ref_angle': rhod[1:4],
                      'ref_pos': np.array([0., 0., rhod[0]]),
                      'd_in': np.dot(uav.f1_rho1(), np.array([uncertainty[2] / uav.m, uncertainty[3], uncertainty[4], uncertainty[5]])) - dot2_rhod,
                      'd_in_obs': delta_obs,
                      'd_out': np.array([uncertainty[0], uncertainty[1]]) / uav.m,
                      'd_out_obs': np.array([0., 0.]),
                      'state': uav.uav_state_call_back()}
        data_record.record(data=data_block)

        uav.rk44(action=ctrl_in.control, dis=uncertainty, n=1)

        ctrl_in.control_update()

    data_record.package2file(path='../datasave/')
