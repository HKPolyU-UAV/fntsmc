import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os, sys


def plot_pos():
    plt.figure()
    plt.subplot(1, 3, 1)
    plt.plot(time, ref_pos[:, 0], 'red')
    plt.plot(time, uav_pos[:, 0], 'blue')
    plt.grid(True)
    plt.ylim((-5, 5))
    plt.yticks(np.arange(-5, 5, 1))
    plt.xlabel('time(s)')
    plt.title('X')

    plt.subplot(1, 3, 2)
    plt.plot(time, ref_pos[:, 1], 'red')
    plt.plot(time, uav_pos[:, 1], 'blue')
    plt.grid(True)
    plt.ylim((-5, 5))
    plt.yticks(np.arange(-5, 5, 1))
    plt.xlabel('time(s)')
    plt.title('Y')

    plt.subplot(1, 3, 3)
    plt.plot(time, ref_pos[:, 2], 'red')
    plt.plot(time, uav_pos[:, 2], 'blue')
    plt.grid(True)
    plt.ylim((-5, 5))
    plt.yticks(np.arange(-5, 5, 1))
    plt.xlabel('time(s)')
    plt.title('Z')


def plot_phi():
    plt.figure()
    plt.plot(time, ref_angle[:, 0], 'red')
    plt.plot(time, uav_angle[:, 0], 'blue')
    plt.grid(True)
    plt.ylim((-90, 90))
    plt.yticks(np.arange(-90, 90, 10))
    plt.xlabel('time(s)')
    plt.title('roll  $\phi$')


def plot_theta():
    plt.figure()
    plt.plot(time, ref_angle[:, 1], 'red')
    plt.plot(time, uav_angle[:, 1], 'blue')
    plt.grid(True)
    plt.ylim((-90, 90))
    plt.yticks(np.arange(-90, 90, 10))
    plt.xlabel('time(s)')
    plt.title('pitch  $\Theta$')


def plot_psi():
    plt.figure()
    plt.plot(time, ref_angle[:, 2], 'red')
    plt.plot(time, uav_angle[:, 2], 'blue')
    plt.grid(True)
    plt.ylim((-90, 90))
    plt.yticks(np.arange(-90, 90, 15))
    plt.xlabel('time(s)')
    plt.title('yaw  $\psi$')


def plot_throttle():
    plt.figure()
    plt.plot(time, in_control, 'red')  # 油门
    plt.grid(True)
    plt.title('throttle')

def plot_obs():
    plt.figure()
    plt.subplot(1, 3, 1)
    plt.plot(time, delta_obs[:, 0], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    plt.title('observe dx')
    # plt.ylim((-4, 4))

    plt.subplot(1, 3, 2)
    plt.plot(time, delta_obs[:, 1], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    # plt.ylim((-4, 4))
    plt.title('observe dy')

    plt.subplot(1, 3, 3)
    plt.plot(time, delta_obs[:, 2], 'red')
    plt.grid(True)
    plt.xlabel('time(s)')
    # plt.ylim((-4, 4))
    plt.title('observe dz')


if __name__ == '__main__':
    """
    Data formation:
        control.csv:    t throttle Tx Ty Tz
        observe.csv:    t in_01 in_02 in_03 in_04 in_01_obs in_02_obs in_03_obs in_04_obs
        ref_cmd.csv:    t rx ry rz r_phi(roll) r_theta(pitch) r_psi(yaw)
        uav_state.csv:  t x y z vx vy vz phi theta psi p q r
    """
    # path = './datasave/'
    path = os.getcwd() + '/src/control/scripts/datasave/'
    # path = ''
    controlData = pd.read_csv(path + 'control.csv', header=0).to_numpy()
    observeData = pd.read_csv(path + 'observe.csv', header=0).to_numpy()
    ref_cmdData = pd.read_csv(path + 'ref_cmd.csv', header=0).to_numpy()
    uav_stateData = pd.read_csv(path +'uav_state.csv', header=0).to_numpy()

    L = controlData.shape[0]
    time = controlData[0: L - 2, 0]

    in_control = controlData[0: L - 2, 1]

    delta_obs = observeData[0: L - 2, 1:4]

    ref_pos = ref_cmdData[0: L - 2, 1: 4]
    ref_angle = ref_cmdData[0: L - 2, 4: 7] * 180 / np.pi

    uav_pos = uav_stateData[0: L - 2, 1: 4]
    uav_angle = uav_stateData[0: L - 2, 7: 10] * 180 / np.pi
    plot_pos()
    plot_phi()
    plot_theta()
    plot_psi()
    plot_throttle()
    plot_obs()

    plt.show()
