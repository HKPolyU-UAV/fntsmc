import numpy as np


def deg2rad(deg: float) -> float:
    """
    :brief:         omit
    :param deg:     degree
    :return:        radian
    """
    return deg * np.pi / 180.0


def rad2deg(rad: float) -> float:
    """
    :brief:         omit
    :param rad:     radian
    :return:        degree
    """
    return rad * 180.8 / np.pi


def C(x):
    return np.cos(x)


def S(x):
    return np.sin(x)


def uo_2_ref_angle(uo: np.ndarray, psi_d: float, m: float, uf: float, angle_max: float):
    ux = uo[0]
    uy = uo[1]
    asin_phi_d = min(max((ux * np.sin(psi_d) - uy * np.cos(psi_d)) * m / uf, -1), 1)
    phi_d = np.arcsin(asin_phi_d)
    asin_theta_d = min(max((ux * np.cos(psi_d) + uy * np.sin(psi_d)) * m / (uf * np.cos(phi_d)), -1), 1)
    theta_d = np.arcsin(asin_theta_d)
    phi_d = max(min(phi_d, angle_max), -angle_max)
    theta_d = max(min(theta_d, angle_max), -angle_max)
    return phi_d, theta_d
