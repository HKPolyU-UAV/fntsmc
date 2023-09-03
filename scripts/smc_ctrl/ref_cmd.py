import numpy as np


def ref_inner(time, amplitude: np.ndarray, period: np.ndarray, bias_a: np.ndarray, bias_phase: np.ndarray):
    w = 2 * np.pi / period
    _r = amplitude * np.sin(w * time + bias_phase) + bias_a
    _dr = amplitude * w * np.cos(w * time + bias_phase)
    _ddr = -amplitude * w ** 2 * np.sin(w * time + bias_phase)
    _dddr = -amplitude * w ** 3 * np.cos(w * time + bias_phase)
    return _r, _dr, _ddr, _dddr


def ref_uav(time: float, amplitude: np.ndarray, period: np.ndarray, bias_a: np.ndarray, bias_phase: np.ndarray):
    w = 2 * np.pi / period
    _r = amplitude * np.sin(w * time + bias_phase) + bias_a
    _dr = amplitude * w * np.cos(w * time + bias_phase)
    _ddr = -amplitude * w ** 2 * np.sin(w * time + bias_phase)
    _dddr = -amplitude * w ** 3 * np.cos(w * time + bias_phase)
    return _r, _dr, _ddr, _dddr


def generate_uncertainty(time: float, is_ideal: bool = False) -> np.ndarray:
    # Fdx, Fdy, Fdz, dp, dq, dr
    if is_ideal:
        return np.array([0, 0, 0, 0, 0, 0]).astype(float)
    else:
        T = 5
        w = 2 * np.pi / T
        Fdx = 0.5 * np.sin(2 * w * time) + 0.2 * np.cos(w * time)
        Fdy = 0.5 * np.cos(2 * w * time) + 0.2 * np.sin(w * time)
        Fdz = 0.5 * np.sin(2 * w * time) + 0.2 * np.cos(w * time)
        dp = 0.5 * np.sin(w * time) + 0.2 * np.cos(2 * w * time)
        dq = 0.5 * np.cos(w * time) + 0.2 * np.sin(2 * w * time)
        dr = 0.5 * np.cos(w * time) + 0.2 * np.sin(2 * w * time)
        return np.array([Fdx, Fdy, Fdz, dp, dq, dr])
