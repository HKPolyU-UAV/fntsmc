import numpy as np


class neso_in:
    def __init__(self, l1: float, l2: float, l3: float, r: float, k1: float, k2: float):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.r = r
        self.k1 = k1
        self.k2 = k2

        self.z1 = 0.
        self.z2 = 0.
        self.z3 = 0.  # 这个是输出
        self.xi = 0.
        self.dot_z1 = 0.
        self.dot_z2 = 0.
        self.dot_z3 = 0.

    def fal(self, xi: float):
        if np.fabs(xi) <= self.k2:
            res = xi / (self.k2 ** (1 - self.k1))
            # xi[i] / (self.k2[i] ** (1 - self.k1[i]))
        else:
            res = np.fabs(xi) ** self.k1 * np.sign(xi)
            # np.fabs(xi[i]) ** self.k1[i] * np.sign(xi[i])
        return res

    def set_init(self, x0: float, dx0: float, syst_dynamic: float):
        self.z1 = x0  # 估计x
        self.z2 = dx0  # 估计dx
        self.z3 = 0. # 估计干扰
        self.xi = x0 - self.z1
        self.dot_z1 = self.z2 + self.l1 / self.r * self.fal(xi=self.r ** 2 * self.xi)
        self.dot_z2 = self.z3 + self.l2 * self.fal(xi=self.r ** 2 * self.xi) + syst_dynamic
        self.dot_z3 = self.r * self.l3 * self.fal(xi=self.r ** 2 * self.xi)

    def observe(self, dt: float, x: float, syst_dynamic: float):
        self.xi = x - self.z1
        self.dot_z1 = self.z2 + self.l1 / self.r * self.fal(xi=self.r ** 2 * self.xi)
        self.dot_z2 = self.z3 + self.l2 * self.fal(xi=self.r ** 2 * self.xi) + syst_dynamic
        self.dot_z3 = self.r * self.l3 * self.fal(xi=self.r ** 2 * self.xi)
        self.z1 += self.dot_z1 * dt
        self.z2 += self.dot_z2 * dt
        self.z3 += self.dot_z3 * dt
        delta_obs = self.z3
        dot_delta_obs = self.dot_z3
        return delta_obs, dot_delta_obs


class neso_out:
    def __init__(self, l1: np.ndarray, l2: np.ndarray, l3: np.ndarray, r: np.ndarray, k1: np.ndarray, k2: np.ndarray):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.r = r
        self.k1 = k1
        self.k2 = k2

        self.z1 = np.array([0., 0.]).astype(float)
        self.z2 = np.array([0., 0.]).astype(float)
        self.z3 = np.array([0., 0.]).astype(float)  # 这个是输出
        self.xi = np.array([0., 0.]).astype(float)
        self.dot_z1 = np.array([0., 0.]).astype(float)
        self.dot_z2 = np.array([0., 0.]).astype(float)
        self.dot_z3 = np.array([0., 0.]).astype(float)

    def fal(self, xi: np.ndarray):
        res = []
        for i in range(2):
            if np.fabs(xi[i]) <= self.k2[i]:
                res.append(xi[i] / (self.k2[i] ** (1 - self.k1[i])))
            else:
                res.append(np.fabs(xi[i]) ** self.k1[i] * np.sign(xi[i]))
        return np.array(res)

    def set_init(self, x0: np.ndarray, dx0: np.ndarray, syst_dynamic: np.ndarray):
        self.z1 = x0  # 估计x
        self.z2 = dx0  # 估计dx
        self.z3 = np.zeros(2)  # 估计干扰
        self.xi = x0 - self.z1
        self.dot_z1 = self.z2 + self.l1 / self.r * self.fal(xi=self.r ** 2 * self.xi)
        self.dot_z2 = self.z3 + self.l2 * self.fal(xi=self.r ** 2 * self.xi) + syst_dynamic
        self.dot_z3 = self.r * self.l3 * self.fal(xi=self.r ** 2 * self.xi)

    def observe(self, dt: float, x: np.ndarray, syst_dynamic: np.ndarray):
        self.xi = x - self.z1
        self.dot_z1 = self.z2 + self.l1 / self.r * self.fal(xi=self.r ** 2 * self.xi)
        self.dot_z2 = self.z3 + self.l2 * self.fal(xi=self.r ** 2 * self.xi) + syst_dynamic
        self.dot_z3 = self.r * self.l3 * self.fal(xi=self.r ** 2 * self.xi)
        self.z1 = self.z1 + self.dot_z1 * dt
        self.z2 = self.z2 + self.dot_z2 * dt
        self.z3 = self.z3 + self.dot_z3 * dt
        delta_obs = self.z3.copy()
        dot_delta_obs = self.dot_z3.copy()
        return delta_obs, dot_delta_obs
