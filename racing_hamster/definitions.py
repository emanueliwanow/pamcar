import os

import numpy as np
from dataclasses import dataclass
from dataclasses import field
import pickle
import path_handling
from scipy.integrate import solve_ivp
from typing import List, Tuple


@dataclass
class Target:
    x1: float
    y1: float
    x2: float
    y2: float


@dataclass
class RacingEnvironmentParameter:
    targets: List[Target] = field(default_factory=list)
    x_min: float = -4
    x_max: float = 4
    y_min: float = -2.5
    y_max: float = 2.5


class RacingEnvironment:
    def __init__(self, name: str):
        self._name = name
        self._save_path = path_handling.ENVIRONMENT_DIR / f'{self._name}.pickle'
        self._parameter: RacingEnvironmentParameter = RacingEnvironmentParameter()

    def get_name(self) -> str:
        return self._name

    def set_name(self, name: str) -> None:
        self._name = name
        self._save_path = path_handling.ENVIRONMENT_DIR / f'{self._name}.pickle'

    def get_x_limits(self) -> Tuple[float, float]:
        return self._parameter.x_min, self._parameter.x_max

    def get_y_limits(self) -> Tuple[float, float]:
        return self._parameter.y_min, self._parameter.y_max

    def save_state_to_disk(self) -> None:
        with open(self._save_path, 'wb') as file:
            pickle.dump(self._parameter, file)

    def load_state_from_disk(self) -> None:
        with open(self._save_path, 'rb') as file:
            self._parameter = pickle.load(file)

    def add_target(self, target: Target) -> None:
        self._parameter.targets.append(target)

    def get_targets(self) -> List[Target]:
        return self._parameter.targets


class Car:
    def __init__(self, x0: np.ndarray = np.array((0, 0, 1, 0, 0))):
        assert x0.shape == (5,)
        self._x = x0  # px, py, cos_theta, sin_theta, v
        self._x0 = x0

    def get_car_position(self) -> Tuple[float, float]:
        return self._x[0], self._x[1]

    def set_car_position(self, x: float, y: float) -> None:
        self._x[0], self._x[1] = x, y

    def step(self, u: np.ndarray, Ts: float) -> None:
        assert u.shape == (2,)

        def f(_, _x, _u):
            px, py, cos_theta, sin_theta, v = tuple(_x)
            a, theta_dot = tuple(_u)
            return np.array([cos_theta * v, sin_theta * v, -sin_theta * theta_dot, cos_theta * theta_dot, a])

        sol = solve_ivp(f, [0, Ts], y0=self._x, args=(u,))
        # noinspection PyUnresolvedReferences
        self._x = sol.y[:, -1]

    def reset(self):
        self._x = self._x0

    def get_speed(self) -> float:
        return self._x[-1]


class InputTrajectory:
    def __init__(self, name: str, u: np.ndarray = np.empty((0, 2)), Ts: float = 0.1):
        self._name = name
        self._u = u
        self._Ts = Ts
        self._save_path = path_handling.INPUT_TRAJECTORY_DIR / self._name

    def get_name(self) -> str:
        return self._name

    def set_name(self, name: str):
        self._name = name

    def get_input(self) -> np.ndarray:
        return self._u

    def append(self, u: np.ndarray):
        assert u.shape == (2,)
        self._u = np.vstack((self._u, u))

    def get_sampling_time(self) -> float:
        return self._Ts

    def save_trajectory_to_disk(self) -> None:
        if not self._save_path.is_dir():
            os.mkdir(self._save_path)
        np.save(self._save_path / "inputs.npy", self._u)
        np.save(self._save_path / "Ts.npy", self._Ts)

    def load_trajectory_from_disk(self) -> None:
        self._u = np.load(self._save_path / "inputs.npy")
        self._Ts = np.load(self._save_path / "Ts.npy")
