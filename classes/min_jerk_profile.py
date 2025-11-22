# -*- coding: utf-8 -*-
"""躍度最小軌跡生成モジュール

このモジュールは、最小躍度（minimum jerk）原理に基づいた
滑らかな軌跡を生成するためのクラスを提供します。
主に角度や姿勢の滑らかな変化を生成するために使用されます。
"""


# 躍度最小軌跡の多項式係数
# x(t) = 6t^5 - 15t^4 + 10t^3 の係数
_POLY_COEFF_POSITION = (6.0, -15.0, 10.0)
# v(t) = 30t^4 - 60t^3 + 30t^2 の係数
_POLY_COEFF_VELOCITY = (30.0, -60.0, 30.0)
# a(t) = 120t^3 - 180t^2 + 60t の係数
_POLY_COEFF_ACCELERATION = (120.0, -180.0, 60.0)
# j(t) = 360t^2 - 360t + 60 の係数
_POLY_COEFF_JERK = (360.0, -360.0, 60.0)


class MinJerkProfile:
    """躍度最小軌跡生成クラス

    最小躍度（minimum jerk）原理に基づき、開始位置から目標位置まで
    滑らかに変化する軌跡を生成します。

    5次多項式を使用して、始点と終点で速度・加速度がゼロとなる
    滑らかな軌跡を生成します。

    Attributes:
        _position_range: 位置の変化量 (x_target - x_start)
        _position_offset: 位置のオフセット (x_start)
        _time_scale: 時間のスケーリング係数 (1 / time)
    """

    def __init__(self, x_start: float, x_target: float, time: float) -> None:
        """コンストラクタ

        Args:
            x_start: 開始位置 [m] または 開始角度 [rad]
            x_target: 目標位置 [m] または 目標角度 [rad]
            time: 移動時間 [s]
        """
        self._position_range: float = 0.0
        self._position_offset: float = 0.0
        self._time_scale: float = 1.0
        self.reset(x_start, x_target, time)

    def reset(self, x_start: float, x_target: float, time: float) -> None:
        """引数の拘束条件から躍度最小曲線を生成する

        Args:
            x_start: 開始位置 [m] または 開始角度 [rad]
            x_target: 目標位置 [m] または 目標角度 [rad]
            time: 移動時間 [s]
        """
        self._position_range = x_target - x_start
        self._position_offset = x_start
        self._time_scale = 1.0 / time

    def j(self, t: float) -> float:
        """時刻tにおける躍度を計算

        Args:
            t: 時刻 [s]

        Returns:
            躍度 [m/s³] または [rad/s³]
        """
        t_normalized = self._time_scale * t
        return self._position_range * (
            _POLY_COEFF_JERK[0] * t_normalized**2
            + _POLY_COEFF_JERK[1] * t_normalized
            + _POLY_COEFF_JERK[2]
        )

    def a(self, t: float) -> float:
        """時刻tにおける加速度を計算

        Args:
            t: 時刻 [s]

        Returns:
            加速度 [m/s²] または [rad/s²]
        """
        t_normalized = self._time_scale * t
        return self._position_range * (
            _POLY_COEFF_ACCELERATION[0] * t_normalized**3
            + _POLY_COEFF_ACCELERATION[1] * t_normalized**2
            + _POLY_COEFF_ACCELERATION[2] * t_normalized
        )

    def v(self, t: float) -> float:
        """時刻tにおける速度を計算

        Args:
            t: 時刻 [s]

        Returns:
            速度 [m/s] または角速度 [rad/s]
        """
        t_normalized = self._time_scale * t
        return self._position_range * (
            _POLY_COEFF_VELOCITY[0] * t_normalized**4
            + _POLY_COEFF_VELOCITY[1] * t_normalized**3
            + _POLY_COEFF_VELOCITY[2] * t_normalized**2
        )

    def x(self, t: float) -> float:
        """時刻tにおける位置を計算

        Args:
            t: 時刻 [s]

        Returns:
            位置 [m] または 角度 [rad]
        """
        t_normalized = self._time_scale * t
        return (
            self._position_range * (
                _POLY_COEFF_POSITION[0] * t_normalized**5
                + _POLY_COEFF_POSITION[1] * t_normalized**4
                + _POLY_COEFF_POSITION[2] * t_normalized**3
            )
            + self._position_offset
        )
