# -*- coding: utf-8 -*-
"""躍度最小軌道生成モジュール

時間固定の躍度最小軌道を5次多項式で生成します。
始点と終点の位置・速度・加速度を0として、滑らかな軌道を生成します。
"""
import numpy as np


class MinimumJerkTrajectory:
    """時間固定の躍度最小軌道生成クラス

    5次多項式を用いて、指定された時間内で始点から終点まで
    滑らかに移動する軌道を生成します。

    境界条件:
    - 始点: 位置=0, 速度=0, 加速度=0
    - 終点: 位置=distance, 速度=0, 加速度=0

    時間を[0, 1]に正規化した5次多項式:
    x(τ) = distance * (10τ³ - 15τ⁴ + 6τ⁵)  where τ = t/T
    """

    def __init__(self, distance: float, duration: float, t_start: float = 0.0) -> None:
        """コンストラクタ

        Args:
            distance: 移動距離（または角度差） [m or rad]
            duration: 所要時間 [s]
            t_start: 開始時刻 [s] (オプション)
        """
        self.distance = distance
        self.duration = duration if duration > 0 else 1e-6  # ゼロ除算対策
        self.t_start = t_start
        self.t_end = t_start + duration

    def _normalize_time(self, t: float) -> float:
        """時刻を[0, 1]に正規化

        Args:
            t: 時刻 [s]

        Returns:
            正規化時刻 τ ∈ [0, 1]
        """
        if t <= self.t_start:
            return 0.0
        elif t >= self.t_end:
            return 1.0
        else:
            return (t - self.t_start) / self.duration

    def position(self, t: float) -> float:
        """時刻tにおける位置を計算

        Args:
            t: 時刻 [s]

        Returns:
            位置 [m or rad]
        """
        tau = self._normalize_time(t)
        # x(τ) = d * (10τ³ - 15τ⁴ + 6τ⁵)
        tau2 = tau * tau
        tau3 = tau2 * tau
        tau4 = tau3 * tau
        tau5 = tau4 * tau
        return self.distance * (10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5)

    def velocity(self, t: float) -> float:
        """時刻tにおける速度を計算

        Args:
            t: 時刻 [s]

        Returns:
            速度 [m/s or rad/s]
        """
        if t < self.t_start or t > self.t_end:
            return 0.0

        tau = self._normalize_time(t)
        # v(τ) = (d/T) * (30τ² - 60τ³ + 30τ⁴)
        tau2 = tau * tau
        tau3 = tau2 * tau
        tau4 = tau3 * tau
        return (self.distance / self.duration) * (30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4)

    def acceleration(self, t: float) -> float:
        """時刻tにおける加速度を計算

        Args:
            t: 時刻 [s]

        Returns:
            加速度 [m/s² or rad/s²]
        """
        if t < self.t_start or t > self.t_end:
            return 0.0

        tau = self._normalize_time(t)
        # a(τ) = (d/T²) * (60τ - 180τ² + 120τ³)
        tau2 = tau * tau
        tau3 = tau2 * tau
        duration2 = self.duration * self.duration
        return (self.distance / duration2) * (60.0 * tau - 180.0 * tau2 + 120.0 * tau3)

    def jerk(self, t: float) -> float:
        """時刻tにおける躍度を計算

        Args:
            t: 時刻 [s]

        Returns:
            躍度 [m/s³ or rad/s³]
        """
        if t < self.t_start or t > self.t_end:
            return 0.0

        tau = self._normalize_time(t)
        # j(τ) = (d/T³) * (60 - 360τ + 360τ²)
        tau2 = tau * tau
        duration3 = self.duration * self.duration * self.duration
        return (self.distance / duration3) * (60.0 - 360.0 * tau + 360.0 * tau2)

    def max_velocity(self) -> float:
        """最大速度を計算（解析的に）

        Returns:
            最大速度の絶対値 [m/s or rad/s]
        """
        # v'(τ) = 0 を解くと τ = 0.5
        # v(0.5) = (d/T) * (30*0.25 - 60*0.125 + 30*0.0625)
        #        = (d/T) * 1.875
        return abs(self.distance / self.duration) * 1.875

    def max_acceleration(self) -> float:
        """最大加速度を計算（解析的に）

        Returns:
            最大加速度の絶対値 [m/s² or rad/s²]
        """
        # a'(τ) = 0 を解くと τ = 0.5 ± sqrt(1/12)
        # τ ≈ 0.2113 で最大
        tau_max = 0.5 - np.sqrt(1.0/12.0)
        tau2 = tau_max * tau_max
        tau3 = tau2 * tau_max
        duration2 = self.duration * self.duration
        return abs(self.distance / duration2) * abs(60.0 * tau_max - 180.0 * tau2 + 120.0 * tau3)

    def max_jerk(self) -> float:
        """最大躍度を計算（解析的に）

        Returns:
            最大躍度の絶対値 [m/s³ or rad/s³]
        """
        # j(0) = (d/T³) * 60
        duration3 = self.duration * self.duration * self.duration
        return abs(self.distance / duration3) * 60.0
