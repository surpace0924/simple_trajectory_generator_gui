# -*- coding: utf-8 -*-
"""速度プロファイル生成モジュール

このモジュールは、躍度・加速度・速度の制約条件を満たしながら、
滑らかな加減速曲線を生成するためのクラスを提供します。
"""
from typing import Optional
import numpy as np


class AccelCurve:
    """加速曲線を生成するクラス

    引数の制約条件に従って加速曲線を生成します。
    - 始点速度と終点速度を滑らかにつなぐ
    - 移動距離の拘束はない
    - 始点速度および終点速度は、正でも負でも可

    Attributes:
        jm: 最大躍度（符号付き） [m/s³]
        am: 最大加速度（符号付き） [m/s²]
        t0, t1, t2, t3: 境界時刻 [s]
        v0, v1, v2, v3: 境界速度 [m/s]
        x0, x1, x2, x3: 境界位置 [m]
    """

    def __init__(
        self,
        j_max: float = 1.0,
        a_max: float = 1.0,
        v_start: float = 0.0,
        v_end: float = 0.0
    ) -> None:
        """コンストラクタ

        Args:
            j_max: 最大躍度の大きさ [m/s³], 正であること
            a_max: 最大加速度の大きさ [m/s²], 正であること
            v_start: 始点速度 [m/s]
            v_end: 終点速度 [m/s]
        """
        # 初期化
        self.jm: float = 1.0
        self.am: float = 1.0
        self.t0: float = 0.0
        self.t1: float = 0.0
        self.t2: float = 0.0
        self.t3: float = 0.0
        self.v0: float = 0.0
        self.v1: float = 0.0
        self.v2: float = 0.0
        self.v3: float = 0.0
        self.x0: float = 0.0
        self.x1: float = 0.0
        self.x2: float = 0.0
        self.x3: float = 0.0
        self.reset(j_max, a_max, v_start, v_end)

    def reset(
        self,
        j_max: float,
        a_max: float,
        v_start: float,
        v_end: float
    ) -> None:
        """引数の拘束条件から曲線を生成する

        Args:
            j_max: 最大躍度の大きさ [m/s³], 正であること
            a_max: 最大加速度の大きさ [m/s²], 正であること
            v_start: 始点速度 [m/s]
            v_end: 終点速度 [m/s]
        """
        # 最大加速度の符号を決定
        self.am = a_max if v_end > v_start else -a_max

        # 最大躍度の符号を決定
        self.jm = j_max if v_end > v_start else -j_max

        # 初期値と最終値を代入
        self.v0 = v_start
        self.v3 = v_end
        self.t0 = 0.0  # 初期値をゼロとする
        self.x0 = 0.0  # 初期値をゼロとする

        # 速度が曲線となる部分の時間を決定
        tc = a_max / j_max

        # 等加速度直線運動の時間を決定
        tm = (self.v3 - self.v0) / self.am - tc

        # 等加速度直線運動の有無で分岐
        if tm > 0:
            # 速度: 曲線 -> 直線 -> 曲線
            self.t1 = self.t0 + tc
            self.t2 = self.t1 + tm
            self.t3 = self.t2 + tc
            self.v1 = self.v0 + self.am * tc / 2  # v(t) を積分
            self.v2 = self.v1 + self.am * tm  # v(t) を積分
            self.x1 = self.x0 + self.v0 * tc + self.am * tc * tc / 6  # x(t) を積分
            self.x2 = self.x1 + self.v1 * tm  # x(t) を積分
            # v(t) グラフの台形の面積より
            self.x3 = self.x0 + (self.v0 + self.v3) / 2 * (self.t3 - self.t0)
        else:
            # 速度: 曲線 -> 曲線
            tcp = np.sqrt((self.v3 - self.v0) / self.jm)  # 変曲までの時間
            self.t1 = self.t2 = self.t0 + tcp
            self.t3 = self.t2 + tcp
            self.v1 = self.v2 = (self.v0 + self.v3) / 2  # 対称性より中点となる
            # x(t) を積分
            self.x1 = self.x2 = self.x0 + self.v1 * tcp + self.jm * tcp**3 / 6
            self.x3 = self.x0 + 2 * self.v1 * tcp  # 速度 v(t) グラフの面積より

    def j(self, t: float) -> float:
        """時刻tにおける躍度を計算

        Args:
            t: 時刻 [s]

        Returns:
            躍度 [m/s³]
        """
        if t <= self.t0:
            return 0.0
        elif t <= self.t1:
            return self.jm
        elif t <= self.t2:
            return 0.0
        elif t <= self.t3:
            return -self.jm
        else:
            return 0.0

    def a(self, t: float) -> float:
        """時刻tにおける加速度を計算

        Args:
            t: 時刻 [s]

        Returns:
            加速度 [m/s²]
        """
        if t <= self.t0:
            return 0.0
        elif t <= self.t1:
            return self.jm * (t - self.t0)
        elif t <= self.t2:
            return self.am
        elif t <= self.t3:
            return -self.jm * (t - self.t3)
        else:
            return 0.0

    def v(self, t: float) -> float:
        """時刻tにおける速度を計算

        Args:
            t: 時刻 [s]

        Returns:
            速度 [m/s]
        """
        if t <= self.t0:
            return self.v0
        elif t <= self.t1:
            return self.v0 + self.jm / 2 * (t - self.t0)**2
        elif t <= self.t2:
            return self.v1 + self.am * (t - self.t1)
        elif t <= self.t3:
            return self.v3 - self.jm / 2 * (t - self.t3)**2
        else:
            return self.v3

    def x(self, t: float) -> float:
        """時刻tにおける位置を計算

        Args:
            t: 時刻 [s]

        Returns:
            位置 [m]
        """
        if t <= self.t0:
            return self.x0 + self.v0 * (t - self.t0)
        elif t <= self.t1:
            return self.x0 + self.v0 * (t - self.t0) + self.jm / 6 * (t - self.t0)**3
        elif t <= self.t2:
            return self.x1 + self.v1 * (t - self.t1) + self.am / 2 * (t - self.t1)**2
        elif t <= self.t3:
            return self.x3 + self.v3 * (t - self.t3) - self.jm / 6 * (t - self.t3)**3
        else:
            return self.x3 + self.v3 * (t - self.t3)

    def t_end(self) -> float:
        """終点時刻を取得

        Returns:
            終点時刻 [s]
        """
        return self.t3

    def v_end(self) -> float:
        """終点速度を取得

        Returns:
            終点速度 [m/s]
        """
        return self.v3

    def x_end(self) -> float:
        """終点位置を取得

        Returns:
            終点位置 [m]
        """
        return self.x3

    def t_0(self) -> float:
        """境界時刻t0を取得"""
        return self.t0

    def t_1(self) -> float:
        """境界時刻t1を取得"""
        return self.t1

    def t_2(self) -> float:
        """境界時刻t2を取得"""
        return self.t2

    def t_3(self) -> float:
        """境界時刻t3を取得"""
        return self.t3

    @staticmethod
    def calc_velocity_end(
        j_max: float,
        a_max: float,
        v_start: float,
        v_target: float,
        distance: float
    ) -> float:
        """走行距離から達しうる終点速度を算出する

        Args:
            j_max: 最大躍度の大きさ [m/s³], 正であること
            a_max: 最大加速度の大きさ [m/s²], 正であること
            v_start: 始点速度 [m/s]
            v_target: 目標速度 [m/s]
            distance: 走行距離 [m]

        Returns:
            終点速度 [m/s]
        """
        # 速度が曲線となる部分の時間を決定
        tc = a_max / j_max

        # 最大加速度の符号を決定
        am = a_max if v_target > v_start else -a_max
        jm = j_max if v_target > v_start else -j_max

        # 等加速度直線運動の有無で分岐
        d_triangle = (v_start + am * tc / 2) * tc  # distance @ tm == 0
        v_triangle = jm / am * distance - v_start  # v_end @ tm == 0

        if d_triangle * v_triangle > 0 and abs(distance) > abs(d_triangle):
            # 曲線・直線・曲線
            amtc = am * tc
            discriminant = amtc**2 - 4 * (amtc * v_start - v_start**2 - 2 * am * distance)
            sqrt_d = np.sqrt(discriminant)

            if distance > 0:
                return (-amtc + sqrt_d) / 2
            else:
                return (-amtc - sqrt_d) / 2

        # 曲線・曲線 (走行距離が短すぎる)
        # 3次方程式を解いて、終点速度を算出
        # 簡単のため、値を一度すべて正に変換して、計算結果に符号を付与して返送
        a = abs(v_start)
        b = jm * distance * distance if distance > 0 else -jm * distance * distance

        aaa = a**3
        c0 = 27 * (32 * aaa * b + 27 * b * b)
        c1 = 16 * aaa + 27 * b

        if c0 >= 0:
            # ルートの中が非負のとき
            c2 = np.cbrt((np.sqrt(c0) + c1) / 2)
            result = (c2 + 4 * a * a / c2 - a) / 3
            return result if distance > 0 else -result
        else:
            # ルートの中が負のとき
            c2 = np.power(complex(c1 / 2, np.sqrt(-c0) / 2), 1.0 / 3)
            result = (c2.real * 2 - a) / 3
            return result if distance > 0 else -result

    @staticmethod
    def calc_velocity_max(
        j_max: float,
        a_max: float,
        v_start: float,
        v_end: float,
        distance: float
    ) -> float:
        """走行距離から達しうる最大速度を算出する

        Args:
            j_max: 最大躍度の大きさ [m/s³], 正であること
            a_max: 最大加速度の大きさ [m/s²], 正であること
            v_start: 始点速度 [m/s]
            v_end: 終点速度 [m/s]
            distance: 走行距離 [m]

        Returns:
            最大速度 [m/s]
        """
        # 速度が曲線となる部分の時間を決定
        tc = a_max / j_max
        am = a_max if distance > 0 else -a_max  # 加速方向は移動方向に依存

        # 2次方程式の解の公式を解く
        amtc = am * tc
        discriminant = (
            amtc**2 - 2 * (v_start + v_end) * amtc
            + 4 * am * distance + 2 * (v_start**2 + v_end**2)
        )

        if discriminant < 0:
            # 拘束条件がおかしい
            print(f"[Error] Discriminant = {discriminant} < 0")

            # 入力のチェック
            if v_start * v_end < 0:
                print(f"[Error] 不正な入力速度です v_start: {v_start}, v_end: {v_end}")
                return v_start

        sqrt_d = np.sqrt(discriminant)

        if distance > 0:
            return (-amtc + sqrt_d) / 2  # 2次方程式の解
        else:
            return (-amtc - sqrt_d) / 2  # 2次方程式の解

    @staticmethod
    def calc_min_distance(
        j_max: float,
        a_max: float,
        v_start: float,
        v_end: float
    ) -> float:
        """速度差から変位を算出する

        Args:
            j_max: 最大躍度の大きさ [m/s³], 正であること
            a_max: 最大加速度の大きさ [m/s²], 正であること
            v_start: 始点速度 [m/s]
            v_end: 終点速度 [m/s]

        Returns:
            変位 [m]
        """
        # 符号付きで代入
        am = a_max if v_end > v_start else -a_max
        jm = j_max if v_end > v_start else -j_max

        # 速度が曲線となる部分の時間を決定
        tc = a_max / j_max

        # 等加速度直線運動の時間を決定
        tm = (v_end - v_start) / am - tc

        # 始点から終点までの時間を決定
        if tm > 0:
            t_all = tc + tm + tc
        else:
            t_all = 2 * np.sqrt((v_end - v_start) / jm)

        return (v_start + v_end) / 2 * t_all  # 速度グラフの面積により


class SpeedProfile:
    """拘束条件を満たす曲線加減速の軌道を生成するクラス

    移動距離の拘束条件を満たす曲線加速軌道を生成します。
    各時刻tにおける躍度j(t)、加速度a(t)、速度v(t)、位置x(t)を提供します。

    注意: 最大加速度a_maxと始点速度v_startなど拘束次第では
    目標速度に達することができない場合があります。

    Attributes:
        t0, t1, t2, t3: 境界点の時刻 [s]
        x0, x3: 境界点の位置 [m]
        ac: 加速曲線オブジェクト
        dc: 減速曲線オブジェクト
    """

    def __init__(
        self,
        j_max: float = 0.0,
        a_max: float = 0.0,
        v_sat: float = 0.0,
        v_start: float = 0.0,
        v_target: float = 0.0,
        dist: float = 0.0,
        x_start: float = 0.0,
        t_start: float = 0.0
    ) -> None:
        """コンストラクタ

        Args:
            j_max: 最大躍度の大きさ [m/s³]、正であること
            a_max: 最大加速度の大きさ [m/s²]、正であること
            v_sat: 飽和速度の大きさ [m/s]、正であること
            v_start: 始点速度 [m/s]
            v_target: 目標速度 [m/s]
            dist: 移動距離 [m]
            x_start: 始点位置 [m] (オプション)
            t_start: 始点時刻 [s] (オプション)
        """
        self.t0: float = 0.0
        self.t1: float = 0.0
        self.t2: float = 0.0
        self.t3: float = 0.0
        self.x0: float = 0.0
        self.x3: float = 0.0

        self.ac = AccelCurve()
        self.dc = AccelCurve()

        if j_max != 0.0:
            self.reset(j_max, a_max, v_sat, v_start, v_target, dist, x_start, t_start)

    def reset(
        self,
        j_max: float,
        a_max: float,
        v_sat: float,
        v_start: float,
        v_target: float,
        dist: float,
        x_start: float = 0.0,
        t_start: float = 0.0
    ) -> None:
        """引数の拘束条件から曲線を生成する

        この関数によって、すべての変数が初期化されます。

        Args:
            j_max: 最大躍度の大きさ [m/s³]、正であること
            a_max: 最大加速度の大きさ [m/s²]、正であること
            v_sat: 飽和速度の大きさ [m/s]、正であること
            v_start: 始点速度 [m/s]
            v_target: 目標速度 [m/s]
            dist: 移動距離 [m]
            x_start: 始点位置 [m] (オプション)
            t_start: 始点時刻 [s] (オプション)
        """
        # 最大速度の仮置き
        if dist > 0:
            v_max = max(v_start, v_sat, v_target)
        else:
            v_max = min(v_start, -v_sat, v_target)

        # 走行距離から終点速度v_eを算出
        v_end = v_target
        dist_min = AccelCurve.calc_min_distance(j_max, a_max, v_start, v_end)

        if abs(dist) < abs(dist_min):
            print("[Info] 走行距離の拘束を満たすため、最大速度まで加速できません")
            # 目標速度v_tに向かい、走行距離dで到達し得る終点速度v_eを算出
            v_end = AccelCurve.calc_velocity_end(j_max, a_max, v_start, v_target, dist)
            v_max = v_end  # 走行距離の拘束を満たすため、飽和速度まで加速できない

        # 曲線を生成
        self.ac.reset(j_max, a_max, v_start, v_max)  # 加速
        self.dc.reset(j_max, a_max, v_max, v_end)  # 減速

        # 飽和速度まで加速すると走行距離の拘束を満たさない場合の処理
        d_sum = self.ac.x_end() + self.dc.x_end()
        if abs(dist) < abs(d_sum):
            print("走行距離の拘束を満たすため、最大速度まで加速できません")
            # 走行距離から最大速度v_mを算出 下記v_maxは上記v_max以下になる
            v_max = AccelCurve.calc_velocity_max(j_max, a_max, v_start, v_end, dist)

            # 無駄な減速を回避
            if dist > 0:
                v_max = max(v_start, v_max, v_end)
            else:
                v_max = min(v_start, v_max, v_end)
            self.ac.reset(j_max, a_max, v_start, v_max)  # 加速
            self.dc.reset(j_max, a_max, v_max, v_end)  # 減速

        # 発散回避
        if v_max == 0:
            v_max = 1.0

        # 各定数の算出
        self.x0 = x_start
        self.x3 = x_start + dist
        self.t0 = t_start
        self.t1 = self.t0 + self.ac.t_end()  # 曲線加速終了の時刻
        # 等速走行終了の時刻
        self.t2 = self.t0 + self.ac.t_end() + (dist - self.ac.x_end() - self.dc.x_end()) / v_max
        # 曲線減速終了の時刻
        self.t3 = (
            self.t0 + self.ac.t_end()
            + (dist - self.ac.x_end() - self.dc.x_end()) / v_max
            + self.dc.t_end()
        )

        # 出力のチェック
        tolerance = 0.00001  # 数値誤差分
        show_info = False

        # 終点速度
        if abs(v_start - v_end) > tolerance + abs(v_start - v_target):
            print("[Error] 終端速度が不正です")
            show_info = True

        # 最大速度
        if abs(v_max) > tolerance + max(v_sat, abs(v_start), abs(v_end)):
            print("[Error] 最大速度が不正です")
            show_info = True

        # タイムスタンプ
        if not (self.t0 <= self.t1 + tolerance and
                self.t1 <= self.t2 + tolerance and
                self.t2 <= self.t3 + tolerance):
            print("[Error] 時間関係が不正です")
            show_info = True

    def j(self, t: float) -> float:
        """時刻tにおける躍度を計算

        Args:
            t: 時刻 [s]

        Returns:
            躍度 [m/s³]
        """
        if t < self.t2:
            return self.ac.j(t - self.t0)
        else:
            return self.dc.j(t - self.t2)

    def a(self, t: float) -> float:
        """時刻tにおける加速度を計算

        Args:
            t: 時刻 [s]

        Returns:
            加速度 [m/s²]
        """
        if t < self.t2:
            return self.ac.a(t - self.t0)
        else:
            return self.dc.a(t - self.t2)

    def v(self, t: float) -> float:
        """時刻tにおける速度を計算

        Args:
            t: 時刻 [s]

        Returns:
            速度 [m/s]
        """
        if t < self.t2:
            return self.ac.v(t - self.t0)
        else:
            return self.dc.v(t - self.t2)

    def x(self, t: float) -> float:
        """時刻tにおける位置を計算

        Args:
            t: 時刻 [s]

        Returns:
            位置 [m]
        """
        if t < self.t2:
            return self.x0 + self.ac.x(t - self.t0)
        else:
            return self.x3 - self.dc.x_end() + self.dc.x(t - self.t2)

    def t_end(self) -> float:
        """終点時刻を取得

        Returns:
            終点時刻 [s]
        """
        return self.t3

    def v_end(self) -> float:
        """終点速度を取得

        Returns:
            終点速度 [m/s]
        """
        return self.dc.v_end()

    def x_end(self) -> float:
        """終点位置を取得

        Returns:
            終点位置 [m]
        """
        return self.x3

    def t_0(self) -> float:
        """境界時刻t0を取得"""
        return self.t0

    def t_1(self) -> float:
        """境界時刻t1を取得"""
        return self.t1

    def t_2(self) -> float:
        """境界時刻t2を取得"""
        return self.t2

    def t_3(self) -> float:
        """境界時刻t3を取得"""
        return self.t3
