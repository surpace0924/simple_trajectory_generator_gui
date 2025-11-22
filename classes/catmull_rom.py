# -*- coding: utf-8 -*-
"""Catmull-Rom曲線補間モジュール

このモジュールは、複数の経由点を滑らかな3次曲線で補間するための
Catmull-Rom補間アルゴリズムを提供します。
"""
from typing import List, Tuple
import numpy as np
import numpy.typing as npt


class CatmullRom:
    """Catmull-Rom曲線補間クラス

    複数の制御点（経由点）を与えると、それらを通る滑らかな3次曲線を生成します。
    始点・終点・中間点で異なる補間式を使用して、連続的な曲線を実現します。
    """

    def __init__(self) -> None:
        """コンストラクタ

        内部状態を初期化します。
        """
        self._control_points: List[npt.NDArray[np.float64]] = []
        self._result_points: List[npt.NDArray[np.float64]] = []
        self._length: float = 0.0

    def set_control_points(self, points: List[List[float]]) -> None:
        """制御点（経由点）を設定する

        Args:
            points: 制御点のリスト [[x, y, angle, speed], ...]
                   最初の2要素(x, y座標)のみを使用します
        """
        self._control_points = [np.array(point[:2]) for point in points]

    def get_result(self) -> Tuple[List[npt.NDArray[np.float64]], float, float]:
        """計算結果を取得する（レガシーメソッド）

        Returns:
            結果座標点のリスト、経路長、時刻のタプル

        Note:
            このメソッドは後方互換性のために残されています。
        """
        return self._result_points, self._length, 0.0

    def calculate(self, divisions: int) -> Tuple[
        List[npt.NDArray[np.float64]],
        List[float],
        List[float]
    ]:
        """曲線を計算する

        制御点を基にCatmull-Rom曲線を生成し、指定された分割数で
        経路上の座標点を計算します。

        Args:
            divisions: 経路全体を通して生成する点数

        Returns:
            (trajectory, length_list, control_point_length)のタプル
            - trajectory: 経路点の座標リスト
            - length_list: 各点の始点からの累積道のり [m]
            - control_point_length: 各制御点での累積道のり [m]
        """
        control_points = self._control_points
        trajectory: List[npt.NDArray[np.float64]] = []
        cumulative_length = 0.0
        length_list: List[float] = []
        point_count = 0
        control_point_length = [0.0]

        num_segments = len(control_points) - 1

        # 各区間について曲線を生成
        for i in range(num_segments):
            # 媒介変数tを0から1まで変化させて点を生成
            step = num_segments / divisions
            for t in np.arange(0, 1, step):
                # 区間の位置によって異なる補間式を使用
                if i == 0:
                    # 始点セグメント
                    point = self._calculate_first_segment(
                        t, control_points[i], control_points[i+1], control_points[i+2]
                    )
                elif i == num_segments - 1:
                    # 終点セグメント
                    point = self._calculate_last_segment(
                        t, control_points[i-1], control_points[i], control_points[i+1]
                    )
                else:
                    # 中間セグメント
                    point = self._calculate_middle_segment(
                        t, control_points[i-1], control_points[i],
                        control_points[i+1], control_points[i+2]
                    )

                trajectory.append(point)

                # 経路長は経路点間のユークリッド距離の積算で計算
                if point_count > 0:
                    cumulative_length += np.linalg.norm(
                        trajectory[point_count] - trajectory[point_count - 1]
                    )
                length_list.append(cumulative_length)
                point_count += 1

            control_point_length.append(length_list[-1])

        return trajectory, length_list, control_point_length

    def _calculate_first_segment(
        self,
        t: float,
        p0: npt.NDArray[np.float64],
        p1: npt.NDArray[np.float64],
        p2: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """始点セグメントの補間計算

        Args:
            t: 媒介変数 [0, 1]
            p0: 始点（制御点0）
            p1: 制御点1
            p2: 制御点2

        Returns:
            補間された座標点
        """
        b = p0 - 2 * p1 + p2
        c = -3 * p0 + 4 * p1 - p2
        d = 2 * p0
        return 0.5 * ((b * t * t) + (c * t) + d)

    def _calculate_middle_segment(
        self,
        t: float,
        p0: npt.NDArray[np.float64],
        p1: npt.NDArray[np.float64],
        p2: npt.NDArray[np.float64],
        p3: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """中間セグメントの補間計算

        標準的なCatmull-Rom補間式を使用します。

        Args:
            t: 媒介変数 [0, 1]
            p0: 前の制御点
            p1: セグメント始点
            p2: セグメント終点
            p3: 次の制御点

        Returns:
            補間された座標点
        """
        a = -p0 + 3 * p1 - 3 * p2 + p3
        b = 2 * p0 - 5 * p1 + 4 * p2 - p3
        c = -p0 + p2
        d = 2 * p1
        return 0.5 * ((a * t * t * t) + (b * t * t) + (c * t) + d)

    def _calculate_last_segment(
        self,
        t: float,
        p0: npt.NDArray[np.float64],
        p1: npt.NDArray[np.float64],
        p2: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """終点セグメントの補間計算

        Args:
            t: 媒介変数 [0, 1]
            p0: 制御点n-2
            p1: 制御点n-1
            p2: 終点（制御点n）

        Returns:
            補間された座標点
        """
        b = p0 - 2 * p1 + p2
        c = -p0 + p2
        d = 2 * p1
        return 0.5 * ((b * t * t) + (c * t) + d)
