# -*- coding: utf-8 -*-
import numpy as np

class CatmullRom():
    def __init__(self):
        self.control_point = []
        self.result_point = []
        self.length = 0

    def setControlPoint(self, points):
        self.control_point = [l[:2] for l in points]

    def getResult(self):
        return self.result_point, self.length, self.time

    # 計算
    # @param div: 経路全体を通して生成する点数
    def calculate(self, div):
        cp = self.control_point
        trajectory = []  # 経路点の座標
        length = 0       # 経路長[m]
        length_list = [] # 各点の始点からの道のり
        point_count = 0  # 点のループカウンタ
        control_point_length = [0]

        # 区間のループ
        for i in range(len(cp) - 1):
            # 媒介変数のループ
            for t in np.arange(0, 1, (len(cp)-1)/div):
                # 始点，終点，中間点で式が違うため，関数の振り分け
                if i == 0:
                    trajectory.append(self.__calculateFirst(t, cp[i], cp[i+1], cp[i+2]))
                elif i == len(cp) - 2:
                    trajectory.append(self.__calculateLast(t, cp[i-1], cp[i], cp[i+1]))
                else:
                    trajectory.append(self.__calculateMiddle(t, cp[i-1], cp[i], cp[i+1], cp[i+2]))

                # 経路長は経路点間のユークリッド距離の積算で計算
                length += np.linalg.norm(trajectory[point_count] - trajectory[point_count - 1])
                length_list.append(length)
                point_count += 1

            control_point_length.append(length_list[-1])

        return trajectory, length_list, control_point_length

    # 各セクションの計算
    def __calculateFirst(self, t, p0, p1, p2):
        b = p0 - 2 * p1 + p2
        c = -3 * p0 + 4 * p1 - p2
        d = 2 * p0
        return 0.5 * ((b * t * t) + (c * t) + d)

    def __calculateMiddle(self, t, p0, p1, p2, p3):
        a = -1 * p0 + 3 * p1 - 3 * p2 + p3
        b = 2 * p0 - 5 * p1 + 4 * p2 - p3
        c = -1 * p0 + p2
        d = 2 * p1
        return 0.5 * ((a * t * t * t) + (b * t * t) + (c * t) + d)

    def __calculateLast(self, t, p0, p1, p2):
        b = p0 - 2 * p1 + p2
        c = -p0 + p2
        d = 2 * p1
        return 0.5 * ((b * t * t) + (c * t) + d)
