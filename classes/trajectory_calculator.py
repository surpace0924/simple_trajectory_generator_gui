# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from classes import catmull_rom
from classes import speed_profile
from classes import min_jerk_profile

class TrajectoryCalculator():
    def __init__(self):
        # 各種制約条件
        self.__dot_gap = 0.001              # 打点間隔[m]
        self.__hz = 1                       # 制御周期[Hz]

        self.__max_linear_jerk = 1          # 並進最高躍度度[m/s^3]
        self.__max_linear_acceleration = 1  # 並進最高加速度[m/s^2]
        self.__max_linear_speed = 1         # 並進最高速度[m/s]

        self.__via_points = []              # 経由点
        self.__via_speed = []               # 経由点における終端速度

        self.__use_via_angle = []           # 経由点における角度を使用するか
        self.__use_via_speed = []           # 経由点における終端速度を使用するか

        # 出力
        self.__trajectory = []
        self.__curvature = []
        self.__speed_profile = []
        self.__angular_speed_profile = []
        self.__acceleration_profile = []
        self.__trajectory_length_list = []
        self.__time_stamp = []
        self.__time = 0


    def setDotGap(self, dot_gap):
        self.__dot_gap = dot_gap

    def setFrequency(self, frequency):
        self.__hz = frequency


    def setMaxLinearJerk(self, max_linear_jerk):
        self.__max_linear_jerk = max_linear_jerk

    def setMaxLinearAcceleration(self, max_linear_acceleration):
        self.__max_linear_acceleration = max_linear_acceleration

    def setMaxLinearSpeed(self, max_linear_speed):
        self.__max_linear_speed = max_linear_speed

    def setViaPoint(self, via_points):
        self.__via_points = via_points

    def setViaSpeed(self, via_speed):
        self.__via_speed = via_speed

    def setUseViaAngle(self, use_via_angle):
        self.__use_via_angle = use_via_angle

    def setUseViaSpeed(self, use_via_speed):
        self.__use_via_speed = use_via_speed


    def getFrequency(self):
        return self.__hz

    def getTrajectory(self):
        return self.__trajectory

    def getCurvature(self):
        return self.__curvature

    def getTrajectoryLengthList(self):
        return self.__trajectory_length_list

    def getTrajectoryLength(self):
        return self.__trajectory_length_list[-1]

    def getTimeStamp(self):
        return self.__time_stamp

    def getExpectedTime(self):
        return self.__time_stamp[-1]

    def getSpeedProfileProfile(self):
        return self.__speed_profile

    def getAngularSpeedProfileProfile(self):
        return self.__angular_speed_profile

    def getAccelerationProfile(self):
        return self.__acceleration_profile

    def getNearestTime(self, p):
        tmp_dist = 1000000
        idx = -1
        for i, traj in enumerate(self.__trajectory):
            dist = np.sqrt((traj[0] - p[0])**2 + (traj[1] - p[1])**2)
            if dist < tmp_dist:
                idx = i
                tmp_dist = dist
        return self.__time_stamp[idx]

    # 計算
    def calculate(self):
        cr = catmull_rom.CatmullRom()
        cr.setControlPoint(self.__via_points)

        # 経路の分割数を計算するために大まかな経路長を算出する
        about_length = 0
        for i in range(len(self.__via_points)-1):
            dx = self.__via_points[i + 1][0] - self.__via_points[i][0]
            dy = self.__via_points[i + 1][1] - self.__via_points[i][1]
            dl = np.sqrt(dx ** 2 + dy ** 2)
            about_length += dl

        # 経路打点数の約10倍の個数で経路を分割する
        div_num = int(10 * about_length / self.__dot_gap)

        # 曲線の計算
        curve, curve_length_list, via_points_length = cr.calculate(div_num)
        curve_length = curve_length_list[-1]

        section_length = [0]
        via_speed_list = []
        pre_idx = 0
        for i, length in enumerate(via_points_length):
            if self.__use_via_speed[i] and i != 0:
                section_length.append(length - via_points_length[pre_idx])
                via_speed_list.append(self.__via_speed[i])
                pre_idx = i
        section_length = section_length[1:]

        # プロファイル読み出し
        dot_profile = []
        self.__time_stamp = []
        self.__acceleration_profile = []
        self.__speed_profile = []
        sp = speed_profile.SpeedProfile()
        for i in range(len(section_length)):
            t0 = sp.t_end()
            sp.reset(
                j_max = self.__max_linear_jerk,
                a_max = self.__max_linear_acceleration,
                v_sat = self.__max_linear_speed,
                v_start = sp.v_end(),
                v_target = via_speed_list[i],
                dist = section_length[i],
                x_start = sp.x_end(),
                t_start = sp.t_end())

            for j in range(int((sp.t_end() - t0) * self.__hz)):
                t = j/self.__hz + t0
                self.__time_stamp.append(t)
                dot_profile.append(sp.x(t))
                self.__acceleration_profile.append(sp.a(t))
                self.__speed_profile.append(sp.v(t))

        # 最終点
        end_time = sp.t_end()
        dot_profile.append(sp.x(end_time))
        self.__time_stamp.append(end_time)
        self.__acceleration_profile.append(sp.a(end_time))
        self.__speed_profile.append(sp.v(end_time))

        # 打点プロファイルに一番近い点を曲線から計算し，それを経路とする
        self.__trajectory, self.__trajectory_length_list = self.__pickupTrajectory(curve, curve_length_list, dot_profile)

        # 角度計算
        # 角度制約がある経由点を通過する時刻のリスト
        via_time_stamp = [0]
        via_angle = [self.__via_points[0][2]]
        tmp_dist = 1000000
        for i, via in enumerate(self.__via_points):
            if self.__use_via_angle[i] == False:
                continue
            if i == 0 or i == len(self.__via_points)-1:
                continue
            time = self.getNearestTime(via)
            via_time_stamp.append(time)
            via_angle.append(self.__via_points[i][2])

        via_time_stamp.append(end_time)
        via_angle.append(self.__via_points[-1][2])

        # print(via_time_stamp)
        # print(via_angle)

        angles = np.empty(0)
        self.__angular_speed_profile = np.empty(0)
        for t in self.__time_stamp:
            # どの区間かを特定
            idx = 0
            for i, ts in enumerate(via_time_stamp):
                idx = i
                if t < ts:
                    break

            # 角度計算
            dt = via_time_stamp[idx] - via_time_stamp[idx-1]
            mjp = min_jerk_profile.MinJerkProfile(
                x_start = via_angle[i-1],
                x_target = via_angle[i],
                time = dt)
            angles = np.append(angles, mjp.x(t - via_time_stamp[idx-1]))
            self.__angular_speed_profile = np.append(
                self.__angular_speed_profile,
                mjp.v(t - via_time_stamp[idx-1]))

        # 角度要素を結合
        self.__trajectory = np.hstack([self.__trajectory, angles.reshape(len(angles), 1)])

        # 曲率計算
        self.__curvature = self.__calculateCurvature(self.__trajectory)
        for i, curvature in zip(range(len(self.__curvature)), self.__curvature):
            self.__curvature[i] = abs(curvature)


    # プロファイルにしたがって，一番近い座標を選択
    def __pickupTrajectory(self, curve, length_list, profile):
        trajectory = []
        trajectory_length_list = []
        for item in profile:
            length, idx = self.__getNearestValue(length_list, item)
            trajectory.append(curve[idx])
            trajectory_length_list.append(length_list[idx])
        return trajectory, trajectory_length_list


    # リストからある値に最も近い値を返却する関数
    # @param list: データ配列
    # @param num: 対象値
    # @return 対象値に最も近い値
    def __getNearestValue(self, list, num):
        # リスト要素と対象値の差分を計算し最小値のインデックスを取得
        idx = np.abs(np.asarray(list) - num).argmin()
        return list[idx], idx


    # 曲率計算
    # @param curve: 曲線の座標リスト
    # @return 曲率リスト
    def __calculateCurvature(self, curve):
        curvatures = [0.0]
        for i in np.arange(1, len(curve)-1):
            dxn = curve[i][0]     - curve[i - 1][0]
            dxp = curve[i + 1][0] - curve[i][0]
            dyn = curve[i][1]     - curve[i - 1][1]
            dyp = curve[i + 1][1] - curve[i][1]
            dn = np.hypot(dxn, dyn)
            dp = np.hypot(dxp, dyp)

            # ゼロ除算対策
            if (dn + dp) < 0.0 or dn == 0.0 or dp == 0:
                curvatures.append(0)
                continue

            dx = 1.0 / (dn + dp) * (dp / dn * dxn + dn / dp * dxp)
            ddx = 2.0 / (dn + dp) * (dxp / dp - dxn / dn)
            dy = 1.0 / (dn + dp) * (dp / dn * dyn + dn / dp * dyp)
            ddy = 2.0 / (dn + dp) * (dyp / dp - dyn / dn)
            curvature = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2) ** 1.5)
            curvatures.append(curvature)

        curvatures.append(abs(curvatures[-1]))
        return curvatures
