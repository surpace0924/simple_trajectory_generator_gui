# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt

class GraphViewer():
    def __init__(self):
        # 配色リスト[黒，青，赤，緑，黄，紫，水色]
        self.color_list = ["#000000", "#296fbc", "#cb360d",
                    "#3d9435", "#e1aa13", "#a54675", "#138bae"]

        # self.fig = plt.figure(figsize=(12.5/2.54, 11/2.54))
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(1, 1, 1)

        # 目盛のスタイル
        # plt.setp(self.ax1.get_xticklabels(), fontsize=10.5)
        # plt.setp(self.ax1.get_yticklabels(), fontsize=10.5)
        self.ax1.grid(ls="--")

        # グラフタイトル
        # plt.title('')

        # グラフ範囲
        # plt.xlim()
        # plt.ylim(0.0, 0.7)

        # 余白設定
        plt.subplots_adjust(left=0.11, right=0.95, bottom=0.13, top=0.95)


        # グラフの凡例
        # self.ax1.legend(fancybox=False, framealpha=1, edgecolor="#000000",
        #         loc='upper right', fontsize=10)


    def displayCurvatureProfile(self, length_list, curvature):
        self.ax1.plot(length_list, curvature, color=self.color_list[2], linewidth=1)
        self.ax1.set_xlabel("length [m]")
        self.ax1.set_ylabel("Curvature [1/m]")
        plt.show()

    def displaySpeedProfile(self, time_stamp, speed_profile):
        # self.ax1.plot(speed_profile, color=self.color_list[3], linewidth=1)
        self.ax1.plot(time_stamp, speed_profile, color=self.color_list[3], linewidth=1)
        self.ax1.set_xlabel("Time [s]")
        self.ax1.set_ylabel("Speed profile [m/s]")
        plt.show()

    def displayAngularSpeedProfile(self, time_stamp, angular_speed_profile):
        # self.ax1.plot(speed_profile, color=self.color_list[3], linewidth=1)
        self.ax1.plot(time_stamp, angular_speed_profile, color=self.color_list[6], linewidth=1)
        self.ax1.set_xlabel("Time [s]")
        self.ax1.set_ylabel("Speed profile [rad/s]")
        plt.show()

    def displayAccelerationProfile(self, time_stamp, acceleration_profile):
        self.ax1.plot(time_stamp, acceleration_profile, color=self.color_list[4], linewidth=1)
        self.ax1.set_xlabel("Time [s]")
        self.ax1.set_ylabel("Acceleration profile [m/s^2]")
        plt.show()

    def displayAngleProfile(self, time_stamp, angle_profile):
        self.ax1.plot(time_stamp, angle_profile, color=self.color_list[5], linewidth=1)
        self.ax1.set_xlabel("Time [s]")
        self.ax1.set_ylabel("angle profile [rad]")
        plt.show()

    def displayAll(self, speed_profile, acceleration_profile, curvature):
        self.ax1.plot(speed_profile, color=self.color_list[2], linewidth=1)
        self.ax1.set_xlabel("Time [s]")

        self.ax1.set_ylabel("speed profile")

        self.ax2 = self.ax1.twinx()
        self.ax3 = self.ax1.twinx()

        self.ax2.plot(acceleration_profile ,color=self.color_list[3], linewidth=1)
        self.ax2.set_ylabel("Acceleration profile [m/s^2]")

        self.ax3.plot(curvature ,color=self.color_list[3], linewidth=1)
        self.ax3.set_ylabel("curvature")

        self.ax3.spines["right"].set_position(("axes", 1.2))
        plt.show()

