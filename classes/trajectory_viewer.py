# -*- coding: utf-8 -*-
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import colorsys

class TrajectoryViewer:
    def __init__(self, poses, dot_hz, disp_period, robot = None, speed_profile = None):
        self.poses = poses
        self.dot_hz = dot_hz
        self.disp_period = disp_period
        self.robot = robot
        self.speed_profile = speed_profile

    def createRobot(self, pose, robot=None, ec='#000000', fc='#FFFFFF', fill=False):
        # ロボットの形状定義
        point = []
        if robot == "AR":
            point.append((+0.062, +0.541))
            point.append((+0.062, +0.447))
            point.append((+0.299, -0.187))
            point.append((+0.299, -0.363))
            point.append((-0.299, -0.363))
            point.append((-0.299, -0.187))
            point.append((-0.062, +0.447))
            point.append((-0.062, +0.541))
        if robot == "AR_extend":
            point.append((+0.020, +0.851))
            point.append((-0.020, +0.851))
            point.append((-0.020, +0.541))
            point.append((-0.062, +0.541))
            point.append((-0.062, +0.447))
            point.append((-0.275, -0.123))
            point.append((-0.432, -0.123))
            point.append((-0.432, -0.283))
            point.append((-0.299, -0.283))
            point.append((-0.299, -0.363))
            point.append((-0.020, -0.363))
            point.append((-0.020, -0.783))
            point.append((+0.020, -0.783))
            point.append((+0.020, -0.363))
            point.append((+0.299, -0.363))
            point.append((+0.299, -0.283))
            point.append((+0.432, -0.283))
            point.append((+0.432, -0.123))
            point.append((+0.275, -0.123))
            point.append((+0.062, +0.447))
            point.append((+0.062, +0.541))
            point.append((+0.020, +0.541))
            point.append((+0.020, +0.851))
        elif robot == "TR":
            point.append((+0.475, +0.250))
            point.append((+0.070, +0.250))
            point.append((+0.041, +0.327))
            point.append((-0.041, +0.327))
            point.append((-0.070, +0.250))
            point.append((-0.475, +0.250))
            point.append((-0.475, -0.250))
            point.append((+0.475, -0.250))

            # point.append((0.3531+0.141, +0.227))
            # point.append((0.3531+0.049, +0.261))
            # point.append((0.3531-0.098, +0.261))
            # point.append((+0.230, +0.260))
            # point.append((+0.161, +0.301))
            # point.append((+0.066, +0.301))
            # point.append((+0.045, +0.380))
            # point.append((-0.045, +0.380))
            # point.append((-0.066, +0.301))
            # point.append((-0.161, +0.301))
            # point.append((-0.230, +0.260))
            # point.append((-(0.3531-0.098), +0.261))
            # point.append((-(0.3531+0.049), +0.261))
            # point.append((-(0.3531+0.141), +0.227))
            # point.append((-(0.3531+0.141), -0.227))
            # point.append((-(0.3531+0.049), -0.261))
            # point.append((-(0.3531-0.098), -0.261))
            # point.append((-0.230, -0.260))
            # point.append((-0.161, -0.301))
            # point.append((+0.161, -0.301))
            # point.append((+0.230, -0.260))
            # point.append((0.3531-0.098, -0.261))
            # point.append((0.3531+0.049, -0.261))
            # point.append((0.3531+0.141, -0.227))

        else:
            point.append((+0.500, +0.500))
            point.append((-0.500, +0.500))
            point.append((-0.500, -0.500))
            point.append((+0.500, -0.500))

        # 回転変換
        rotated_point = np.empty((0,2), float)
        for i, p in enumerate(point):
            x = pose[0]+(p[0]*np.cos(pose[2]) - p[1]*np.sin(pose[2]))
            y = pose[1]+(p[0]*np.sin(pose[2]) + p[1]*np.cos(pose[2]))
            rotated_point = np.append(rotated_point, np.array([[x, y]]), axis=0)

        return plt.Polygon(rotated_point, ec=ec, fc=fc, fill=fill)

    def createRectangle(self, pose, width, height, ec='#000000', fc='#FFFFFF', fill=False):
        point = []
        point.append((width/2, +height/2))
        point.append((-width/2, +height/2))
        point.append((-width/2, -height/2))
        point.append((+width/2, -height/2))

        for i, p in enumerate(point):
            x = pose[0]+(p[0]*np.cos(pose[2]) - p[1]*np.sin(pose[2]))
            y = pose[1]+(p[0]*np.sin(pose[2]) + p[1]*np.cos(pose[2]))
            point[i] = (x, y)

        return plt.Polygon((point[0], point[1], point[2], point[3]), ec=ec, fc=fc, fill=fill)

    def display(self):
        fig = plt.figure()
        ax = plt.axes()

        # フィールド構造物の描画
        field = []
        field.append(plt.Polygon(((+3.450, -5.950),
                                  (+3.450, -4.000),
                                  (-0.025, -4.000),
                                  (-0.025, -3.950),
                                  (+3.450, -3.950),
                                  (+3.450, +1.950),
                                  (-3.450, +1.950),
                                  (-3.450, -3.950),
                                  (-1.025, -3.950),
                                  (-1.025, -4.000),
                                  (-3.450, -4.000),
                                  (-3.450, -5.950)),
                                ec='#000000', fill=False))
        # テーブル
        field.append(self.createRectangle([0.0, 0.0, 0.0], 1.150, 0.330, fill=True))
        field.append(self.createRectangle([0.0, -2.5, 0.0], 1.150, 0.330, fill=True))
        field.append(patches.Circle(xy=(-2, 0.28), radius=0.46, ec='#000000', fill=False))
        field.append(patches.Circle(xy=(-2, -0.28), radius=0.46, ec='#000000', fill=False))
        field.append(self.createRectangle([-2.0, 0.0, 0.0], 1.0, 0.560, ec='#FFFFFF', fill=True))
        for f in field:
            ax.add_patch(f)

        # スタートゾーン
        ax.plot([3.45, 2.45], [-4.95, -4.95], "black", linewidth = 1)
        ax.plot([2.45, 2.45], [-4.95, -5.95], "black", linewidth = 1)
        ax.plot([-0.025, -0.025], [-4.000, -3.000], "black", linewidth = 1)
        ax.plot([-1.025, -0.025], [-3.000, -3.000], "black", linewidth = 1)
        ax.plot([-1.025, -1.025], [-3.000, -4.000], "black", linewidth = 1)
        ax.plot([-1.025, -0.025], [-4.000, -4.000], "black", linewidth = 1)

        # ラック
        ax.plot([-2.45, -3.45], [-5.45, -5.45], "black", linewidth = 1)
        ax.plot([-2.45, -2.45], [-5.45, -5.95], "black", linewidth = 1)
        ax.plot([-2.45, -3.45], [-5.63, -5.63], "black", linewidth = 1)

        # 速度に応じて打点色を変える
        color_list = []
        max_speed = max(self.speed_profile)
        for speed in self.speed_profile:
            rgb = colorsys.hsv_to_rgb(0.7*(1-(speed / max_speed)), 1, 1)
            color_list.append([rgb[0], rgb[1], rgb[2]])

        for i, pose in enumerate(self.poses):
            ax.scatter(pose[0], pose[1], marker='.', color=color_list[i])
            if i == 0:
                continue
            if (i%int(self.disp_period*self.dot_hz) == 0):
                robot_patch = self.createRobot(
                    [pose[0], pose[1], pose[2]],
                    robot = self.robot,
                    ec='green')
                ax.add_patch(robot_patch)

        # 始点
        robot_patch = self.createRobot(
            [self.poses[0][0], self.poses[0][1], self.poses[0][2]],
            robot = self.robot,
            ec='blue')
        ax.add_patch(robot_patch)

        # 終点
        robot_patch = self.createRobot(
            [self.poses[-1][0], self.poses[-1][1], self.poses[-1][2]],
            robot = self.robot,
            ec='blue')
        ax.add_patch(robot_patch)

        plt.title("robot:" + self.robot + ", draw period:" + str(self.disp_period) + "[sec]")
        plt.axis('scaled')
        ax.set_aspect('equal')

        plt.show()
