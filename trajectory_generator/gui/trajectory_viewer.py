# -*- coding: utf-8 -*-
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import colorsys
from typing import Optional, Union
from trajectory_generator.models.robot_shape import RobotShape, get_robot_shape

class TrajectoryViewer:
    def __init__(
        self,
        poses,
        dot_hz,
        disp_period,
        robot: Optional[Union[dict, RobotShape]] = None,
        speed_profile = None
    ):
        """軌跡の可視化クラス

        Args:
            poses: ロボットの姿勢リスト
            dot_hz: 打点周波数 [Hz]
            disp_period: 表示周期 [sec]
            robot: ロボット形状
                   - 形状定義辞書 (dict) {"name": "...", "vertices": [...]}
                   - RobotShapeインスタンス
            speed_profile: 速度プロファイル
        """
        self.poses = poses
        self.dot_hz = dot_hz
        self.disp_period = disp_period
        self.speed_profile = speed_profile

        # RobotShape インスタンスを取得
        if robot is None:
            raise ValueError(
                "ロボット形状が指定されていません。\n"
                "設定JSONファイルに 'robot_shape' フィールドを追加してください。"
            )
        elif isinstance(robot, RobotShape):
            self.robot_shape = robot
            self.robot_name = robot.name
        elif isinstance(robot, dict):
            # 辞書形式の場合
            self.robot_shape = get_robot_shape(robot)
            self.robot_name = robot.get('name', 'CustomRobot')
        else:
            raise ValueError(
                f"ロボット形状は辞書形式で指定してください。\n"
                f"受け取った型: {type(robot)}"
            )

    def createRobot(self, pose, robot_shape=None, ec='#000000', fc='#FFFFFF', fill=False):
        """ロボット形状のポリゴンを生成

        Args:
            pose: ロボットの姿勢 (x, y, theta)
            robot_shape: RobotShapeインスタンス（Noneの場合はself.robot_shapeを使用）
            ec: 枠線の色
            fc: 塗りつぶしの色
            fill: 塗りつぶしフラグ

        Returns:
            matplotlib.pyplot.Polygon
        """
        if robot_shape is None:
            robot_shape = self.robot_shape

        # RobotShapeクラスを使って回転・移動した頂点を取得
        rotated_vertices = robot_shape.get_rotated_vertices(pose)

        return plt.Polygon(rotated_vertices, ec=ec, fc=fc, fill=fill)

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
                    ec='green')
                ax.add_patch(robot_patch)

        # 始点
        robot_patch = self.createRobot(
            [self.poses[0][0], self.poses[0][1], self.poses[0][2]],
            ec='blue')
        ax.add_patch(robot_patch)

        # 終点
        robot_patch = self.createRobot(
            [self.poses[-1][0], self.poses[-1][1], self.poses[-1][2]],
            ec='blue')
        ax.add_patch(robot_patch)

        plt.title("robot:" + self.robot_name + ", draw period:" + str(self.disp_period) + "[sec]")
        plt.axis('scaled')
        ax.set_aspect('equal')

        plt.show()
