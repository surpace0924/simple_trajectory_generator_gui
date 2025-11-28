# -*- coding: utf-8 -*-

import os
from PySide6.QtWidgets import QSizePolicy, QTableWidgetItem
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import numpy as np
import colorsys


def createRectangle(pose, width, height, ec='#000000', fc='#FFFFFF', fill=False):
    point = []
    point.append((width/2, +height/2))
    point.append((-width/2, +height/2))
    point.append((-width/2, -height/2))
    point.append((+width/2, -height/2))

    for i, p in enumerate(point):
        x = pose[0]+(p[0]*np.cos(pose[2]) - p[1]*np.sin(pose[2]))
        y = pose[1]+(p[0]*np.sin(pose[2]) + p[1]*np.cos(pose[2]))
        point[i] = (x, y)

    return plt.Polygon((point[0], point[1], point[2], point[3]), ec=ec, fc=fc, fill=False)


def resizeByRange(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def resizeByScale(x, a, b):
    return a * (x - b)


def convertToMeter(px, scale, origin):
    """[px]から[m]へ単位を変換"""
    result = [0, 0]
    result[0] = scale * (px[0] - origin[0])
    result[1] = scale * (-px[1] + origin[1])
    return result


def convertToPx(meter, scale, origin):
    """[m]から[px]へ単位を変換"""
    result = [0, 0]
    result[0] = origin[0] + (meter[0] / scale)
    result[1] = origin[1] - (meter[1] / scale)
    return result


class PlotCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        # 各種パラメータ
        self.parent = parent
        self.origin = [380, 380]                                       # 原点の座標[px]
        self.scale = 0.01593                                      # 縮尺[m/px]
        self.map_path = os.path.dirname(__file__) + "/../map/2021.png"  # map画像のパス
        self.control_point = None
        self.traj = None
        self.origin_point = None
        self.map_data = None  # マップデータ
        self.map_renderer = None  # マップレンダラー

        # 描画エリアの初期設定
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.ax = fig.add_subplot(111)
        self.ax.set_position([0, 0, 1, 1])
        FigureCanvas.__init__(self, fig)
        self.setParent(parent)
        FigureCanvas.setSizePolicy(self,
                                   QSizePolicy.Expanding,
                                   QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)
        self.add_zoom_func()
        self.plot()

        # キャンバスクリック時のイベント関数を登録
        cid = fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.drawOrigin(self.origin)

    def setMapData(self, map_data, map_renderer):
        """マップデータとレンダラーを設定

        Args:
            map_data: MapDataインスタンス
            map_renderer: MapRendererインスタンス
        """
        self.map_data = map_data
        self.map_renderer = map_renderer

    def clearMap(self):
        """マップをクリア"""
        self.ax.clear()
        # 軸を消す
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        # デフォルトの表示範囲を取得して設定
        default_x_range, default_y_range = self.get_default_range()
        self.ax.set_xlim(default_x_range)
        self.ax.set_ylim(default_y_range)
        self.ax.set_aspect('equal')

    def plot(self):
        # マップデータがあればマップを描画、なければデフォルトフィールドを描画
        if self.map_data and self.map_renderer:
            self.map_renderer.render(self.ax, self.map_data)
        else:
            self._draw_default_field()

        # 描画
        self.draw()

    def _draw_default_field(self):
        """デフォルトのフィールド構造物を描画"""
        # 軸範囲を設定
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.set_xlim(-4, 4)
        self.ax.set_ylim(-6, 2)
        self.ax.set_aspect('equal')

        # フィールド構造物は描画しない（元のplot_canvasの実装ではplotメソッド内で描画していなかった）

    def drawOrigin(self, point):
        """原点を描画"""
        self.origin = point
        # すでに点が描かれている場合はそれを消す
        if self.origin_point != None:
            try:
                self.origin_point[0].remove()
            except (NotImplementedError, ValueError):
                # マップ要素など削除できない場合は、axisをクリアして再描画
                self.ax.clear()
                self.plot()

        dot_px = []
        plot_x, plot_y = self.__convertToPlot(dot_px)
        self.origin_point = self.ax.plot(0, 0, marker='.', markersize=12, color="yellow", linestyle='None')
        self.draw()

    def onclick(self, event):
        """mapクリック時の処理"""
        point = [event.xdata, event.ydata]
        point.append(0)  # 角度はとりあえず0としておく
        self.parent.via_points.append(point)
        self.parent.via_speed.append(0)

        # tableをクリアし，座標の数だけ行を用意
        items = self.parent.via_points
        # self.parent.tableWidget.clearContents()
        self.parent.tableWidget.setRowCount(len(items))

        # tableへの書き込み
        r = len(items)-1
        self.parent.tableWidget.setItem(
            r, 0, QTableWidgetItem('{0:.3f}'.format(items[r][0])))
        self.parent.tableWidget.setItem(
            r, 1, QTableWidgetItem('{0:.3f}'.format(items[r][1])))
        self.parent.tableWidget.setItem(
            r, 2, QTableWidgetItem('{0:.3f}'.format(items[r][2])))
        self.parent.tableWidget.setItem(
            r, 3, QTableWidgetItem('{0:.3f}'.format(0)))
        # print()

    def drawControlPoint(self, point):
        # すでに点が描かれている場合はそれを消す
        if self.control_point != None:
            try:
                self.control_point[0].remove()
            except (NotImplementedError, ValueError):
                # マップ要素など削除できない場合は、axisをクリアして再描画
                self.ax.clear()
                self.plot()

        plot_x = [l[0] for l in point]
        plot_y = [l[1] for l in point]
        self.control_point = self.ax.plot(
            plot_x, plot_y, marker='.', markersize=12, color="green", linestyle='None')
        self.draw()

    def drawTrajectory(self, dots, speed_profile):
        ax = self.figure.axes[0]
        dots = np.array(dots)
        dot_px = []

        # すでに経路が描かれている場合はそれを消す
        if self.traj != None:
            try:
                self.traj.remove()
            except (NotImplementedError, ValueError):
                # マップ要素など削除できない場合は、axisをクリアして再描画
                self.ax.clear()
                self.plot()
                # control_pointも再描画
                if self.control_point != None and hasattr(self.parent, 'via_points') and self.parent.via_points:
                    plot_x_ctrl = [l[0] for l in self.parent.via_points]
                    plot_y_ctrl = [l[1] for l in self.parent.via_points]
                    self.control_point = self.ax.plot(
                        plot_x_ctrl, plot_y_ctrl, marker='.', markersize=12, color="green", linestyle='None')

        plot_x = [l[0] for l in dots]
        plot_y = [l[1] for l in dots]

        color_list = []
        max_speed = max(speed_profile)
        for speed in speed_profile:
            rgb = colorsys.hsv_to_rgb(0.7*(1-(speed / max_speed)), 1, 1)
            color_list.append([rgb[0], rgb[1], rgb[2]])

        # 描画
        self.traj = self.ax.scatter(plot_x, plot_y, marker='.', c=color_list)
        # print(len(self.traj))
        # self.traj = self.ax.plot(plot_x, plot_y, marker='.', colors="red")
        self.draw()

    # Matplotlibに入れるために座標リストを2つのベクトルに変換する
    # [[x1, y1], [x2, y2], ...] -> [x1, x2, ...], [y1, y2, ...]
    def __convertToPlot(self, matrix):
        x = []
        y = []
        for i in range(len(matrix)):
            x.append(matrix[i][0])
            y.append(matrix[i][1])
        return x, y

    def get_default_range(self):
        """デフォルトの表示範囲を取得

        マップデータがあればマップの境界、なければデフォルト値を返す
        """
        if self.map_data and self.map_renderer:
            bounds = self.map_renderer.get_map_bounds(self.map_data)
            if bounds:
                x_min, x_max, y_min, y_max = bounds
                return (x_min, x_max), (y_min, y_max)
        return (-4, 4), (-6, 2)

    # スクロールで拡大縮小するやつ
    def add_zoom_func(self, base_scale=1.5):
        def zoom_func(event):
            bbox = self.ax.get_window_extent()
            if not (bbox.x0 < event.x < bbox.x1):
                return
            if not (bbox.y0 < event.y < bbox.y1):
                return
            if event.xdata is None or event.ydata is None:
                return
            cur_xlim = self.ax.get_xlim()
            cur_ylim = self.ax.get_ylim()
            cur_xrange = (cur_xlim[1] - cur_xlim[0]) * .5
            cur_yrange = (cur_ylim[1] - cur_ylim[0]) * .5
            xdata = event.xdata  # get event x location
            ydata = event.ydata  # get event y location
            # print(event.button, event.x, event.y, event.xdata, event.ydata)
            if event.button == 'up':
                # deal with zoom in
                scale_factor = base_scale
            elif event.button == 'down':
                # deal with zoom out
                scale_factor = 1 / base_scale
            else:
                # deal with something that should never happen
                scale_factor = 1
                # print(event.button)
            # set new limits
            xlim = [xdata - (xdata - cur_xlim[0]) / scale_factor,
                    xdata + (cur_xlim[1] - xdata) / scale_factor]
            ylim = [ydata - (ydata - cur_ylim[0]) / scale_factor,
                    ydata + (cur_ylim[1] - ydata) / scale_factor]

            self.ax.set_xlim(xlim)
            self.ax.set_ylim(ylim)

            # デフォルトの表示範囲を取得
            default_x_range, default_y_range = self.get_default_range()
            if (abs(xlim[1] - xlim[0]) > default_x_range[1] - default_x_range[0]) and (abs(ylim[1] - ylim[0]) > default_y_range[1] - default_y_range[0]):
                self.ax.set_xlim(default_x_range)
                self.ax.set_ylim(default_y_range)

            self.figure.canvas.draw()
        self.figure.canvas.mpl_connect('scroll_event', zoom_func)
