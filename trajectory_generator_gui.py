# -*- coding: utf-8 -*-
import sys
import os
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QApplication, QMainWindow, QMenu, QVBoxLayout, QSizePolicy, QMessageBox, QWidget, QPushButton,QComboBox,QListView,QLabel,QTableWidgetItem, QFileDialog
from PyQt5.QtGui import QIcon
from trajectory_generator_ui import Ui_MainWindow
import numpy as np
import json
import datetime
from collections import OrderedDict

from classes import trajectory_viewer
from classes import graph_viewer
from classes import trajectory_calculator
from classes import plot_canvas

import matplotlib.pyplot as plt

class TrajectoryGeneratorGui(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(TrajectoryGeneratorGui, self).__init__(parent)
        self.setupUi(self)

        # matplotlibの領域を描画
        self.canvas = plot_canvas.PlotCanvas(self, width=4.5, height=3.8)
        self.canvas.move(20, 100)

        # 設定ファイルに書き込まれるパラメータ辞書
        self.app_param = {}

        # 経路計算機
        self.tc = trajectory_calculator.TrajectoryCalculator()
        self.via_points = []    # 経由点の座標リスト
        self.via_speed = []     # 経由点での司令速度リスト
        self.use_via_angle = [] # 経由点の角度を使用するかのリスト
        self.use_via_speed = [] # 経由点での司令速度を使用するかのリスト


    @pyqtSlot()
    def button_generate_Click(self):
        # 制約条件の設定
        try:
            self.tc.setMaxLinearJerk(float(self.lineEdit_01.text()))
            self.tc.setMaxLinearAcceleration(float(self.lineEdit_02.text()))
            self.tc.setMaxLinearSpeed(float(self.lineEdit_03.text()))
            self.tc.setFrequency(float(self.lineEdit_07.text()))
            self.tc.setDotGap(float(self.lineEdit_08.text()))

        except ValueError:
            self.textBrowser.append("")
            self.textBrowser.append("----- " + str(datetime.datetime.now()) + " -----")
            self.textBrowser.append("[Error] 制約条件の入力値が不正です")
            self.redraw()
            return

        # 経由点の座標と速度を設定
        self.updateViaPoints()

        if len(self.via_points) < 3:
            self.printLog("[Error] 3つ以上の経由点を指定してください")
            return

        self.tc.setViaPoint(np.array(self.via_points))
        self.tc.setViaSpeed(self.via_speed)
        self.tc.setUseViaAngle(self.use_via_angle)
        self.tc.setUseViaSpeed(self.use_via_speed)

        # 計算
        self.tc.calculate()

        # 結果表示
        self.textBrowser.append("")
        self.textBrowser.append("----- " + str(datetime.datetime.now()) + " -----")
        self.textBrowser.append("経路生成終了")
        self.textBrowser.append("経路長　: " + '{0:.3f}'.format(self.tc.getTrajectoryLength()) + " [m]")
        self.textBrowser.append("到達時間: " + '{0:.3f}'.format(self.tc.getExpectedTime()) + " [sec]")
        self.textBrowser.append("最大速度: " + '{0:.3f}'.format(max(self.tc.getSpeedProfileProfile())) + " [m/s]")

        # 描画
        self.canvas.drawTrajectory(self.tc.getTrajectory(), self.tc.getSpeedProfileProfile())
        self.redraw()

    def redraw(self):
        self.resize(self.width(), self.height()+1)
        self.resize(self.width(), self.height()-1)


    # ファイル保存
    def button_export_Click(self):
        # 保存先の取得
        fname, selectedFilter = QFileDialog.getSaveFileName(self, 'ファイルの保存', 'trajectory.csv')
        if fname == "":
            return

        # 書き込むテキストの生成
        # float64 tolerance_linear
        # float64 tolerance_angular
        # float64 max_speed_linear
        # float64 max_speed_angular
        # float64 max_acceleration_linear
        # float64 max_acceleration_angular
        # float64 max_jerk_linear
        # float64 max_jerk_angular
        max_acceleration_angular = max(self.tc.getAngularSpeedProfileProfile())
        if abs(min(self.tc.getAngularSpeedProfileProfile())) > max_acceleration_angular:
            max_acceleration_angular = abs(min(self.tc.getAngularSpeedProfileProfile()))
        if max_acceleration_angular < 1:
            max_acceleration_angular = 1
        write_text = self.lineEdit_8.text() + "," + \
                     self.lineEdit_9.text() + "," + \
                     str(max(self.tc.getSpeedProfileProfile())) + "," + \
                     str(max_acceleration_angular) + "\n"


        for i in range(len(self.tc.getTrajectory())):
            write_text += str(self.tc.getTrajectory()[i][0])
            write_text += ","
            write_text += str(self.tc.getTrajectory()[i][1])
            write_text += ","
            write_text += str(self.tc.getTrajectory()[i][2])
            write_text += "\n"

        # 書き込み
        self.saveFile(fname, write_text)
        self.redraw()

    # セルを一つ上に
    def button_cell_up_Click(self):
        current_row = self.tableWidget.currentRow()
        self.exchangeTalbeRow(current_row - 1, current_row)
        self.redraw()

    # セルを一つ下に
    def button_cell_down_Click(self):
        current_row = self.tableWidget.currentRow()
        self.exchangeTalbeRow(current_row + 1, current_row)
        self.redraw()

    # tableの変更イベント
    def cell_changed(self):
        # tableの値をリストに書き込む
        self.updateViaPoints()

        # 制御点を再描画
        self.canvas.drawControlPoint(self.via_points)

    def button_cell_add_Click(self):
        self.redraw()

    def button_deg2rad(self):
        deg = float(self.lineEdit_10.text())
        rad = deg*np.pi/180
        self.lineEdit_11.setText('{0:.5f}'.format(rad))

    def button_rad2deg(self):
        rad = float(self.lineEdit_11.text())
        deg = rad/np.pi*180
        self.lineEdit_10.setText('{0:.5f}'.format(deg))

    def button_cell_delete_Click(self):
        # 削除する行の特定
        current_row = self.tableWidget.currentRow()

        # Tableから行を削除
        self.tableWidget.removeRow(current_row)

        # リストを更新
        self.updateViaPoints()

        # 点の再描画
        self.canvas.drawControlPoint(self.via_points)
        self.redraw()

    def printLog(self, text):
        self.textBrowser.append("")
        self.textBrowser.append("----- " + str(datetime.datetime.now()) + " -----")
        self.textBrowser.append(text)

    # 設定ファイル読み込み
    def button_select_settingfile(self):
        fname, selectedFilter = QFileDialog.getOpenFileName(self, 'ファイルの読み込み', 'setting.json')
        if fname == '':
            return

        with open(fname) as f:
            self.app_param = json.load(f)

        # 各lineEdit，tableに反映
        try:
            self.lineEdit_00.setText(fname)
            self.lineEdit_01.setText(self.app_param['max_linear_jerk'])
            self.lineEdit_02.setText(self.app_param['max_linear_acceleration'])
            self.lineEdit_03.setText(self.app_param['max_linear_speed'])
            self.lineEdit_07.setText(self.app_param['hz'])
            self.lineEdit_08.setText(self.app_param['precision'])
            self.lineEdit_04.setText(self.app_param['disp_period'])
            self.lineEdit_8.setText(self.app_param['linear_tolerance'])
            self.lineEdit_9.setText(self.app_param['angular_tolerance'])
        except KeyError:
            print("不正な設定ファイルです．一部のデータの読み込みに失敗しました．")

        index = self.comboBox.findText(self.app_param['robot'])
        if index >= 0:
            self.comboBox.setCurrentIndex(index)

        # tableをクリアし，座標の数だけ行を用意
        items = self.app_param['table']
        self.tableWidget.clearContents()
        self.tableWidget.setRowCount(len(items))

        # tableへの書き込み
        for r, item in enumerate(items):
            for c in range(len(item)):
                self.tableWidget.setItem(r, c, QTableWidgetItem(item[c]))
        self.redraw()


    # 設定ファイルのエクスポート
    def button_export_settingfile(self):
        # 制約条件など
        self.app_param['max_linear_jerk']          = self.lineEdit_01.text()
        self.app_param['max_linear_acceleration']  = self.lineEdit_02.text()
        self.app_param['max_linear_speed']         = self.lineEdit_03.text()
        self.app_param['hz']                       = self.lineEdit_07.text()
        self.app_param['precision']                = self.lineEdit_08.text()
        self.app_param['linear_tolerance']         = self.lineEdit_8.text()
        self.app_param['angular_tolerance']        = self.lineEdit_9.text()
        self.app_param['disp_period']              = self.lineEdit_04.text()
        self.app_param['robot']                    = self.comboBox.currentText()

        # Tableデータ
        table_val = []
        for row in range(self.tableWidget.rowCount()):
            row_data = []
            for column in range(4):
                row_data.append(self.tableWidget.item(row, column).text())
            table_val.append(row_data)
        self.app_param['table'] = table_val

        # 保存先の取得
        fname, selectedFilter = QFileDialog.getSaveFileName(self, 'ファイルの保存', 'setting.json')
        if fname == '':
            return
        # 辞書型をjsonの文字列に変換
        write_text = json.dumps(self.app_param)
        # 書き込み
        self.saveFile(fname, write_text)
        self.redraw()

    def button_open_map(self):
        self.redraw()

    def button_adjust_origin(self):
        point = [0, 0]
        point[0] = float(self.lineEdit_6.text())
        point[1] = float(self.lineEdit_7.text())
        self.canvas.drawOrigin(point)
        self.redraw()

    def button_curvature_check(self):
        gv = graph_viewer.GraphViewer()
        gv.displayCurvatureProfile(self.tc.getTrajectoryLengthList(), self.tc.getCurvature())
        self.redraw()

    def button_speed_check(self):
        gv = graph_viewer.GraphViewer()
        gv.displaySpeedProfile(self.tc.getTimeStamp(), self.tc.getSpeedProfileProfile())
        self.redraw()

    def button_angular_speed_check(self):
        gv = graph_viewer.GraphViewer()
        gv.displayAngularSpeedProfile(self.tc.getTimeStamp(), self.tc.getAngularSpeedProfileProfile())
        self.redraw()

    def button_acceleration_check(self):
        gv = graph_viewer.GraphViewer()
        gv.displayAccelerationProfile(self.tc.getTimeStamp(), self.tc.getAccelerationProfile())
        self.redraw()

    def button_angle_check(self):
        gv = graph_viewer.GraphViewer()
        gv.displayAngleProfile(self.tc.getTimeStamp(), [l[2] for l in self.tc.getTrajectory()])
        self.redraw()

    def button_view_result(self):
        tv = trajectory_viewer.TrajectoryViewer(
            poses = self.tc.getTrajectory(),
            dot_hz = self.tc.getFrequency(),
            disp_period = float(self.lineEdit_04.text()),
            robot = self.comboBox.currentText(),
            speed_profile = self.tc.getSpeedProfileProfile())
        tv.display()
        self.redraw()

    # tableの行を入れ替える
    def exchangeTalbeRow(self, row1, row2):
        for col in range(self.tableWidget.columnCount()):
                temp = self.tableWidget.takeItem(row1, col)
                self.tableWidget.setItem(row1, col, self.tableWidget.takeItem(row2, col))
                self.tableWidget.setItem(row2, col, temp)
        self.tableWidget.setCurrentCell(row1, self.tableWidget.currentColumn())

    # tableの値を読み込み，配列化
    def updateViaPoints(self):
        row_num = self.tableWidget.rowCount()
        self.use_via_angle = [True]*row_num
        self.use_via_speed = [True]*row_num

        # Tableを走査していき値を代入
        table_data_list = []
        for row in range(row_num):
            row_data = []
            for column in range(4):
                # 各要素を読み込み
                try:
                    item_str = self.tableWidget.item(row, column).text()
                except AttributeError:
                    return -1
                try:
                    item_float = float(item_str)

                except ValueError:
                    # 空欄判定
                    if column == 2:
                        self.use_via_angle[row] = False
                    if column == 3:
                        self.use_via_speed[row] = False

                row_data.append(item_float)
            table_data_list.append(row_data)

        self.via_points = [l[:3] for l in table_data_list]
        self.via_speed = [l[3] for l in table_data_list]


    def saveFile(self, name, text):
        with open(name, mode='w') as f:
            f.write(text)

if __name__ == '__main__':
    argvs = sys.argv
    app = QApplication(argvs)
    trajectory_generator_gui = TrajectoryGeneratorGui()
    trajectory_generator_gui.show()
    sys.exit(app.exec_())

