# -*- coding: utf-8 -*-
"""経路生成GUIアプリケーション

このモジュールは、ロボットの経路生成を行うためのGUIアプリケーションです。
経由点の設定、制約条件の指定、経路の生成・可視化を行います。
"""
import sys
import os
import json
import datetime
from typing import List, Dict, Any, Optional

import numpy as np
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QTableWidgetItem,
    QFileDialog
)

from trajectory_generator.resources.trajectory_generator_ui import Ui_MainWindow
from trajectory_generator.gui import trajectory_viewer
from trajectory_generator.gui import graph_viewer
from trajectory_generator.core.trajectory_planner import TrajectoryPlanner
from trajectory_generator.models.models import TrajectoryConfig, ViaPoint, TrajectoryResult
from trajectory_generator.exceptions import (
    InvalidViaPointError,
    InvalidConfigurationError,
    CalculationError
)
from trajectory_generator.gui import plot_canvas

# 定数定義
CANVAS_X_POSITION = 20
CANVAS_Y_POSITION = 100
CANVAS_WIDTH = 4.5
CANVAS_HEIGHT = 3.8
MIN_VIA_POINTS = 3
TABLE_COLUMN_COUNT = 4
COLUMN_INDEX_ANGLE = 2
COLUMN_INDEX_SPEED = 3


class TrajectoryGeneratorGui(QMainWindow, Ui_MainWindow):
    """経路生成GUIメインウィンドウ

    ロボットの経路生成を行うためのGUIアプリケーションのメインクラスです。
    経由点の設定、制約条件の指定、経路の生成・可視化を行います。

    Attributes:
        canvas: matplotlib描画キャンバス
        app_param: アプリケーション設定パラメータ辞書
        result: 経路計算結果（TrajectoryResult）
        via_points: 経由点の座標リスト
        via_speed: 経由点での目標速度リスト
        use_via_angle: 経由点の角度制約を使用するかのリスト
        use_via_speed: 経由点の速度制約を使用するかのリスト
    """

    def __init__(self, parent: Optional[QMainWindow] = None) -> None:
        """コンストラクタ

        Args:
            parent: 親ウィンドウ（オプション）
        """
        super(TrajectoryGeneratorGui, self).__init__(parent)
        self.setupUi(self)

        # matplotlibの領域を描画
        self.canvas = plot_canvas.PlotCanvas(
            self,
            width=CANVAS_WIDTH,
            height=CANVAS_HEIGHT
        )
        self.canvas.move(CANVAS_X_POSITION, CANVAS_Y_POSITION)

        # 設定ファイルに書き込まれるパラメータ辞書
        self.app_param: Dict[str, Any] = {}

        # 経路計算結果（初期値はNone）
        self.result: Optional[TrajectoryResult] = None
        self.via_points: List[List[float]] = []  # 経由点の座標リスト
        self.via_speed: List[float] = []  # 経由点での目標速度リスト
        self.use_via_angle: List[bool] = []  # 経由点の角度制約を使用するか
        self.use_via_speed: List[bool] = []  # 経由点の速度制約を使用するか

        # 前回使用した設定ファイルのパスを保存するファイル（リポジトリ内）
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.last_config_file = os.path.join(script_dir, '.last_config.json')

        # 起動時に前回の設定ファイルを読み込む
        self._load_last_settings()

    @pyqtSlot()
    def button_generate_Click(self) -> None:
        """経路生成ボタンのクリックイベントハンドラ

        制約条件と経由点から経路を生成し、結果を表示します。
        """
        # 経由点の座標と速度を更新
        self._update_via_points()

        if len(self.via_points) < MIN_VIA_POINTS:
            self._print_log(f"[Error] {MIN_VIA_POINTS}つ以上の経由点を指定してください")
            return

        # 制約条件の設定
        try:
            # 角度の制約条件は度で入力されるのでラジアンに変換
            max_angular_jerk_deg = float(self.lineEdit_8.text())
            max_angular_accel_deg = float(self.lineEdit_9.text())
            max_angular_speed_deg = float(self.lineEdit_10.text())

            config = TrajectoryConfig(
                max_linear_jerk=float(self.lineEdit_01.text()),
                max_linear_acceleration=float(self.lineEdit_02.text()),
                max_linear_speed=float(self.lineEdit_03.text()),
                max_angular_jerk=np.deg2rad(max_angular_jerk_deg),
                max_angular_acceleration=np.deg2rad(max_angular_accel_deg),
                max_angular_speed=np.deg2rad(max_angular_speed_deg),
                control_frequency=int(self.lineEdit_07.text()),
                precision=float(self.lineEdit_08.text())
            )
        except (ValueError, InvalidConfigurationError) as e:
            self._print_log(f"[Error] 制約条件の入力値が不正です: {e}")
            return

        # ViaPointオブジェクトのリストを作成
        try:
            via_point_objects = []
            for i, (point, speed, use_angle, use_speed) in enumerate(
                zip(self.via_points, self.via_speed, self.use_via_angle, self.use_via_speed)
            ):
                via_point_objects.append(
                    ViaPoint(
                        position=[point[0], point[1]],
                        angle=point[2] if use_angle else None,
                        speed=speed if use_speed else None,
                        use_angle_constraint=use_angle,
                        use_speed_constraint=use_speed
                    )
                )
        except (ValueError, IndexError) as e:
            self._print_log(f"[Error] 経由点データの変換に失敗しました: {e}")
            return

        # 経路計画の実行
        try:
            planner = TrajectoryPlanner(config)
            self.result = planner.plan(via_point_objects)
        except InvalidViaPointError as e:
            self._print_log(f"[Error] 経由点が不正です: {e}")
            return
        except CalculationError as e:
            self._print_log(f"[Error] 経路計算中にエラーが発生しました: {e}")
            return
        except Exception as e:
            self._print_log(f"[Error] 予期しないエラーが発生しました: {e}")
            return

        # 結果表示
        max_speed = np.max(self.result.velocities)
        self._print_log(
            "経路生成終了\n"
            f"経路長　: {self.result.total_length:.3f} [m]\n"
            f"到達時間: {self.result.total_time:.3f} [sec]\n"
            f"最大速度: {max_speed:.3f} [m/s]"
        )

        # 描画
        self.canvas.drawTrajectory(
            self.result.positions,
            self.result.velocities
        )
        self._redraw()

    def _redraw(self) -> None:
        """ウィンドウを再描画する

        PyQt5の描画更新を強制するためのハック。
        ウィンドウサイズを1px変更して元に戻すことで再描画を促す。
        """
        self.resize(self.width(), self.height() + 1)
        self.resize(self.width(), self.height() - 1)

    def _print_log(self, text: str) -> None:
        """ログメッセージをテキストブラウザに出力する

        Args:
            text: 出力するメッセージ
        """
        self.textBrowser.append("")
        self.textBrowser.append(f"----- {datetime.datetime.now()} -----")
        self.textBrowser.append(text)

    @pyqtSlot()
    def button_export_Click(self) -> None:
        """経路データをCSVファイルにエクスポート"""
        if self.result is None:
            self._print_log("[Error] 先に経路を生成してください")
            return

        # 保存先の取得
        fname, _ = QFileDialog.getSaveFileName(
            self,
            'ファイルの保存',
            'trajectory.csv'
        )
        if not fname:
            return

        # 経路データの生成（ヘッダーなし）
        trajectory_lines = [
            f"{point[0]},{point[1]},{point[2]}\n"
            for point in self.result.positions
        ]

        # ファイルに書き込み
        write_text = ''.join(trajectory_lines)
        self._save_file(fname, write_text)
        self._redraw()

    @pyqtSlot()
    def button_cell_up_Click(self) -> None:
        """テーブルの選択行を1つ上に移動"""
        current_row = self.tableWidget.currentRow()
        self._exchange_table_row(current_row - 1, current_row)
        self._redraw()

    @pyqtSlot()
    def button_cell_down_Click(self) -> None:
        """テーブルの選択行を1つ下に移動"""
        current_row = self.tableWidget.currentRow()
        self._exchange_table_row(current_row + 1, current_row)
        self._redraw()

    @pyqtSlot()
    def cell_changed(self) -> None:
        """テーブルの内容が変更されたときのイベントハンドラ"""
        # tableの値をリストに書き込む
        self._update_via_points()

        # 制御点を再描画
        self.canvas.drawControlPoint(self.via_points)

    @pyqtSlot()
    def button_cell_add_Click(self) -> None:
        """経由点を追加"""
        self._redraw()

    @pyqtSlot()
    def button_cell_delete_Click(self) -> None:
        """選択された経由点を削除"""
        # 削除する行の特定
        current_row = self.tableWidget.currentRow()

        # Tableから行を削除
        self.tableWidget.removeRow(current_row)

        # リストを更新
        self._update_via_points()

        # 点の再描画
        self.canvas.drawControlPoint(self.via_points)
        self._redraw()

    @pyqtSlot()
    def button_select_settingfile(self) -> None:
        """設定ファイルを読み込んでGUIに反映"""
        fname, _ = QFileDialog.getOpenFileName(
            self,
            'ファイルの読み込み',
            'setting.json'
        )
        if not fname:
            return

        self._load_settings_from_file(fname)

    @pyqtSlot()
    def button_export_settingfile(self) -> None:
        """現在の設定をJSONファイルにエクスポート

        Note:
            テーブルの角度は度(deg)のまま保存されます。
            読み込み時に再度度として表示されます。
        """
        # 制約条件などをパラメータ辞書に保存
        self.app_param['max_linear_jerk'] = self.lineEdit_01.text()
        self.app_param['max_linear_acceleration'] = self.lineEdit_02.text()
        self.app_param['max_linear_speed'] = self.lineEdit_03.text()
        self.app_param['max_angular_jerk'] = self.lineEdit_8.text()
        self.app_param['max_angular_acceleration'] = self.lineEdit_9.text()
        self.app_param['max_angular_speed'] = self.lineEdit_10.text()
        self.app_param['hz'] = self.lineEdit_07.text()
        self.app_param['precision'] = self.lineEdit_08.text()
        self.app_param['disp_period'] = self.lineEdit_04.text()

        # Tableデータの取得（角度は度のまま保存）
        table_val = []
        for row in range(self.tableWidget.rowCount()):
            row_data = []
            for column in range(TABLE_COLUMN_COUNT):
                item = self.tableWidget.item(row, column)
                row_data.append(item.text() if item else "")
            table_val.append(row_data)
        self.app_param['table'] = table_val

        # 保存先の取得
        fname, _ = QFileDialog.getSaveFileName(
            self,
            'ファイルの保存',
            'setting.json'
        )
        if not fname:
            return

        # JSON形式で書き込み
        write_text = json.dumps(self.app_param, indent=2, ensure_ascii=False)
        self._save_file(fname, write_text)
        self._redraw()

    @pyqtSlot()
    def button_open_map(self) -> None:
        """マップを開く"""
        self._redraw()

    @pyqtSlot()
    def button_adjust_origin(self) -> None:
        """原点位置を調整"""
        try:
            point = [
                float(self.lineEdit_6.text()),
                float(self.lineEdit_7.text())
            ]
            self.canvas.drawOrigin(point)
            self._redraw()
        except ValueError:
            self._print_log("[Error] 原点座標の入力値が不正です")

    @pyqtSlot()
    def button_curvature_check(self) -> None:
        """曲率プロファイルを表示"""
        if self.result is None:
            self._print_log("[Error] 先に経路を生成してください")
            return

        gv = graph_viewer.GraphViewer()
        gv.displayCurvatureProfile(
            self.result.trajectory_length_list,
            self.result.curvatures
        )
        self._redraw()

    @pyqtSlot()
    def button_speed_check(self) -> None:
        """速度プロファイルを表示"""
        if self.result is None:
            self._print_log("[Error] 先に経路を生成してください")
            return

        gv = graph_viewer.GraphViewer()
        gv.displaySpeedProfile(
            self.result.timestamps,
            self.result.velocities
        )
        self._redraw()

    @pyqtSlot()
    def button_angular_speed_check(self) -> None:
        """角速度プロファイルを表示"""
        if self.result is None:
            self._print_log("[Error] 先に経路を生成してください")
            return

        gv = graph_viewer.GraphViewer()
        gv.displayAngularSpeedProfile(
            self.result.timestamps,
            self.result.angular_velocities
        )
        self._redraw()

    @pyqtSlot()
    def button_acceleration_check(self) -> None:
        """加速度プロファイルを表示"""
        if self.result is None:
            self._print_log("[Error] 先に経路を生成してください")
            return

        gv = graph_viewer.GraphViewer()
        gv.displayAccelerationProfile(
            self.result.timestamps,
            self.result.accelerations
        )
        self._redraw()

    @pyqtSlot()
    def button_angle_check(self) -> None:
        """角度プロファイルを表示"""
        if self.result is None:
            self._print_log("[Error] 先に経路を生成してください")
            return

        gv = graph_viewer.GraphViewer()
        angles = [point[2] for point in self.result.positions]
        gv.displayAngleProfile(self.result.timestamps, angles)
        self._redraw()

    @pyqtSlot()
    def button_view_result(self) -> None:
        """経路のアニメーション表示"""
        if self.result is None:
            self._print_log("[Error] 先に経路を生成してください")
            return

        try:
            disp_period = float(self.lineEdit_04.text())
            control_frequency = int(self.lineEdit_07.text())
        except ValueError:
            self._print_log("[Error] 表示周期または制御周波数の入力値が不正です")
            return

        # ロボット形状の取得
        if 'robot_shape' not in self.app_param or not self.app_param['robot_shape']:
            self._print_log("[Error] ロボット形状が設定されていません。設定JSONに 'robot_shape' フィールドを追加してください。")
            return

        try:
            tv = trajectory_viewer.TrajectoryViewer(
                poses=self.result.positions,
                dot_hz=control_frequency,
                disp_period=disp_period,
                robot=self.app_param['robot_shape'],
                speed_profile=self.result.velocities
            )
            tv.display()
            self._redraw()
        except ValueError as e:
            self._print_log(f"[Error] ロボット形状の読み込みに失敗しました: {e}")
            return

    def _exchange_table_row(self, row1: int, row2: int) -> None:
        """テーブルの2行を入れ替える

        Args:
            row1: 入れ替える行1のインデックス
            row2: 入れ替える行2のインデックス
        """
        for col in range(self.tableWidget.columnCount()):
            temp = self.tableWidget.takeItem(row1, col)
            self.tableWidget.setItem(row1, col, self.tableWidget.takeItem(row2, col))
            self.tableWidget.setItem(row2, col, temp)
        self.tableWidget.setCurrentCell(row1, self.tableWidget.currentColumn())

    def _update_via_points(self) -> None:
        """テーブルの値を読み込んで経由点リストを更新

        テーブルの各行から座標、角度、速度を読み取り、
        内部の経由点リストを更新します。空欄の場合は制約なしとして扱います。

        Note:
            テーブルの角度入力は度(deg)で、内部的にラジアン(rad)に変換されます。
        """
        row_num = self.tableWidget.rowCount()
        self.use_via_angle = [True] * row_num
        self.use_via_speed = [True] * row_num

        # Tableを走査していき値を代入
        table_data_list = []
        for row in range(row_num):
            row_data = []
            for column in range(TABLE_COLUMN_COUNT):
                # 各要素を読み込み
                try:
                    item = self.tableWidget.item(row, column)
                    if item is None:
                        return
                    item_str = item.text()
                except AttributeError:
                    return

                try:
                    item_float = float(item_str)

                    # 角度列の場合は度からラジアンに変換
                    if column == COLUMN_INDEX_ANGLE:
                        item_float = np.deg2rad(item_float)

                except ValueError:
                    # 空欄判定
                    if column == COLUMN_INDEX_ANGLE:
                        self.use_via_angle[row] = False
                    if column == COLUMN_INDEX_SPEED:
                        self.use_via_speed[row] = False
                    item_float = 0.0

                row_data.append(item_float)
            table_data_list.append(row_data)

        self.via_points = [row[:3] for row in table_data_list]
        self.via_speed = [row[3] for row in table_data_list]

    def _save_file(self, name: str, text: str) -> None:
        """テキストをファイルに保存

        Args:
            name: 保存先ファイルパス
            text: 保存するテキスト内容
        """
        try:
            with open(name, mode='w', encoding='utf-8') as f:
                f.write(text)
        except IOError as e:
            self._print_log(f"[Error] ファイルの保存に失敗しました: {e}")

    def _load_settings_from_file(self, fname: str) -> None:
        """設定ファイルを読み込んでGUIに反映

        Args:
            fname: 設定ファイルのパス
        """
        try:
            with open(fname, 'r', encoding='utf-8') as f:
                self.app_param = json.load(f)
        except (IOError, json.JSONDecodeError) as e:
            self._print_log(f"[Error] 設定ファイルの読み込みに失敗しました: {e}")
            return

        # 前回使用したファイルパスを保存
        self._save_last_config_path(fname)

        # 各lineEditに反映
        try:
            self.lineEdit_00.setText(fname)
            self.lineEdit_01.setText(self.app_param.get('max_linear_jerk', '5.0'))
            self.lineEdit_02.setText(self.app_param.get('max_linear_acceleration', '2.0'))
            self.lineEdit_03.setText(self.app_param.get('max_linear_speed', '2.0'))
            self.lineEdit_8.setText(self.app_param.get('max_angular_jerk', '100.0'))
            self.lineEdit_9.setText(self.app_param.get('max_angular_acceleration', '50.0'))
            self.lineEdit_10.setText(self.app_param.get('max_angular_speed', '50.0'))
            self.lineEdit_07.setText(self.app_param.get('hz', '100'))
            self.lineEdit_08.setText(self.app_param.get('precision', '0.01'))
            self.lineEdit_04.setText(self.app_param.get('disp_period', '0.5'))
        except KeyError as e:
            self._print_log(f"[Warning] 一部のパラメータが見つかりません: {e}")

        # ロボット形状の名前をlabel_19に表示
        if 'robot_shape' in self.app_param and self.app_param['robot_shape']:
            robot_name = self.app_param['robot_shape'].get('name', '未設定')
            self.label_19.setText(robot_name)
        else:
            self.label_19.setText('未設定')

        # tableに経由点データを反映
        if 'table' in self.app_param:
            items = self.app_param['table']
            self.tableWidget.clearContents()
            self.tableWidget.setRowCount(len(items))

            for r, item in enumerate(items):
                for c in range(len(item)):
                    value = item[c]
                    # 角度列(c=2)の場合、既存JSONとの互換性のため自動判定
                    if c == COLUMN_INDEX_ANGLE and value:
                        try:
                            value_num = float(value)
                            # -π〜πの範囲ならラジアンと判定して度に変換
                            # それ以外は既に度と判定してそのまま使用
                            if -np.pi <= value_num <= np.pi and abs(value_num) < 10:
                                # ラジアン値として度に変換
                                value = f"{np.rad2deg(value_num):.3f}"
                            else:
                                # 既に度の値としてそのまま使用
                                value = str(value)
                        except (ValueError, TypeError):
                            pass  # 変換失敗時はそのまま使用
                    self.tableWidget.setItem(r, c, QTableWidgetItem(str(value)))

        self._redraw()

    def _save_last_config_path(self, path: str) -> None:
        """前回使用した設定ファイルのパスを保存

        Args:
            path: 保存する設定ファイルのパス
        """
        try:
            with open(self.last_config_file, 'w', encoding='utf-8') as f:
                json.dump({'last_config_path': path}, f, ensure_ascii=False)
        except IOError:
            # 保存に失敗しても処理は継続
            pass

    def _load_last_settings(self) -> None:
        """起動時に前回使用した設定ファイルを読み込む"""
        # 前回の設定ファイルパスを取得
        if not os.path.exists(self.last_config_file):
            return

        try:
            with open(self.last_config_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
                last_path = data.get('last_config_path')

            if last_path and os.path.exists(last_path):
                self._load_settings_from_file(last_path)
                self._print_log(f"[Info] 前回の設定ファイルを読み込みました: {last_path}")
        except (IOError, json.JSONDecodeError, KeyError):
            # 読み込みに失敗しても処理は継続
            pass


def main() -> None:
    """メインエントリーポイント"""
    app = QApplication(sys.argv)
    gui = TrajectoryGeneratorGui()
    gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
