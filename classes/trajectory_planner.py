# -*- coding: utf-8 -*-
"""経路計画モジュール

このモジュールは、データクラスベースのクリーンなインターフェースで
経路計画を行います。
"""
from typing import List
import numpy as np

from classes.catmull_rom import CatmullRom
from classes.speed_profile import SpeedProfile
from classes.min_jerk_profile import MinJerkProfile
from classes.models import TrajectoryConfig, ViaPoint, TrajectoryResult, ViaPointValidator
from classes.exceptions import InvalidViaPointError, CalculationError


class TrajectoryPlanner:
    """経路計画クラス

    Attributes:
        config: 経路計画の設定パラメータ
    """

    def __init__(self, config: TrajectoryConfig) -> None:
        """コンストラクタ

        Args:
            config: 経路計画の設定パラメータ

        Raises:
            ValueError: 設定が不正な場合
        """
        config.validate()
        self.config = config

    def plan(self, via_points: List[ViaPoint]) -> TrajectoryResult:
        """経路を計画する

        Args:
            via_points: 経由点のリスト

        Returns:
            計算された経路結果

        Raises:
            InvalidViaPointError: 経由点が不正な場合
            CalculationError: 計算中にエラーが発生した場合
        """
        try:
            # 入力検証
            ViaPointValidator.validate_via_points(via_points)

            # 経路補間曲線の計算
            curve, curve_length_list, via_points_length = self._calculate_interpolation(
                via_points
            )

            # 速度プロファイルの計算
            timestamps, velocities, accelerations, dot_profile = (
                self._calculate_speed_profile(via_points, via_points_length)
            )

            # 経路点の抽出
            trajectory, trajectory_length_list = self._pickup_trajectory(
                curve, curve_length_list, dot_profile
            )

            # 角度プロファイルの計算
            angles, angular_velocities = self._calculate_angle_profile(
                via_points, trajectory, timestamps
            )

            # 角度を経路に統合
            trajectory_with_angles = np.hstack([
                trajectory,
                angles.reshape(len(angles), 1)
            ])

            # 曲率の計算
            curvatures = self._calculate_curvature(trajectory_with_angles)

            # 結果を構造化
            return TrajectoryResult(
                positions=np.array(trajectory_with_angles),
                velocities=np.array(velocities),
                accelerations=np.array(accelerations),
                angular_velocities=angular_velocities,
                timestamps=np.array(timestamps),
                curvatures=np.array(curvatures),
                trajectory_length_list=trajectory_length_list
            )

        except ValueError as e:
            raise InvalidViaPointError(f"経由点が不正です: {e}") from e
        except ZeroDivisionError as e:
            raise CalculationError(f"数値計算エラー: {e}") from e
        except Exception as e:
            raise CalculationError(f"予期しないエラー: {e}") from e

    def _calculate_interpolation(
        self,
        via_points: List[ViaPoint]
    ) -> tuple:
        """Catmull-Rom曲線による経路補間

        Args:
            via_points: 経由点のリスト

        Returns:
            (curve, curve_length_list, via_points_length)のタプル
        """
        # 旧形式への変換
        legacy_points = [vp.to_legacy_format() for vp in via_points]

        # 概算経路長の計算
        approximate_length = 0.0
        for i in range(len(via_points) - 1):
            p1 = via_points[i].position
            p2 = via_points[i + 1].position
            approximate_length += np.linalg.norm(p2 - p1)

        # 分割数の決定（経路打点数の約10倍）
        divisions = int(10 * approximate_length / self.config.precision)

        # Catmull-Rom補間
        interpolator = CatmullRom()
        interpolator.set_control_points(legacy_points)
        return interpolator.calculate(divisions)

    def _calculate_speed_profile(
        self,
        via_points: List[ViaPoint],
        via_points_length: List[float]
    ) -> tuple:
        """速度プロファイルの計算

        Args:
            via_points: 経由点のリスト
            via_points_length: 各経由点での累積距離

        Returns:
            (timestamps, velocities, accelerations, dot_profile)のタプル
        """
        # 速度制約のある区間を抽出
        section_lengths = []
        target_speeds = []
        prev_idx = 0

        for i, via_point in enumerate(via_points):
            if via_point.use_speed_constraint and i != 0:
                section_length = via_points_length[i] - via_points_length[prev_idx]
                section_lengths.append(section_length)
                target_speeds.append(via_point.speed)
                prev_idx = i

        # プロファイル計算
        timestamps = []
        velocities = []
        accelerations = []
        dot_profile = []

        sp = SpeedProfile()

        for i, (section_length, target_speed) in enumerate(
            zip(section_lengths, target_speeds)
        ):
            t0 = sp.t_end()
            sp.reset(
                j_max=self.config.max_linear_jerk,
                a_max=self.config.max_linear_acceleration,
                v_sat=self.config.max_linear_speed,
                v_start=sp.v_end(),
                v_target=target_speed,
                dist=section_length,
                x_start=sp.x_end(),
                t_start=sp.t_end()
            )

            # サンプリング
            num_samples = int((sp.t_end() - t0) * self.config.control_frequency)
            for j in range(num_samples):
                t = j / self.config.control_frequency + t0
                timestamps.append(t)
                dot_profile.append(sp.x(t))
                accelerations.append(sp.a(t))
                velocities.append(sp.v(t))

        # 終点を追加
        end_time = sp.t_end()
        timestamps.append(end_time)
        dot_profile.append(sp.x(end_time))
        accelerations.append(sp.a(end_time))
        velocities.append(sp.v(end_time))

        return timestamps, velocities, accelerations, dot_profile

    def _pickup_trajectory(
        self,
        curve: List,
        length_list: List[float],
        dot_profile: List[float]
    ) -> tuple:
        """速度プロファイルに従って経路点を抽出

        Args:
            curve: 補間曲線の座標点
            length_list: 各点の累積距離
            dot_profile: 距離プロファイル

        Returns:
            (trajectory, trajectory_length_list)のタプル
        """
        trajectory = []
        trajectory_length_list = []

        for distance in dot_profile:
            # 最も近い点を検索
            idx = np.abs(np.asarray(length_list) - distance).argmin()
            trajectory.append(curve[idx])
            trajectory_length_list.append(length_list[idx])

        return trajectory, trajectory_length_list

    def _calculate_angle_profile(
        self,
        via_points: List[ViaPoint],
        trajectory: List,
        timestamps: List[float]
    ) -> tuple:
        """角度プロファイルの計算

        Args:
            via_points: 経由点のリスト
            trajectory: 経路座標点
            timestamps: タイムスタンプ

        Returns:
            (angles, angular_velocities)のタプル
        """
        # 角度制約のある経由点の時刻とその角度
        via_time_stamps = [0.0]
        via_angles = [via_points[0].angle if via_points[0].angle is not None else 0.0]

        # 中間の角度制約を収集
        for i, via_point in enumerate(via_points):
            if i in (0, len(via_points) - 1):
                continue
            if not via_point.use_angle_constraint:
                continue

            # この経由点に最も近い軌跡点の時刻を検索
            time = self._find_nearest_time(via_point, trajectory, timestamps)
            via_time_stamps.append(time)
            via_angles.append(via_point.angle)

        # 終点を追加
        end_time = timestamps[-1]
        via_time_stamps.append(end_time)
        last_angle = via_points[-1].angle if via_points[-1].angle is not None else 0.0
        via_angles.append(last_angle)

        # 各タイムスタンプで角度を計算
        angles = np.empty(0)
        angular_velocities = np.empty(0)

        for t in timestamps:
            # どの区間に属するか特定
            # via_time_stamps[idx-1] <= t < via_time_stamps[idx] となるidxを見つける
            idx = 1
            for i in range(1, len(via_time_stamps)):
                if t < via_time_stamps[i]:
                    idx = i
                    break
            else:
                # ループが完了した場合は最後の区間
                idx = len(via_time_stamps) - 1

            # MinJerkProfileで角度を補間
            dt = via_time_stamps[idx] - via_time_stamps[idx - 1]
            if dt > 0:
                mjp = MinJerkProfile(
                    x_start=via_angles[idx - 1],
                    x_target=via_angles[idx],
                    time=dt
                )
                angles = np.append(angles, mjp.x(t - via_time_stamps[idx - 1]))
                angular_velocities = np.append(
                    angular_velocities,
                    mjp.v(t - via_time_stamps[idx - 1])
                )
            else:
                angles = np.append(angles, via_angles[idx])
                angular_velocities = np.append(angular_velocities, 0.0)

        return angles, angular_velocities

    def _find_nearest_time(
        self,
        via_point: ViaPoint,
        trajectory: List,
        timestamps: List[float]
    ) -> float:
        """経由点に最も近い軌跡点の時刻を検索

        Args:
            via_point: 経由点
            trajectory: 軌跡座標点
            timestamps: タイムスタンプ

        Returns:
            最も近い点の時刻
        """
        min_dist = float('inf')
        nearest_idx = 0

        for i, point in enumerate(trajectory):
            dist = np.linalg.norm(point - via_point.position)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i

        return timestamps[nearest_idx]

    def _calculate_curvature(self, trajectory: np.ndarray) -> List[float]:
        """経路の曲率を計算

        Args:
            trajectory: 経路座標点（x, y, angleを含む）

        Returns:
            各点での曲率のリスト
        """
        curvatures = [0.0]

        for i in range(1, len(trajectory) - 1):
            # 前後の点との差分
            dxn = trajectory[i][0] - trajectory[i - 1][0]
            dxp = trajectory[i + 1][0] - trajectory[i][0]
            dyn = trajectory[i][1] - trajectory[i - 1][1]
            dyp = trajectory[i + 1][1] - trajectory[i][1]

            dn = np.hypot(dxn, dyn)
            dp = np.hypot(dxp, dyp)

            # ゼロ除算対策
            if (dn + dp) < 1e-10 or dn < 1e-10 or dp < 1e-10:
                curvatures.append(0.0)
                continue

            # 曲率計算
            dx = (dp / dn * dxn + dn / dp * dxp) / (dn + dp)
            ddx = 2.0 * (dxp / dp - dxn / dn) / (dn + dp)
            dy = (dp / dn * dyn + dn / dp * dyp) / (dn + dp)
            ddy = 2.0 * (dyp / dp - dyn / dn) / (dn + dp)

            curvature = (ddy * dx - ddx * dy) / ((dx**2 + dy**2)**1.5)
            curvatures.append(abs(curvature))

        # 終点の曲率
        curvatures.append(abs(curvatures[-1]))

        return curvatures
