# -*- coding: utf-8 -*-
"""経路計算用のデータモデル定義

このモジュールは、経路計算に使用するデータクラスを定義します。
"""
from dataclasses import dataclass, field
from typing import List, Optional
import numpy as np
import numpy.typing as npt


@dataclass
class TrajectoryConfig:
    """経路生成の設定パラメータ

    経路計算に必要な制約条件を保持します。

    Attributes:
        max_linear_jerk: 最大並進躍度 [m/s³]
        max_linear_acceleration: 最大並進加速度 [m/s²]
        max_linear_speed: 最大並進速度 [m/s]
        max_angular_jerk: 最大回転躍度 [rad/s³]
        max_angular_acceleration: 最大回転加速度 [rad/s²]
        max_angular_speed: 最大回転速度 [rad/s]
        control_frequency: 制御周期 [Hz]
        precision: 経路打点間隔 [m]
    """
    max_linear_jerk: float = 5.0
    max_linear_acceleration: float = 1.0
    max_linear_speed: float = 0.5
    max_angular_jerk: float = 5.0
    max_angular_acceleration: float = 2.0
    max_angular_speed: float = 3.0
    control_frequency: int = 100
    precision: float = 0.01

    def __post_init__(self) -> None:
        """初期化後の検証"""
        self.validate()

    def validate(self) -> None:
        """設定値の妥当性を検証

        Raises:
            ValueError: 設定値が不正な場合
        """
        if self.max_linear_jerk <= 0:
            raise ValueError(f"最大並進躍度は正の値である必要があります: {self.max_linear_jerk}")
        if self.max_linear_acceleration <= 0:
            raise ValueError(f"最大並進加速度は正の値である必要があります: {self.max_linear_acceleration}")
        if self.max_linear_speed <= 0:
            raise ValueError(f"最大並進速度は正の値である必要があります: {self.max_linear_speed}")
        if self.max_angular_jerk <= 0:
            raise ValueError(f"最大回転躍度は正の値である必要があります: {self.max_angular_jerk}")
        if self.max_angular_acceleration <= 0:
            raise ValueError(f"最大回転加速度は正の値である必要があります: {self.max_angular_acceleration}")
        if self.max_angular_speed <= 0:
            raise ValueError(f"最大回転速度は正の値である必要があります: {self.max_angular_speed}")
        if self.control_frequency <= 0:
            raise ValueError(f"制御周期は正の値である必要があります: {self.control_frequency}")
        if self.precision <= 0:
            raise ValueError(f"打点間隔は正の値である必要があります: {self.precision}")


@dataclass
class ViaPoint:
    """経由点の定義

    経路上の経由点とその制約条件を表現します。

    Attributes:
        position: 位置座標 [x, y] (m)
        angle: 角度 [rad] (Noneの場合は角度制約なし)
        speed: 速度 [m/s] (Noneの場合は速度制約なし)
        use_angle_constraint: 角度制約を使用するか
        use_speed_constraint: 速度制約を使用するか
    """
    position: List[float]
    angle: Optional[float] = None
    speed: Optional[float] = None
    use_angle_constraint: bool = False
    use_speed_constraint: bool = False

    def __post_init__(self) -> None:
        """初期化後の処理"""
        # positionをNumPy配列に変換
        if not isinstance(self.position, np.ndarray):
            self.position = np.array(self.position[:2])  # x, yのみ使用

        # 制約フラグの自動設定
        if self.angle is not None and not self.use_angle_constraint:
            self.use_angle_constraint = True
        if self.speed is not None and not self.use_speed_constraint:
            self.use_speed_constraint = True

        self.validate()

    def validate(self) -> None:
        """経由点の妥当性を検証

        Raises:
            ValueError: 経由点の値が不正な場合
        """
        if len(self.position) < 2:
            raise ValueError(f"位置座標は最低2次元必要です: {self.position}")

        if self.angle is not None and not (-np.pi <= self.angle <= np.pi):
            raise ValueError(
                f"角度は-π〜πラジアン(-180°〜180°)の範囲である必要があります: "
                f"{self.angle} rad ({np.rad2deg(self.angle):.1f}°)"
            )

        if self.speed is not None and self.speed < 0:
            raise ValueError(f"速度は非負の値である必要があります: {self.speed}")

    def to_legacy_format(self) -> List[float]:
        """旧形式への変換（後方互換性用）

        Returns:
            [x, y, angle, speed]形式のリスト
        """
        return [
            float(self.position[0]),
            float(self.position[1]),
            self.angle if self.angle is not None else 0.0,
            self.speed if self.speed is not None else 0.0
        ]

    @classmethod
    def from_legacy_format(cls, data: List[float],
                          use_angle: bool = False,
                          use_speed: bool = False) -> 'ViaPoint':
        """旧形式からの変換（後方互換性用）

        Args:
            data: [x, y, angle, speed]形式のリスト
            use_angle: 角度制約を使用するか
            use_speed: 速度制約を使用するか

        Returns:
            ViaPointインスタンス
        """
        return cls(
            position=[data[0], data[1]],
            angle=data[2] if len(data) > 2 else None,
            speed=data[3] if len(data) > 3 else None,
            use_angle_constraint=use_angle,
            use_speed_constraint=use_speed
        )


@dataclass
class TrajectoryResult:
    """経路計算結果

    計算された経路とその詳細情報を保持します。

    Attributes:
        positions: 経路上の座標点 [N x 3] (x, y, angle)
        velocities: 各点での速度 [m/s]
        accelerations: 各点での加速度 [m/s²]
        angular_velocities: 各点での角速度 [rad/s]
        timestamps: 各点でのタイムスタンプ [s]
        curvatures: 各点での曲率 [1/m]
        trajectory_length_list: 各点の始点からの累積距離 [m]
    """
    positions: npt.NDArray[np.float64] = field(default_factory=lambda: np.array([]))
    velocities: npt.NDArray[np.float64] = field(default_factory=lambda: np.array([]))
    accelerations: npt.NDArray[np.float64] = field(default_factory=lambda: np.array([]))
    angular_velocities: npt.NDArray[np.float64] = field(default_factory=lambda: np.array([]))
    timestamps: npt.NDArray[np.float64] = field(default_factory=lambda: np.array([]))
    curvatures: npt.NDArray[np.float64] = field(default_factory=lambda: np.array([]))
    trajectory_length_list: List[float] = field(default_factory=list)

    @property
    def total_length(self) -> float:
        """総経路長を取得 [m]"""
        return self.trajectory_length_list[-1] if self.trajectory_length_list else 0.0

    @property
    def total_time(self) -> float:
        """総所要時間を取得 [s]"""
        return self.timestamps[-1] if len(self.timestamps) > 0 else 0.0

    @property
    def num_points(self) -> int:
        """経路点数を取得"""
        return len(self.positions)

    def get_state_at_index(self, index: int) -> dict:
        """指定インデックスの状態を取得

        Args:
            index: インデックス

        Returns:
            状態を表す辞書
        """
        if not (0 <= index < self.num_points):
            raise IndexError(f"インデックスが範囲外です: {index}")

        return {
            'position': self.positions[index],
            'velocity': self.velocities[index],
            'acceleration': self.accelerations[index],
            'angular_velocity': self.angular_velocities[index],
            'timestamp': self.timestamps[index],
            'curvature': self.curvatures[index],
            'distance': self.trajectory_length_list[index]
        }


class ViaPointValidator:
    """経由点リストの検証クラス"""

    @staticmethod
    def validate_via_points(via_points: List[ViaPoint]) -> None:
        """経由点リストの妥当性を検証

        Args:
            via_points: 経由点のリスト

        Raises:
            ValueError: 経由点リストが不正な場合
        """
        if len(via_points) < 3:
            raise ValueError(f"経由点は最低3点必要です（Catmull-Rom補間の制約）。現在: {len(via_points)}点")

        # 各経由点の検証
        for i, point in enumerate(via_points):
            try:
                point.validate()
            except ValueError as e:
                raise ValueError(f"経由点{i}が不正です: {e}")

        # 連続する経由点間の距離チェック
        for i in range(len(via_points) - 1):
            p1 = via_points[i].position
            p2 = via_points[i + 1].position
            distance = np.linalg.norm(p2 - p1)

            if distance < 1e-6:  # ほぼ同じ位置
                raise ValueError(
                    f"経由点{i}と{i+1}が近すぎます。距離: {distance:.6f}m"
                )
