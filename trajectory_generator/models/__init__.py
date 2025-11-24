# -*- coding: utf-8 -*-
"""データモデルモジュール

経路計算用のデータクラスを提供します。
"""

from trajectory_generator.models.models import (
    TrajectoryConfig,
    ViaPoint,
    TrajectoryResult,
    ViaPointValidator
)
from trajectory_generator.models.robot_shape import (
    RobotShape,
    get_robot_shape
)

__all__ = [
    'TrajectoryConfig',
    'ViaPoint',
    'TrajectoryResult',
    'ViaPointValidator',
    'RobotShape',
    'get_robot_shape',
]
