# -*- coding: utf-8 -*-
"""Trajectory Generator

ロボットの経路生成を行うPythonパッケージです。
"""

__version__ = '2.4.1'
__author__ = 'Ryoga Sato'

from trajectory_generator.core.trajectory_planner import TrajectoryPlanner
from trajectory_generator.models.models import TrajectoryConfig, ViaPoint, TrajectoryResult
from trajectory_generator.exceptions import (
    TrajectoryGenerationError,
    InvalidConfigurationError,
    InvalidViaPointError,
    CalculationError,
    ConstraintViolationError
)

__all__ = [
    'TrajectoryPlanner',
    'TrajectoryConfig',
    'ViaPoint',
    'TrajectoryResult',
    'TrajectoryGenerationError',
    'InvalidConfigurationError',
    'InvalidViaPointError',
    'CalculationError',
    'ConstraintViolationError',
]
