# -*- coding: utf-8 -*-
"""経路計算用の例外クラス定義

このモジュールは、経路計算で発生する可能性のある
カスタム例外クラスを定義します。
"""


class TrajectoryGenerationError(Exception):
    """経路生成エラーの基底クラス"""
    pass


class InvalidConfigurationError(TrajectoryGenerationError):
    """設定エラー

    設定パラメータが不正な場合に発生します。
    """
    pass


class InvalidViaPointError(TrajectoryGenerationError):
    """経由点エラー

    経由点が不正な場合に発生します。
    """
    pass


class CalculationError(TrajectoryGenerationError):
    """計算エラー

    経路計算中にエラーが発生した場合に発生します。
    """
    pass


class ConstraintViolationError(TrajectoryGenerationError):
    """制約違反エラー

    制約条件を満たせない場合に発生します。
    """
    pass
