# -*- coding: utf-8 -*-
"""ロボット形状の定義

ユーザーがJSONファイルからロボットのフットプリント形状を定義できるようにします。
"""

import json
from typing import List, Tuple, Optional, Union
import numpy as np
from dataclasses import dataclass


@dataclass
class RobotShape:
    """ロボットの形状を表すクラス

    Attributes:
        name: ロボット名
        vertices: 頂点座標のリスト [(x1, y1), (x2, y2), ...]
                 ロボット中心を原点とした座標系で定義
    """
    name: str
    vertices: List[Tuple[float, float]]

    def __post_init__(self):
        """初期化後の検証"""
        self._validate()

    def _validate(self) -> None:
        """形状データの妥当性を検証

        Raises:
            ValueError: 形状データが不正な場合
        """
        if not self.name:
            raise ValueError("ロボット名が空です")

        if len(self.vertices) < 3:
            raise ValueError(
                f"頂点は最低3つ必要です。現在の頂点数: {len(self.vertices)}"
            )

        for i, vertex in enumerate(self.vertices):
            if len(vertex) != 2:
                raise ValueError(
                    f"頂点{i}の座標が不正です。(x, y)の形式が必要: {vertex}"
                )

    def get_rotated_vertices(
        self,
        pose: Tuple[float, float, float]
    ) -> np.ndarray:
        """指定された姿勢でロボット形状を回転・移動

        Args:
            pose: ロボットの姿勢 (x, y, theta)
                  x, y: 位置 [m]
                  theta: 角度 [rad]

        Returns:
            回転・移動後の頂点座標配列 shape=(N, 2)
        """
        x, y, theta = pose
        rotated_vertices = np.empty((0, 2), float)

        for vertex in self.vertices:
            vx, vy = vertex
            # 回転行列を適用
            rx = x + (vx * np.cos(theta) - vy * np.sin(theta))
            ry = y + (vx * np.sin(theta) + vy * np.cos(theta))
            rotated_vertices = np.append(
                rotated_vertices,
                np.array([[rx, ry]]),
                axis=0
            )

        return rotated_vertices

    @classmethod
    def from_dict(cls, data: dict) -> 'RobotShape':
        """辞書からRobotShapeを生成

        Args:
            data: ロボット形状データ
                  {"name": "TR", "vertices": [[x1, y1], [x2, y2], ...]}

        Returns:
            RobotShapeインスタンス

        Raises:
            ValueError: データ形式が不正な場合
        """
        if 'name' not in data:
            raise ValueError("'name'フィールドが必要です")
        if 'vertices' not in data:
            raise ValueError("'vertices'フィールドが必要です")

        # vertices を List[Tuple[float, float]] に変換
        vertices = [tuple(v) for v in data['vertices']]

        return cls(name=data['name'], vertices=vertices)

    @classmethod
    def from_json_file(cls, filepath: str) -> 'RobotShape':
        """JSONファイルからRobotShapeを生成

        Args:
            filepath: JSONファイルのパス

        Returns:
            RobotShapeインスタンス

        Raises:
            FileNotFoundError: ファイルが見つからない場合
            json.JSONDecodeError: JSON形式が不正な場合
            ValueError: データ形式が不正な場合
        """
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)

        return cls.from_dict(data)

    def to_dict(self) -> dict:
        """辞書形式に変換

        Returns:
            ロボット形状データの辞書
        """
        return {
            'name': self.name,
            'vertices': [list(v) for v in self.vertices]
        }

    def to_json_file(self, filepath: str) -> None:
        """JSON形式でファイルに保存

        Args:
            filepath: 保存先ファイルパス
        """
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(self.to_dict(), f, indent=2, ensure_ascii=False)


def get_robot_shape(shape_dict: dict) -> RobotShape:
    """ロボット形状を取得

    辞書からロボット形状を取得します。

    Args:
        shape_dict: 形状定義の辞書 {"name": "...", "vertices": [...]}

    Returns:
        RobotShapeインスタンス

    Raises:
        ValueError: データ形式が不正な場合
    """
    return RobotShape.from_dict(shape_dict)
