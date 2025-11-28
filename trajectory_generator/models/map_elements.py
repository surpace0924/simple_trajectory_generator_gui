# -*- coding: utf-8 -*-
"""マップ要素のデータモデル

このモジュールは、キャンバスに描画するマップ要素のデータクラスを定義します。
"""
from dataclasses import dataclass, field
from typing import List, Optional, Union
import json


@dataclass
class MapElement:
    """マップ要素の基底クラス

    Attributes:
        type: 要素タイプ（rectangle, circle, polygon, line, text）
        label: ラベル名（オプション）
        color: 描画色（HTMLカラーコード）
        alpha: 透明度 [0.0-1.0]
        z_order: 描画順序（大きいほど前面）
    """
    type: str
    label: Optional[str] = None
    color: str = "#888888"
    alpha: float = 1.0
    z_order: int = 0


@dataclass
class Rectangle(MapElement):
    """矩形要素

    Attributes:
        position: 左下角の座標 [x, y] (m)
        width: 幅 (m)
        height: 高さ (m)
        filled: 塗りつぶすか
    """
    position: List[float] = field(default_factory=lambda: [0.0, 0.0])
    width: float = 1.0
    height: float = 1.0
    filled: bool = True

    def __post_init__(self):
        self.type = "rectangle"


@dataclass
class Circle(MapElement):
    """円要素

    Attributes:
        center: 中心座標 [x, y] (m)
        radius: 半径 (m)
        filled: 塗りつぶすか
    """
    center: List[float] = field(default_factory=lambda: [0.0, 0.0])
    radius: float = 1.0
    filled: bool = True

    def __post_init__(self):
        self.type = "circle"


@dataclass
class Polygon(MapElement):
    """多角形要素

    Attributes:
        vertices: 頂点座標のリスト [[x1, y1], [x2, y2], ...] (m)
        filled: 塗りつぶすか
    """
    vertices: List[List[float]] = field(default_factory=list)
    filled: bool = True

    def __post_init__(self):
        self.type = "polygon"


@dataclass
class Line(MapElement):
    """線要素

    Attributes:
        points: 点座標のリスト [[x1, y1], [x2, y2], ...] (m)
        width: 線幅 (m)
        style: 線スタイル（solid, dashed, dotted）
    """
    points: List[List[float]] = field(default_factory=list)
    width: float = 0.05
    style: str = "solid"

    def __post_init__(self):
        self.type = "line"


@dataclass
class Text(MapElement):
    """テキスト要素

    Attributes:
        position: テキスト位置 [x, y] (m)
        text: 表示テキスト
        font_size: フォントサイズ
        rotation: 回転角度 (degree)
    """
    position: List[float] = field(default_factory=lambda: [0.0, 0.0])
    text: str = ""
    font_size: int = 12
    rotation: float = 0.0

    def __post_init__(self):
        self.type = "text"


@dataclass
class MapData:
    """マップ全体のデータ

    Attributes:
        name: マップ名
        elements: マップ要素のリスト
        scale: スケール倍率
        origin: 原点座標 [x, y] (m)
        background_color: 背景色
    """
    name: str = "Untitled Map"
    elements: List[MapElement] = field(default_factory=list)
    scale: float = 1.0
    origin: List[float] = field(default_factory=lambda: [0.0, 0.0])
    background_color: Optional[str] = None

    @classmethod
    def from_dict(cls, data: dict) -> 'MapData':
        """辞書からMapDataを生成

        Args:
            data: マップデータの辞書

        Returns:
            MapDataインスタンス
        """
        name = data.get('name', 'Untitled Map')
        scale = data.get('scale', 1.0)
        origin = data.get('origin', [0.0, 0.0])
        background_color = data.get('background_color')

        elements = []
        for elem_data in data.get('elements', []):
            elem_type = elem_data.get('type')

            if elem_type == 'rectangle':
                rect = Rectangle(
                    type='rectangle',
                    position=elem_data.get('position', [0.0, 0.0]),
                    width=elem_data.get('width', 1.0),
                    height=elem_data.get('height', 1.0),
                    filled=elem_data.get('filled', True),
                    label=elem_data.get('label'),
                    color=elem_data.get('color', '#888888'),
                    alpha=elem_data.get('alpha', 1.0),
                    z_order=elem_data.get('z_order', 0)
                )
                elements.append(rect)
            elif elem_type == 'circle':
                circ = Circle(
                    type='circle',
                    center=elem_data.get('center', [0.0, 0.0]),
                    radius=elem_data.get('radius', 1.0),
                    filled=elem_data.get('filled', True),
                    label=elem_data.get('label'),
                    color=elem_data.get('color', '#888888'),
                    alpha=elem_data.get('alpha', 1.0),
                    z_order=elem_data.get('z_order', 0)
                )
                elements.append(circ)
            elif elem_type == 'polygon':
                poly = Polygon(
                    type='polygon',
                    vertices=elem_data.get('vertices', []),
                    filled=elem_data.get('filled', True),
                    label=elem_data.get('label'),
                    color=elem_data.get('color', '#888888'),
                    alpha=elem_data.get('alpha', 1.0),
                    z_order=elem_data.get('z_order', 0)
                )
                elements.append(poly)
            elif elem_type == 'line':
                line = Line(
                    type='line',
                    points=elem_data.get('points', []),
                    width=elem_data.get('width', 0.05),
                    style=elem_data.get('style', 'solid'),
                    label=elem_data.get('label'),
                    color=elem_data.get('color', '#000000'),
                    alpha=elem_data.get('alpha', 1.0),
                    z_order=elem_data.get('z_order', 0)
                )
                elements.append(line)
            elif elem_type == 'text':
                text = Text(
                    type='text',
                    position=elem_data.get('position', [0.0, 0.0]),
                    text=elem_data.get('text', ''),
                    font_size=elem_data.get('font_size', 12),
                    rotation=elem_data.get('rotation', 0.0),
                    label=elem_data.get('label'),
                    color=elem_data.get('color', '#000000'),
                    alpha=elem_data.get('alpha', 1.0),
                    z_order=elem_data.get('z_order', 0)
                )
                elements.append(text)

        return cls(
            name=name,
            elements=elements,
            scale=scale,
            origin=origin,
            background_color=background_color
        )

    @classmethod
    def from_json_file(cls, filepath: str) -> 'MapData':
        """JSONファイルからMapDataを読み込み

        Args:
            filepath: JSONファイルのパス

        Returns:
            MapDataインスタンス
        """
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)
        return cls.from_dict(data.get('map', {}))
