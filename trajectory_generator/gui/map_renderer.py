# -*- coding: utf-8 -*-
"""マップレンダラーモジュール

このモジュールは、マップ要素をmatplotlibのaxisに描画する機能を提供します。
"""
from typing import Optional
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.axes import Axes

from trajectory_generator.models.map_elements import (
    MapData,
    Rectangle,
    Circle,
    Polygon,
    Line,
    Text
)


class MapRenderer:
    """マップ要素を描画するクラス

    matplotlibのaxisにマップ要素を描画します。
    """

    def __init__(self):
        """コンストラクタ"""
        self.line_style_map = {
            'solid': '-',
            'dashed': '--',
            'dotted': ':',
            'dashdot': '-.'
        }

    def get_map_bounds(self, map_data: MapData) -> tuple:
        """マップの境界を計算

        Args:
            map_data: マップデータ

        Returns:
            (x_min, x_max, y_min, y_max) のタプル
        """
        if map_data is None or not map_data.elements:
            return None

        x_coords = []
        y_coords = []

        for element in map_data.elements:
            if isinstance(element, Rectangle):
                # スケールと原点オフセットを適用
                pos = self._apply_scale_and_origin(element.position, map_data)
                width = element.width * map_data.scale
                height = element.height * map_data.scale
                x_coords.extend([pos[0], pos[0] + width])
                y_coords.extend([pos[1], pos[1] + height])

            elif isinstance(element, Circle):
                center = self._apply_scale_and_origin(element.center, map_data)
                radius = element.radius * map_data.scale
                x_coords.extend([center[0] - radius, center[0] + radius])
                y_coords.extend([center[1] - radius, center[1] + radius])

            elif isinstance(element, Polygon):
                if element.vertices:
                    vertices = self._apply_scale_and_origin(element.vertices, map_data)
                    for v in vertices:
                        x_coords.append(v[0])
                        y_coords.append(v[1])

            elif isinstance(element, Line):
                if len(element.points) >= 2:
                    points = self._apply_scale_and_origin(element.points, map_data)
                    for p in points:
                        x_coords.append(p[0])
                        y_coords.append(p[1])

            elif isinstance(element, Text):
                pos = self._apply_scale_and_origin(element.position, map_data)
                x_coords.append(pos[0])
                y_coords.append(pos[1])

        if not x_coords or not y_coords:
            return None

        # 余白を追加（10%）
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)
        x_margin = (x_max - x_min) * 0.1
        y_margin = (y_max - y_min) * 0.1

        return (x_min - x_margin, x_max + x_margin, y_min - y_margin, y_max + y_margin)

    def render(self, ax: Axes, map_data: MapData) -> None:
        """マップをaxisに描画

        Args:
            ax: matplotlib axis
            map_data: マップデータ
        """
        if map_data is None:
            return

        # 背景色を設定
        if map_data.background_color:
            ax.set_facecolor(map_data.background_color)

        # マップの境界を取得して軸範囲を設定
        bounds = self.get_map_bounds(map_data)
        if bounds:
            x_min, x_max, y_min, y_max = bounds
            ax.set_xlim(x_min, x_max)
            ax.set_ylim(y_min, y_max)
            ax.set_aspect('equal')

        # 軸を消す
        ax.set_xticks([])
        ax.set_yticks([])

        # z_orderでソート（小さい順 = 奥から描画）
        sorted_elements = sorted(map_data.elements, key=lambda e: e.z_order)

        # 各要素を描画
        for element in sorted_elements:
            if isinstance(element, Rectangle):
                self._draw_rectangle(ax, element, map_data)
            elif isinstance(element, Circle):
                self._draw_circle(ax, element, map_data)
            elif isinstance(element, Polygon):
                self._draw_polygon(ax, element, map_data)
            elif isinstance(element, Line):
                self._draw_line(ax, element, map_data)
            elif isinstance(element, Text):
                self._draw_text(ax, element, map_data)

    def _apply_scale_and_origin(self, coords, map_data: MapData):
        """座標にスケールと原点オフセットを適用

        Args:
            coords: 座標（単一座標またはリスト）
            map_data: マップデータ

        Returns:
            変換後の座標
        """
        if isinstance(coords[0], (list, tuple)):
            # リストの場合
            return [
                [
                    c[0] * map_data.scale + map_data.origin[0],
                    c[1] * map_data.scale + map_data.origin[1]
                ]
                for c in coords
            ]
        else:
            # 単一座標の場合
            return [
                coords[0] * map_data.scale + map_data.origin[0],
                coords[1] * map_data.scale + map_data.origin[1]
            ]

    def _draw_rectangle(self, ax: Axes, rect: Rectangle, map_data: MapData) -> None:
        """矩形を描画

        Args:
            ax: matplotlib axis
            rect: 矩形要素
            map_data: マップデータ
        """
        # スケールと原点オフセットを適用
        pos = self._apply_scale_and_origin(rect.position, map_data)
        width = rect.width * map_data.scale
        height = rect.height * map_data.scale

        rectangle = patches.Rectangle(
            pos,
            width,
            height,
            linewidth=1,
            edgecolor=rect.color,
            facecolor=rect.color if rect.filled else 'none',
            alpha=rect.alpha,
            zorder=rect.z_order
        )
        ax.add_patch(rectangle)

    def _draw_circle(self, ax: Axes, circ: Circle, map_data: MapData) -> None:
        """円を描画

        Args:
            ax: matplotlib axis
            circ: 円要素
            map_data: マップデータ
        """
        # スケールと原点オフセットを適用
        center = self._apply_scale_and_origin(circ.center, map_data)
        radius = circ.radius * map_data.scale

        circle = patches.Circle(
            center,
            radius,
            linewidth=1,
            edgecolor=circ.color,
            facecolor=circ.color if circ.filled else 'none',
            alpha=circ.alpha,
            zorder=circ.z_order
        )
        ax.add_patch(circle)

    def _draw_polygon(self, ax: Axes, poly: Polygon, map_data: MapData) -> None:
        """多角形を描画

        Args:
            ax: matplotlib axis
            poly: 多角形要素
            map_data: マップデータ
        """
        if not poly.vertices:
            return

        # スケールと原点オフセットを適用
        vertices = self._apply_scale_and_origin(poly.vertices, map_data)

        polygon = patches.Polygon(
            vertices,
            linewidth=1,
            edgecolor=poly.color,
            facecolor=poly.color if poly.filled else 'none',
            alpha=poly.alpha,
            zorder=poly.z_order
        )
        ax.add_patch(polygon)

    def _draw_line(self, ax: Axes, line: Line, map_data: MapData) -> None:
        """線を描画

        Args:
            ax: matplotlib axis
            line: 線要素
            map_data: マップデータ
        """
        if len(line.points) < 2:
            return

        # スケールと原点オフセットを適用
        points = self._apply_scale_and_origin(line.points, map_data)

        x_coords = [p[0] for p in points]
        y_coords = [p[1] for p in points]

        linestyle = self.line_style_map.get(line.style, '-')

        ax.plot(
            x_coords,
            y_coords,
            color=line.color,
            linewidth=line.width * map_data.scale * 10,  # 線幅をスケーリング
            linestyle=linestyle,
            alpha=line.alpha,
            zorder=line.z_order
        )

    def _draw_text(self, ax: Axes, text_elem: Text, map_data: MapData) -> None:
        """テキストを描画

        Args:
            ax: matplotlib axis
            text_elem: テキスト要素
            map_data: マップデータ
        """
        if not text_elem.text:
            return

        # スケールと原点オフセットを適用
        pos = self._apply_scale_and_origin(text_elem.position, map_data)

        ax.text(
            pos[0],
            pos[1],
            text_elem.text,
            fontsize=text_elem.font_size,
            color=text_elem.color,
            rotation=text_elem.rotation,
            alpha=text_elem.alpha,
            zorder=text_elem.z_order,
            ha='center',
            va='center'
        )
