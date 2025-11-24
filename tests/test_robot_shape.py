# -*- coding: utf-8 -*-
"""ロボット形状クラスのテスト"""

import sys
import os
import json
import tempfile
import numpy as np

# プロジェクトルートをパスに追加
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from trajectory_generator.models.robot_shape import (
    RobotShape,
    get_robot_shape
)


def test_robot_shape_creation():
    """RobotShapeの基本的な生成テスト"""
    print('Test: RobotShape基本生成')

    # 正常な形状の生成
    robot = RobotShape(
        name='TestRobot',
        vertices=[
            (1.0, 0.0),
            (0.0, 1.0),
            (-1.0, 0.0),
            (0.0, -1.0)
        ]
    )

    assert robot.name == 'TestRobot'
    assert len(robot.vertices) == 4
    print('  [OK] 基本的な生成成功')


def test_robot_shape_validation():
    """RobotShapeのバリデーションテスト"""
    print('Test: RobotShapeバリデーション')

    # 頂点が2つしかない不正な形状
    try:
        robot = RobotShape(
            name='InvalidRobot',
            vertices=[(0.0, 0.0), (1.0, 1.0)]
        )
        assert False, "頂点数不足のエラーが発生すべき"
    except ValueError as e:
        assert "頂点は最低3つ必要" in str(e)
        print('  [OK] 頂点数不足の検出')

    # 名前が空の不正な形状
    try:
        robot = RobotShape(
            name='',
            vertices=[(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]
        )
        assert False, "空の名前エラーが発生すべき"
    except ValueError as e:
        assert "ロボット名が空" in str(e)
        print('  [OK] 空の名前の検出')


def test_rotated_vertices():
    """回転・移動変換のテスト"""
    print('Test: 頂点の回転・移動')

    # 正方形のロボット
    robot = RobotShape(
        name='Square',
        vertices=[
            (0.5, 0.5),
            (-0.5, 0.5),
            (-0.5, -0.5),
            (0.5, -0.5)
        ]
    )

    # 原点、角度0での変換（変換なし）
    pose1 = (0.0, 0.0, 0.0)
    vertices1 = robot.get_rotated_vertices(pose1)
    assert vertices1.shape == (4, 2)
    assert np.allclose(vertices1[0], [0.5, 0.5], atol=1e-10)
    print('  [OK] 無変換')

    # 平行移動のみ
    pose2 = (1.0, 2.0, 0.0)
    vertices2 = robot.get_rotated_vertices(pose2)
    assert np.allclose(vertices2[0], [1.5, 2.5], atol=1e-10)
    print('  [OK] 平行移動')

    # 90度回転
    pose3 = (0.0, 0.0, np.pi / 2)
    vertices3 = robot.get_rotated_vertices(pose3)
    assert np.allclose(vertices3[0], [-0.5, 0.5], atol=1e-10)
    print('  [OK] 90度回転')

    # 平行移動 + 回転
    pose4 = (1.0, 1.0, np.pi / 4)
    vertices4 = robot.get_rotated_vertices(pose4)
    expected_x = 1.0 + 0.5 * np.cos(np.pi/4) - 0.5 * np.sin(np.pi/4)
    expected_y = 1.0 + 0.5 * np.sin(np.pi/4) + 0.5 * np.cos(np.pi/4)
    assert np.allclose(vertices4[0], [expected_x, expected_y], atol=1e-10)
    print('  [OK] 平行移動+回転')


def test_json_io():
    """JSON入出力のテスト"""
    print('Test: JSON入出力')

    robot = RobotShape(
        name='TestRobot',
        vertices=[
            (1.0, 0.0),
            (0.0, 1.0),
            (-1.0, 0.0)
        ]
    )

    # 一時ファイルに保存
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        temp_file = f.name
        robot.to_json_file(temp_file)

    try:
        # 読み込み
        loaded_robot = RobotShape.from_json_file(temp_file)
        assert loaded_robot.name == 'TestRobot'
        assert len(loaded_robot.vertices) == 3
        assert loaded_robot.vertices[0] == (1.0, 0.0)
        print('  [OK] JSONファイルの保存と読み込み')
    finally:
        # 一時ファイル削除
        os.remove(temp_file)


def test_from_dict():
    """辞書からの生成テスト"""
    print('Test: 辞書からの生成')

    data = {
        'name': 'DictRobot',
        'vertices': [
            [1.0, 0.0],
            [0.0, 1.0],
            [-1.0, 0.0]
        ]
    }

    robot = RobotShape.from_dict(data)
    assert robot.name == 'DictRobot'
    assert len(robot.vertices) == 3
    assert robot.vertices[0] == (1.0, 0.0)
    print('  [OK] 辞書からの生成')

    # 必須フィールド不足
    try:
        invalid_data = {'name': 'NoVertices'}
        RobotShape.from_dict(invalid_data)
        assert False, "vertices不足のエラーが発生すべき"
    except ValueError as e:
        assert 'vertices' in str(e)
        print('  [OK] 必須フィールドの検証')


def test_get_robot_shape():
    """get_robot_shape関数のテスト"""
    print('Test: get_robot_shape関数')

    # 辞書から直接取得
    data_dict = {
        'name': 'DictRobot',
        'vertices': [[1.0, 0.0], [0.0, 1.0], [-1.0, 0.0]]
    }
    robot = get_robot_shape(data_dict)
    assert robot.name == 'DictRobot'
    assert len(robot.vertices) == 3
    print('  [OK] 辞書から取得')

    # 不正なデータ型（文字列は辞書ではない）
    try:
        get_robot_shape("invalid_string")
        assert False, "文字列が渡された場合はエラーが発生すべき"
    except (ValueError, AttributeError):
        print('  [OK] 不正な型の検出')


def run_all_tests():
    """すべてのテストを実行"""
    print('=' * 60)
    print('RobotShape テストスイート')
    print('=' * 60)
    print()

    tests = [
        test_robot_shape_creation,
        test_robot_shape_validation,
        test_rotated_vertices,
        test_json_io,
        test_from_dict,
        test_get_robot_shape,
    ]

    passed = 0
    failed = 0

    for test in tests:
        try:
            test()
            passed += 1
            print()
        except AssertionError as e:
            print(f'  ✗ 失敗: {e}')
            failed += 1
            print()

    print('=' * 60)
    print(f'結果: {passed}件成功, {failed}件失敗')
    print('=' * 60)

    return failed == 0


if __name__ == '__main__':
    success = run_all_tests()
    sys.exit(0 if success else 1)
