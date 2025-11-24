# -*- coding: utf-8 -*-
"""TrajectoryPlannerのテスト

使用方法:
    python3 -m tests.test_trajectory_planner
"""
import numpy as np
from trajectory_generator.exceptions import InvalidViaPointError
from trajectory_generator.models import TrajectoryConfig, ViaPoint
from trajectory_generator.core.trajectory_planner import TrajectoryPlanner
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def test_basic_planning():
    """基本的な経路計画のテスト"""
    print('Test: 基本的な経路計画')

    config = TrajectoryConfig(
        max_linear_jerk=5.0,
        max_linear_acceleration=1.0,
        max_linear_speed=0.5
    )

    via_points = [
        ViaPoint([0.0, 0.0], angle=0.0, speed=0.0),
        ViaPoint([1.0, 1.0], speed=0.3, use_speed_constraint=True),
        ViaPoint([2.0, 0.0], angle=0.0, speed=0.0)
    ]

    planner = TrajectoryPlanner(config)
    result = planner.plan(via_points)

    assert result.num_points > 0, "経路点が生成されていません"
    assert result.total_length > 0, "経路長が0です"
    assert result.total_time > 0, "所要時間が0です"
    assert len(result.velocities) == result.num_points, "速度データの長さが不一致"
    assert len(result.accelerations) == result.num_points, "加速度データの長さが不一致"

    print(f'  [OK] {result.num_points}点生成, 経路長{result.total_length:.3f}m, 時間{result.total_time:.3f}s')


def test_straight_line():
    """直線経路のテスト"""
    print('Test: 直線経路')

    config = TrajectoryConfig(
        max_linear_jerk=10.0,
        max_linear_acceleration=2.0,
        max_linear_speed=1.0
    )

    via_points = [
        ViaPoint([0.0, 0.0], speed=0.0),
        ViaPoint([2.5, 0.0], speed=0.5),
        ViaPoint([5.0, 0.0], speed=0.0)
    ]

    planner = TrajectoryPlanner(config)
    result = planner.plan(via_points)

    # 直線なので、y座標はほぼ0のはず
    max_y_deviation = max(abs(p[1]) for p in result.positions)
    assert max_y_deviation < 0.1, f"直線からの逸脱が大きい: {max_y_deviation}"

    print(f'  [OK] 直線経路生成成功, y方向最大偏差{max_y_deviation:.6f}m')


def test_constraint_satisfaction():
    """制約条件の満足度テスト"""
    print('Test: 制約条件の満足度')

    config = TrajectoryConfig(
        max_linear_jerk=5.0,
        max_linear_acceleration=1.0,
        max_linear_speed=0.5
    )

    via_points = [
        ViaPoint([0.0, 0.0], speed=0.0),
        ViaPoint([1.0, 1.0], speed=0.3),
        ViaPoint([2.0, 2.0], speed=0.0)
    ]

    planner = TrajectoryPlanner(config)
    result = planner.plan(via_points)

    # 制約チェック
    max_velocity = max(abs(v) for v in result.velocities)
    max_acceleration = max(abs(a) for a in result.accelerations)

    tolerance = 1.01  # 1%の余裕
    assert max_velocity <= config.max_linear_speed * tolerance, \
        f"速度制約違反: {max_velocity:.3f} > {config.max_linear_speed}"
    assert max_acceleration <= config.max_linear_acceleration * tolerance, \
        f"加速度制約違反: {max_acceleration:.3f} > {config.max_linear_acceleration}"

    print(f'  [OK] 最大速度{max_velocity:.3f}m/s, 最大加速度{max_acceleration:.3f}m/s²')


def test_invalid_via_points():
    """不正な経由点のテスト"""
    print('Test: 不正な経由点の検出')

    config = TrajectoryConfig()
    planner = TrajectoryPlanner(config)

    # 1点のみ→エラー
    try:
        planner.plan([ViaPoint([0.0, 0.0])])
        assert False, "1点のみでエラーが発生しませんでした"
    except InvalidViaPointError:
        print('  [OK] 1点のみの経由点を正しく検出')

    # 2点のみ→エラー（Catmull-Romは最低3点必要）
    try:
        via_points = [
            ViaPoint([0.0, 0.0]),
            ViaPoint([1.0, 1.0])
        ]
        planner.plan(via_points)
        assert False, "2点のみでエラーが発生しませんでした"
    except InvalidViaPointError:
        print('  [OK] 2点のみの経由点を正しく検出')

    # 近すぎる経由点→エラー
    try:
        via_points = [
            ViaPoint([0.0, 0.0]),
            ViaPoint([0.0, 1e-7]),  # ほぼ同じ位置
            ViaPoint([1.0, 1.0])
        ]
        planner.plan(via_points)
        assert False, "近すぎる経由点でエラーが発生しませんでした"
    except InvalidViaPointError:
        print('  [OK] 近すぎる経由点を正しく検出')


def test_angle_profile():
    """角度プロファイルのテスト"""
    print('Test: 角度プロファイル')

    config = TrajectoryConfig()

    # 正の角度のテスト
    via_points_positive = [
        ViaPoint([0.0, 0.0], angle=0.0, speed=0.0),
        ViaPoint([1.0, 0.0], angle=np.pi/2, speed=0.3),
        ViaPoint([2.0, 0.0], angle=np.pi, speed=0.0)
    ]

    planner = TrajectoryPlanner(config)
    result = planner.plan(via_points_positive)

    # 始点と終点の角度をチェック
    start_angle = result.positions[0][2]
    end_angle = result.positions[-1][2]

    assert abs(start_angle - 0.0) < 0.1, f"始点角度が不一致: {start_angle}"
    assert abs(end_angle - np.pi) < 0.1, f"終点角度が不一致: {end_angle}"

    print(f'  [OK] 正の角度: 始点{start_angle:.3f}rad, 終点{end_angle:.3f}rad')

    # 負の角度のテスト
    via_points_negative = [
        ViaPoint([0.0, 0.0], angle=0.0, speed=0.0),
        ViaPoint([1.0, 0.0], angle=-np.pi/4, speed=0.3),
        ViaPoint([2.0, 0.0], angle=-np.pi/2, speed=0.0)
    ]

    result_neg = planner.plan(via_points_negative)
    start_angle_neg = result_neg.positions[0][2]
    end_angle_neg = result_neg.positions[-1][2]

    assert abs(start_angle_neg - 0.0) < 0.1, f"始点角度が不一致: {start_angle_neg}"
    assert abs(end_angle_neg - (-np.pi/2)) < 0.1, f"終点角度が不一致: {end_angle_neg}"

    print(f'  [OK] 負の角度: 始点{start_angle_neg:.3f}rad, 終点{end_angle_neg:.3f}rad')


def run_all_tests():
    """すべてのテストを実行"""
    print('=' * 60)
    print('TrajectoryPlanner テストスイート')
    print('=' * 60)
    print()

    tests = [
        test_basic_planning,
        test_straight_line,
        test_constraint_satisfaction,
        test_invalid_via_points,
        test_angle_profile
    ]

    passed = 0
    failed = 0

    for test in tests:
        try:
            test()
            passed += 1
        except AssertionError as e:
            print(f'  ✗ FAILED: {e}')
            failed += 1
        except Exception as e:
            print(f'  ✗ ERROR: {e}')
            failed += 1
        print()

    print('=' * 60)
    print(f'結果: {passed}件成功, {failed}件失敗')
    print('=' * 60)

    return failed == 0


if __name__ == '__main__':
    success = run_all_tests()
    sys.exit(0 if success else 1)
