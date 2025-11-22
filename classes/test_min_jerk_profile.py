import numpy as np
import matplotlib.pyplot as plt
import min_jerk_profile

if __name__ == "__main__":
    end_time = 10.0

    # クラスのインスタンス化
    mjp = min_jerk_profile.MinJerkProfile(
        x_start = 0,
        x_target = -2,
        time = end_time)

    # 表示用配列
    cal_hz = 100
    time = []
    dist = []
    vel = []
    acc = []

    # 計算結果代入
    for i in range(int(end_time * cal_hz)):
        t = i/cal_hz
        time.append(t)
        dist.append(mjp.x(t))
        vel.append(mjp.v(t))
        acc.append(mjp.a(t))

    # プロット
    fig = plt.figure()
    ax = plt.axes()
    ax.plot(time, dist)
    ax.plot(time, vel)
    ax.plot(time, acc)
    plt.show()
