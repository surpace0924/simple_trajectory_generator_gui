import numpy as np
import matplotlib.pyplot as plt
import speed_profile

if __name__ == "__main__":
    # クラスのインスタンス化
    sp = speed_profile.SpeedProfile(
        j_max = 150,
        a_max = 15,
        v_sat = 7.5,
        v_start = 0,
        v_target = 0,
        dist = 5,
        x_start = 0,
        t_start = 0)

    # 表示用配列
    cal_hz = 100
    time = []
    dist = []
    vel = []
    acc = []

    # 計算結果代入
    for i in range(int(sp.t_end() * cal_hz)):
        t = i/cal_hz
        time.append(t)
        dist.append(sp.x(t))
        vel.append(sp.v(t))
        acc.append(sp.a(t))

    # プロット
    fig = plt.figure()
    ax = plt.axes()
    ax.plot(time, dist)
    ax.plot(time, vel)
    ax.plot(time, acc)
    plt.show()
