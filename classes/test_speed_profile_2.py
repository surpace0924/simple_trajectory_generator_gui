import numpy as np
import matplotlib.pyplot as plt
import speed_profile

#  @param j_max     最大躍度の大きさ [m/s/s/s]，正であること
#  @param a_max     最大加速度の大きさ [m/s/s], 正であること
#  @param v_sat     飽和速度の大きさ [m/s]，正であること
#  @param v_start   始点速度 [m/s]
#  @param v_target  目標速度 [m/s]
#  @param v_end     終点速度 [m/s]
#  @param dist      移動距離 [m]
#  @param x_start   始点位置 [m] (オプション)
#  @param t_start   始点時刻 [s] (オプション)

if __name__ == "__main__":
    # パラメータ
    j_max = 3
    a_max = 1
    v_max = 1
    vel_list = [1, 0, 2, 0] # 終端速度
    len_list = [1, 2, 1, 3] # 経路長

    # 表示用配列
    cal_hz = 100
    time = []
    dist = []
    vel = []
    acc = []

    # クラスのインスタンス化
    sp = speed_profile.SpeedProfile()

    # プロファイル読み出し
    for i in range(len(len_list)):
        t0 = sp.t_end()
        sp.reset(j_max, a_max, v_max, sp.v_end(), vel_list[i], len_list[i], sp.x_end(), sp.t_end())
        for j in range(int((sp.t_end() - t0) * cal_hz)):
            t = j/cal_hz + t0
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
