# -*- coding: utf-8 -*-
import numpy as np

class MinJerkProfile:
    #  @brief 初期化付きコンストラクタ
    #  @param x_start   開始位置 [m]
    #  @param self.x_target  目標位置 [m]
    #  @param time      移動時間 [s]
    def __init__(self, x_start, x_target, time):
        self.reset(x_start, x_target, time)

    #  @brief 引数の拘束条件から躍度最小曲線を生成する．
    #  この関数によって，すべての変数が初期化される．
    #  @param x_start   開始位置 [m]
    #  @param self.x_target  目標位置 [m]
    #  @param time      移動時間 [s]
    def reset(self, x_start, x_target, time):
        self.x_rate = x_target - x_start
        self.x_bias = x_start
        self.t_rate = 1/time

    #  @brief 時刻 $t$ における躍度 $j$
    #  @param t 時刻[s]
    #  @return j 躍度[m/s/s/s]
    def j(self, t):
        t_ = self.t_rate * t
        return self.x_rate * (360*t_**2 - 360*t_ + 60)

    #  @brief 時刻 $t$ における加速度 $a$
    #  @param t 時刻 [s]
    #  @return a 加速度 [m/s/s]
    def a(self, t):
        t_ = self.t_rate * t
        return self.x_rate * (120*t_**3 - 180*t_**2 + 60*t_)

    #  @brief 時刻 $t$ における速度 $v$
    #  @param t 時刻 [s]
    #  @return v 速度 [m/s]
    def v(self, t):
        t_ = self.t_rate * t
        return self.x_rate * (30*t_**4 - 60*t_**3 + 30*t_**2)

    #  @brief 時刻 $t$ における位置 $x$
    #  @param t 時刻 [s]
    #  @return x 位置 [m]
    def x(self, t):
        t_ = self.t_rate * t
        return self.x_rate * (6*t_**5 - 15*t_**4 + 10*t_**3) + self.x_bias
