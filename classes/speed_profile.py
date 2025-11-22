# -*- coding: utf-8 -*-
import numpy as np

# @brief 加速曲線を生成するクラス
#
# - 引数の拘束に従って加速曲線を生成する
# - 始点速度と終点速度を滑らかにつなぐ
# - 移動距離の拘束はない
# - 始点速度および終点速度は，正でも負でも可
class AccelCurve:

    # @brief 初期化付きのコンストラクタ．
    # @param j_max   最大躍度の大きさ [m/s/s/s], 正であること
    # @param a_max   最大加速度の大きさ [m/s/s], 正であること
    # @param v_start 始点速度 [m/s]
    # @param v_end   終点速度 [m/s]
    def __init__(self, j_max = 1, a_max = 1, v_start = 0, v_end = 0):
        # とりあえず適当に初期化
        self.jm = 1
        self.am = 1
        self.t0 = 0
        self.t1 = 0
        self.t2 = 0
        self.t3 = 0
        self.v0 = 0
        self.v1 = 0
        self.v2 = 0
        self.v3 = 0
        self.x0 = 0
        self.x1 = 0
        self.x2 = 0
        self.x3 = 0
        self.reset(j_max, a_max, v_start, v_end)

    # @brief 引数の拘束条件から曲線を生成する．
    # @param j_max   最大躍度の大きさ [m/s/s/s], 正であること
    # @param a_max   最大加速度の大きさ [m/s/s], 正であること
    # @param v_start 始点速度 [m/s]
    # @param v_end   終点速度 [m/s]
    def reset(self, j_max, a_max, v_start, v_end):
        # 符号付きで代入
        # 最大加速度の符号を決定
        if v_end > v_start:
            self.am = a_max
        else:
            self.am = -a_max

        # 最大躍度の符号を決定
        if v_end > v_start:
            self.jm = j_max
        else:
            self.jm = -j_max

        # 初期値と最終値を代入
        self.v0 = v_start # 代入
        self.v3 = v_end   # 代入
        self.t0 = 0       # ここでは初期値をゼロとする
        self.x0 = 0       # ここでは初期値はゼロとする

        # 速度が曲線となる部分の時間を決定
        tc = a_max / j_max

        # 等加速度直線運動の時間を決定
        tm = (self.v3 - self.v0) / self.am - tc

        # 等加速度直線運動の有無で分岐
        if tm > 0:
            # 速度: 曲線 -> 直線 -> 曲線
            self.t1 = self.t0 + tc
            self.t2 = self.t1 + tm
            self.t3 = self.t2 + tc
            self.v1 = self.v0 + self.am * tc / 2                # v(t) を積分
            self.v2 = self.v1 + self.am * tm                    # v(t) を積分
            self.x1 = self.x0 + self.v0 * tc + self.am * tc * tc / 6 # x(t) を積分
            self.x2 = self.x1 + self.v1 * tm                    # x(t) を積分
            self.x3 = self.x0 + (self.v0 + self.v3) / 2 * (self.t3 - self.t0) # v(t) グラフの台形の面積より
        else:
            # 速度: 曲線 -> 曲線
            tcp = np.sqrt((self.v3 - self.v0) / self.jm) # 変曲までの時間
            self.t1 = self.t2 = self.t0 + tcp
            self.t3 = self.t2 + tcp
            self.v1 = self.v2 = (self.v0 + self.v3) / 2 # 対称性より中点となる
            self.x1 = self.x2 = self.x0 + self.v1 * tcp + self.jm * tcp * tcp * tcp / 6 # x(t) を積分
            self.x3 = self.x0 + 2 * self.v1 * tcp # 速度 v(t) グラフの面積より



    # @brief 時刻 $t$ における躍度 $j$
    # @param t 時刻 [s]
    # @return j 躍度 [m/s/s/s]
    def j(self, t):
        if t <= self.t0:
            return 0
        elif t <= self.t1:
            return self.jm
        elif t <= self.t2:
            return 0
        elif t <= self.t3:
            return -self.jm
        else:
            return 0


    # @brief 時刻 $t$ における加速度 $a$
    # @param t 時刻 [s]
    # @return a 加速度 [m/s/s]
    def a(self, t):
        if t <= self.t0:
            return 0
        elif t <= self.t1:
            return self.jm * (t - self.t0)
        elif t <= self.t2:
            return self.am
        elif t <= self.t3:
            return -self.jm * (t - self.t3)
        else:
            return 0


    # @brief 時刻 $t$ における速度 $v$
    # @param t 時刻 [s]
    # @return v 速度 [m/s]
    def v(self, t):
        if t <= self.t0:
            return self.v0
        elif t <= self.t1:
            return self.v0 + self.jm / 2 * (t - self.t0) * (t - self.t0)
        elif t <= self.t2:
            return self.v1 + self.am * (t - self.t1)
        elif t <= self.t3:
            return self.v3 - self.jm / 2 * (t - self.t3) * (t - self.t3)
        else:
            return self.v3


    # @brief 時刻 $t$ における位置 $x$
    # @param t 時刻 [s]
    # @return x 位置 [m]
    def x(self, t):
        if t <= self.t0:
            return self.x0 + self.v0 * (t - self.t0)
        elif t <= self.t1:
            return self.x0 + self.v0 * (t - self.t0) + self.jm / 6 * (t - self.t0) * (t - self.t0) * (t - self.t0)
        elif t <= self.t2:
            return self.x1 + self.v1 * (t - self.t1) + self.am / 2 * (t - self.t1) * (t - self.t1)
        elif t <= self.t3:
            return self.x3 + self.v3 * (t - self.t3) - self.jm / 6 * (t - self.t3) * (t - self.t3) * (t - self.t3)
        else:
            return self.x3 + self.v3 * (t - self.t3)


    # @brief 終点時刻 [s]
    def t_end(self):
        return self.t3

    # @brief 終点速度 [m/s]
    def v_end(self):
        return self.v3

    # @brief 終点位置 [m]
    def x_end(self):
        return self.x3

    # @brief 境界の時刻
    def t_0(self):
        return self.t0
    def t_1(self):
        return self.t1
    def t_2(self):
        return self.t2
    def t_3(self):
        return self.t3

    # @brief 走行距離から達しうる終点速度を算出する関数
    # @param j_max 最大躍度の大きさ [m/s/s/s], 正であること
    # @param a_max 最大加速度の大きさ [m/s/s], 正であること
    # @param vs    始点速度 [m/s]
    # @param vt    目標速度 [m/s]
    # @param d     走行距離 [m]
    # @return ve   終点速度 [m/s]
    @staticmethod
    def calcVelocityEnd(j_max, a_max, vs, vt, d):
        # 速度が曲線となる部分の時間を決定
        tc = a_max / j_max

        # 最大加速度の符号を決定
        am = 0
        if vt > vs:
            am = a_max
        else:
            am = -a_max

        jm = 0
        if vt > vs:
            jm = j_max
        else:
            jm = -j_max

        # 等加速度直線運動の有無で分岐
        d_triangle = (vs + am * tc / 2) * tc # distance @ tm == 0
        v_triangle = jm / am * d - vs        # v_end @ tm == 0

        if d * v_triangle > 0 and abs(d) > abs(d_triangle):
            # 曲線・直線・曲線
            # print("v: curve - straight - curve")

            # 2次方程式の解の公式を解く
            amtc = am * tc
            D = amtc * amtc - 4 * (amtc * vs - vs * vs - 2 * am * d)
            np.sqrtD = np.sqrt(D)

            if d > 0:
                return (-amtc + np.sqrtD) / 2
            else:
                return (-amtc + (-np.sqrtD)) / 2

        # 曲線・曲線 (走行距離が短すぎる)
        # 3次方程式を解いて，終点速度を算出
        # 簡単のため，値を一度すべて正に変換して，計算結果に符号を付与して返送
        a = abs(vs)

        b = 0
        if d > 0:
            b = jm * d * d
        else:
            b = -1 * jm * d * d

        aaa = a * a * a
        c0 = 27 * (32 * aaa * b + 27 * b * b)
        c1 = 16 * aaa + 27 * b

        if (c0 >= 0):
            # ルートの中が非負のとき
            # print("v: curve - curve (accel)")
            c2 = np.cbrt((np.sqrt(c0) + c1) / 2)

            # 3次方程式の解
            if d > 0:
                return (c2 + 4 * a * a / c2 - a) / 3
            else:
                return -1*(c2 + 4 * a * a / c2 - a) / 3
        else:
            # ルートの中が負のとき
            # print("v: curve - curve (decel)")
            # c2 = pow(complex<float>(c1 / 2, np.sqrt(-c0) / 2), float(1) / 3)
            c2 = np.power(complex(c1 / 2, np.sqrt(-c0) / 2), float(1 / 3))

            if d > 0:
                return (c2.real * 2 - a) / 3
            else:
                return -(c2.real * 2 - a) / 3



    # @brief 走行距離から達しうる最大速度を算出する関数
    # @param j_max 最大躍度の大きさ [m/s/s/s], 正であること
    # @param a_max 最大加速度の大きさ [m/s/s], 正であること
    # @param vs    始点速度 [m/s]
    # @param ve    終点速度 [m/s]
    # @param d     走行距離 [m]
    # @return vm   最大速度 [m/s]
    @staticmethod
    def calcVelocityMax(j_max, a_max, vs, ve, d):
        # 速度が曲線となる部分の時間を決定
        tc = a_max / j_max
        am = 0
        if d > 0:
            am = a_max
        else:
            am =-a_max # 加速方向は移動方向に依存

        # 2次方程式の解の公式を解く
        amtc = am * tc
        D = amtc * amtc - 2 * (vs + ve) * amtc + 4 * am * d + 2 * (vs * vs + ve * ve)

        if D < 0:
            # 拘束条件がおかしい
            print("[Error] D = " + D + " < 0")

            # 入力のチェック
            if vs * ve < 0:
                print("[Error] 不正な入力速度です vs: " + vs + ", ve: " + ve)
                return vs

        np.sqrtD = np.sqrt(D)

        if d > 0:
            return (-amtc + np.sqrtD) / 2 # 2次方程式の解
        else:
            return (-amtc + (-np.sqrtD)) / 2 # 2次方程式の解


    # @brief 速度差から変位を算出する関数
    # @param j_max   最大躍度の大きさ [m/s/s/s], 正であること
    # @param a_max   最大加速度の大きさ [m/s/s], 正であること
    # @param v_start 始点速度 [m/s]
    # @param v_end   終点速度 [m/s]
    # @return d      変位 [m]
    @staticmethod
    def calcMinDistance(j_max, a_max, v_start, v_end):
        # 符号付きで代入
        am = 0
        if v_end > v_start:
            am = a_max
        else:
            am = -a_max

        jm = 0
        if v_end > v_start:
            jm = j_max
        else:
            jm = -j_max

        # 速度が曲線となる部分の時間を決定
        tc = a_max / j_max

        # 等加速度直線運動の時間を決定
        tm = (v_end - v_start) / am - tc

        # 始点から終点までの時間を決定
        t_all = 0
        if tm > 0:
            t_all = tc + tm + tc
        else:
            t_all = 2 * np.sqrt((v_end - v_start) / jm)

        return (v_start + v_end) / 2 * t_all # 速度グラフの面積により




#  @brief 拘束条件を満たす曲線加減速の軌道を生成するクラス
#  - 移動距離の拘束条件を満たす曲線加速軌道を生成する
#  - 各時刻 $t$ における躍度 $j(t)$，加速度 $a(t)$，速度 $v(t)$，位置 $x(t)$
#  を提供する
#  - 最大加速度 $a_\max$ と始点速度 $v_s$
#  など拘束次第では目標速度に達することができない場合があるので注意する
class SpeedProfile:
    #  @brief 初期化付きコンストラクタ
    #
    #  @param j_max         最大躍度の大きさ [m/s/s/s]，正であること
    #  @param a_max         最大加速度の大きさ [m/s/s], 正であること
    #  @param v_sat         飽和速度の大きさ [m/s]，正であること
    #  @param v_start     始点速度 [m/s]
    #  @param v_target    目標速度 [m/s]
    #  @param v_end         終点速度 [m/s]
    #  @param dist            移動距離 [m]
    #  @param x_start     始点位置 [m] (オプション)
    #  @param t_start     始点時刻 [s] (オプション)

    def __init__(self, j_max=0, a_max=0, v_sat=0, v_start=0, v_target=0, dist=0, x_start = 0, t_start = 0):
        self.t0 = 0
        self.t1 = 0
        self.t2 = 0
        self.t3 = 0 ##brief 境界点の時刻 [s]
        self.x0 = 0
        self.x3 = 0        #        #brief 境界点の位置 [m]

        self.ac = AccelCurve()
        self.dc = AccelCurve() ##brief 曲線加速，曲線減速オブジェクト

        if j_max is 0:
            return
        self.reset(j_max, a_max, v_sat, v_start, v_target, dist, x_start, t_start)


    #  @brief 空のコンストラクタ．あとで reset() により初期化すること．

    # AccelDesigner()  self.t0 = self.t1 = self.t2 = self.t3 = self.x0 = self.x3 = 0

    #  @brief 引数の拘束条件から曲線を生成する．
    #  この関数によって，すべての変数が初期化される．(漏れはない)
    #
    #  @param j_max         最大躍度の大きさ [m/s/s/s]，正であること
    #  @param a_max         最大加速度の大きさ [m/s/s], 正であること
    #  @param v_sat         飽和速度の大きさ [m/s]，正であること
    #  @param v_start     始点速度 [m/s]
    #  @param v_target    目標速度 [m/s]
    #  @param v_end         終点速度 [m/s]
    #  @param dist            移動距離 [m]
    #  @param x_start     始点位置 [m] (オプション)
    #  @param t_start     始点時刻 [s] (オプション)

    def reset(self, j_max, a_max, v_sat, v_start, v_target, dist, x_start = 0, t_start = 0):
        # 最大速度の仮置き
        v_max = 0
        if dist > 0:
            v_max = max(v_start, v_sat, v_target)
        else:
            v_max = min(v_start, -v_sat, v_target)

        # 走行距離から終点速度$v_e$を算出
        v_end = v_target
        dist_min = AccelCurve.calcMinDistance(j_max, a_max, v_start, v_end)

        # logd << "dist_min: " << dist_min)
        if abs(dist) < abs(dist_min):
            print("[Info] 走行距離の拘束を満たすため，最大速度まで加速できません")
            # 目標速度$v_t$に向かい，走行距離$d$で到達し得る終点速度$v_e$を算出
            v_end = AccelCurve.calcVelocityEnd(j_max, a_max, v_start, v_target, dist)
            v_max = v_end # 走行距離の拘束を満たすため，飽和速度まで加速できない
            # logd << "ve: " << v_end)

        # 曲線を生成
        self.ac.reset(j_max, a_max, v_start, v_max) # 加速
        self.dc.reset(j_max, a_max, v_max, v_end)     # 減速

        # 飽和速度まで加速すると走行距離の拘束を満たさない場合の処理
        d_sum = self.ac.x_end() + self.dc.x_end()
        if abs(dist) < abs(d_sum):
            print("走行距離の拘束を満たすため，最大速度まで加速できません")
            # 走行距離から最大速度$v_m$を算出 下記v_maxは上記v_max以下になる
            v_max = AccelCurve.calcVelocityMax(j_max, a_max, v_start, v_end, dist)

            # 無駄な減速を回避
            v_max = 0
            if dist > 0:
                v_max = max(v_start, v_max, v_end)
            else:
                v_max = min(v_start, v_max, v_end)
            self.ac.reset(j_max, a_max, v_start, v_max) # 加速
            self.dc.reset(j_max, a_max, v_max, v_end)     # 減速

        # 発散回避
        if v_max == 0:
            v_max = 1

        # 各定数の算出
        self.x0 = x_start
        self.x3 = x_start + dist
        self.t0 = t_start
        self.t1 = self.t0 + self.ac.t_end() # 曲線加速終了の時刻
        self.t2 = self.t0 + self.ac.t_end() + (dist - self.ac.x_end() - self.dc.x_end()) / v_max # 等速走行終了の時刻
        self.t3 = self.t0 + self.ac.t_end() + (dist - self.ac.x_end() - self.dc.x_end()) / v_max + self.dc.t_end() # 曲線減速終了の時刻

        # 出力のチェック
        e = 0.00001 # 数値誤差分
        show_info = False
        # 終点速度
        if abs(v_start - v_end) > e + abs(v_start - v_target):
            print("[Error] 終端速度が不正です")
            show_info = True

        # 最大速度
        if abs(v_max) > e + max(v_sat, abs(v_start), abs(v_end)):
            print("[Error] 最大速度が不正です")
            show_info = True

        # タイムスタンプ
        if not(self.t0 <= self.t1 + e and self.t1 <= self.t2 + e and self.t2 <= self.t3 + e):
            print("[Error] 時間関係が不正です")
            show_info = True

    #  @brief 時刻 $t$ における躍度 $j$
    #  @param t 時刻[s]
    #  @return j 躍度[m/s/s/s]
    def j(self, t):
        if t < self.t2:
            return self.ac.j(t - self.t0)
        else:
            return self.dc.j(t - self.t2)


    #  @brief 時刻 $t$ における加速度 $a$
    #  @param t 時刻 [s]
    #  @return a 加速度 [m/s/s]
    def a(self, t):
        if (t < self.t2):
            return self.ac.a(t - self.t0)
        else:
            return self.dc.a(t - self.t2)


    #  @brief 時刻 $t$ における速度 $v$
    #  @param t 時刻 [s]
    #  @return v 速度 [m/s]
    def v(self, t):
        if t < self.t2:
            return self.ac.v(t - self.t0)
        else:
            return self.dc.v(t - self.t2)


    #  @brief 時刻 $t$ における位置 $x$
    #  @param t 時刻 [s]
    #  @return x 位置 [m]
    def x(self, t):
        if t < self.t2:
            return self.x0 + self.ac.x(t - self.t0)
        else:
            return self.x3 - self.dc.x_end() + self.dc.x(t - self.t2)

    #  @brief 終点時刻 [s]
    def t_end(self):
        return self.t3

    #  @brief 終点速度 [m/s]
    def v_end(self):
        return self.dc.v_end()

    #  @brief 終点位置 [m]
    def x_end(self):
        return self.x3

    #  @brief 境界の時刻
    def t_0(self):
        return self.t0

    def t_1(self):
        return self.t1

    def t_2(self):
        return self.t2

    def t_3(self):
        return self.t3
