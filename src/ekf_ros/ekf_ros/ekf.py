import numpy as np
import threading
from ekf_ros.math_tools import expSO2, logSO2, diff_SO2, transformAccelAndGyro2D


class State2D:
    """2次元状態（位置・姿勢・速度・バイアス）を表すクラス"""
    
    def __init__(self, pos, yaw, vel, ba=None, bg=None):
        self.pos = np.array(pos, dtype=float)
        self.yaw = np.array(yaw, dtype=float)
        self.vel = np.array(vel, dtype=float)
        self.bg = np.array(bg, dtype=float) if bg is not None else np.zeros(1)
        self.ba = np.array(ba, dtype=float) if ba is not None else np.zeros(2)

    def to_array(self):
        return np.concatenate([self.pos, self.yaw, self.vel, self.bg, self.ba])

    def __add__(self, delta):
        assert len(delta) == 8
        p = self.pos + delta[0:2]
        dy = float(delta[2])
        v = self.vel + delta[3:5]
        R = expSO2(self.yaw)
        Rn = R @ expSO2(dy)
        bg = self.bg + delta[5]
        ba = self.ba + delta[6:8]
        yaw_n = logSO2(Rn)
        return State2D(p, yaw_n, v, ba, bg)

    def __sub__(self, other):
        p = self.pos - other.pos
        v = self.vel - other.vel
        R_d = logSO2(expSO2(other.yaw).T @ expSO2(self.yaw))
        bg = self.bg - other.bg
        ba = self.ba - other.ba
        return np.concatenate([p, np.array([R_d]), v, bg, ba])

    def __len__(self):
        return 8


class EKF:
    """拡張カルマンフィルタ（IMU予測 + GNSS補正）"""
    
    def __init__(self, initial_mean, initial_cov, process_noise_cov, measurement_noise_cov,
                 imu_to_gnss_offset=None, gravity=9.80665):
        """
        Parameters
        ----------
        initial_mean : State2D
            初期状態
        initial_cov : ndarray (8x8)
            初期共分散行列
        process_noise_cov : ndarray (6x6)
            プロセスノイズ共分散 [n_wz, n_ax, n_ay, n_bg, n_bax, n_bay]
        measurement_noise_cov : ndarray (5x5)
            観測ノイズ共分散 [px, py, yaw, vx, vy]
        imu_to_gnss_offset : array-like (3,), optional
            GNSSから見たIMUの位置オフセット (default: [-0.330, 0, 0])
        gravity : float
            重力加速度 (default: 9.80665)
        """
        self.x = initial_mean
        self.P = np.array(initial_cov, dtype=float)
        
        self.Q = np.array(process_noise_cov, dtype=float)
        assert self.Q.shape == (6, 6), "process_noise_cov must be 6x6"
        
        self.R = np.array(measurement_noise_cov, dtype=float)
        assert self.R.shape == (5, 5), "measurement_noise_cov must be 5x5"
        
        self.gravity = gravity
        self._prev_wz = None
        
        self._lock = threading.Lock()

    def predict(self, imu_data, dt):
        """
        予測ステップ（IMUによる状態遷移）
        
        Parameters
        ----------
        imu_data : array-like
            [ax, ay, wz]
        dt : float
            時間刻み
        """
        with self._lock:
            # 入力整形
            a_m = np.array(imu_data[0:2], dtype=float)
            wz_m = float(imu_data[2])
            bg = float(self.x.bg[0])
            ba = self.x.ba.copy()

            # バイアス補正
            a_bm = a_m - ba #加速度, Eq(2)
            wz = wz_m - bg  #角速度, Eq(3)

            # 角加速度計算
            if self._prev_wz is None:
                dot_wz = 0.0
            else:
                dot_wz = (wz - self._prev_wz) / max(dt, 1e-6)
            self._prev_wz = wz

            # 剛体変換（IMU→GNSS位置）
            Rba = np.eye(3) #GNSS と IMU の向きは等しい
            tba = np.array([-0.330, 0.00, 0.00], dtype=float) #GNSS から見て IMU は 0.300m 後ろにある
            acc_gnss, gyro_gnss = transformAccelAndGyro2D(a_bm, wz, Rba, tba, dot_wz)

            # 離散時間状態遷移
            # Eq.(5)
            Rk = expSO2(self.x.yaw)
            pos = self.x.pos + self.x.vel * dt
            vel = self.x.vel + (Rk @ acc_gnss) * dt
            Rkp = Rk @ expSO2(gyro_gnss * dt)
            yawp = logSO2(Rkp)

            # ヤコビアン Fx (8x8)
            Fx = np.eye(8, dtype=float)
            Fx[0:2, 3:5] = np.eye(2) * dt
            
            S = np.array([[0, -1], [1, 0]], dtype=float)
            J_yaw = Rk @ S @ acc_gnss
            Fx[3:5, 2] += J_yaw * dt
            Fx[0:2, 2] += 0.5 * J_yaw * dt**2
            Fx[3:5, 6:8] += -Rk * dt

            # ノイズヤコビアン Fn (8x6)
            Fn = np.zeros((8, 6), dtype=float)
            Fn[2, 0] = dt
            Fn[3:5, 1:3] = -Rk * dt
            Fn[5, 3] = dt
            Fn[6:8, 4:6] = np.eye(2) * dt

            # 状態・共分散更新
            self.x = State2D(pos, yawp, vel, ba, self.x.bg.copy())
            self.P = Fx @ self.P @ Fx.T + Fn @ self.Q @ Fn.T #Eq.(6)

    def correct(self, z):
        """
        補正ステップ（GNSS観測による更新）
        
        Parameters
        ----------
        z : array-like (5,)
            観測値 [px, py, yaw, vx, vy]
        """
        z = np.asarray(z, dtype=float)
        
        with self._lock:
            # 観測予測
            yaw = float(self.x.yaw)
            pos = np.asarray(self.x.pos).reshape(2,)
            vel = np.asarray(self.x.vel).reshape(2,)
            h = np.array([pos[0], pos[1], yaw, vel[0], vel[1]], dtype=float) #Eq.(8)

            # 観測ヤコビアン H (5x8)
            H = np.zeros((5, 8), dtype=float)
            H[0:2, 0:2] = np.eye(2)
            H[2, 2] = 1.0
            H[3:5, 3:5] = np.eye(2)
            
            # イノベーション計算
            z_pred = h
            y = z - z_pred
            y[2] = diff_SO2(float(z[2]), float(z_pred[2])) #Eq.(9)
            
            # カルマンゲイン計算
            # Eq.(10)
            S = H @ self.P @ H.T + self.R
            K = self.P @ H.T @ np.linalg.pinv(S)
            
            # 状態・共分散更新
            self.x = self.x + K @ y #Eq.(11)
            I = np.eye(len(self.x))
            self.P = (I - K @ H) @ self.P

    def get_estimate(self):
        """
        現在の推定値と共分散を取得
        
        Returns
        -------
        x : State2D
            状態推定値
        P : ndarray (8x8)
            共分散行列
        """
        with self._lock:
            return self.x, self.P.copy()

    def reset_imu_state(self):
        """IMU内部状態（前回角速度）をリセット"""
        with self._lock:
            self._prev_wz = None