import numpy as np
import matplotlib.pyplot as plt
from casadi import *

# ======================
# 车辆参数设置
# ======================
L = 2.9  # 轴距 (m)
dt = 0.1  # 时间步长 (s)
N = 10  # 预测时域
v_ref = 5.0  # 参考速度 (m/s)
max_steer = np.deg2rad(30)  # 最大前轮转角 (rad)
max_accel = 1.0  # 最大加速度 (m/s²)
max_decel = -3.0  # 最大减速度 (m/s²)


# ======================
# 运动学自行车模型（CasADi符号表达式）
# ======================
def bicycle_model():
    # 状态变量: [x, y, yaw, v]
    x = MX.sym('x', 4)
    # 控制输入: [steer, accel]
    u = MX.sym('u', 2)

    beta = arctan(tan(u[0]) / 2)  # 简化转向模型

    dx = x[3] * cos(x[2] + beta)
    dy = x[3] * sin(x[2] + beta)
    dyaw = x[3] * tan(u[0]) * cos(beta) / L
    dv = u[1]

    xdot = vertcat(dx, dy, dyaw, dv)
    return Function('f', [x, u], [xdot])


# ======================
# 生成参考轨迹 (圆形)
# ======================
def generate_circular_ref(t, radius=10.0):
    omega = v_ref / radius
    x_ref = radius * np.sin(omega * t)
    y_ref = radius * (1 - np.cos(omega * t))
    yaw_ref = omega * t
    return np.array([x_ref, y_ref, yaw_ref, v_ref])


# ======================
# MPC控制器类
# ======================
class CasadiMPC:
    def __init__(self):
        self.nx = 4  # 状态维度
        self.nu = 2  # 输入维度

        # 权重矩阵
        Q = diag([10.0, 10.0, 5.0, 1.0])  # 状态误差
        R = diag([0.5, 0.1])  # 控制输入
        self.Q = Q
        self.R = R

        # 构建预测模型
        self.f = bicycle_model()
        self.build_controller()

    def build_controller(self):
        # 优化问题变量
        X = MX.sym('X', self.nx, N + 1)  # 状态序列
        U = MX.sym('U', self.nu, N)  # 控制序列
        x0 = MX.sym('x0', self.nx)  # 初始状态
        ref = MX.sym('ref', self.nx, N + 1)  # 参考轨迹

        # 初始化目标函数和约束
        J = 0
        g = []

        # 初始状态约束
        g.append(X[:, 0] - x0)

        # 构建目标函数和动态约束
        for k in range(N):
            # 状态误差项
            J += mtimes([(X[:, k] - ref[:, k]).T, self.Q, (X[:, k] - ref[:, k])])

            # 控制输入项
            if k < N:
                J += mtimes([U[:, k].T, self.R, U[:, k]])

            # 动态约束
            x_next = X[:, k] + self.f(X[:, k], U[:, k]) * dt
            g.append(X[:, k + 1] - x_next)

            # 输入约束
            g.append(U[0, k] - max_steer)  # <=0
            g.append(-U[0, k] - max_steer)  # >=0
            g.append(U[1, k] - max_accel)  # <=0
            g.append(U[1, k] - max_decel)  # >=0

        # 终端代价
        J += mtimes([(X[:, -1] - ref[:, -1]).T, self.Q, (X[:, -1] - ref[:, -1])])

        # 创建优化问题
        opts = {'ipopt.print_level': 0, 'print_time': 0}
        nlp = {'x': vertcat(X.reshape((-1, 1))),
                            'f': J,
        'g': vertcat(*g),
        'p': vertcat(x0, ref.reshape((-1, 1)))}

        self.solver = nlpsol('solver', 'ipopt', nlp, opts)

    def solve(self, x0, ref_traj):
        # 将参考轨迹展平
        ref_flat = ref_traj.T.reshape((-1, 1))

        # 初始猜测（可选）
        x_init = np.tile(x0, N + 1)
        u_init = np.zeros(self.nu * N)
        vars_init = np.concatenate([x_init, u_init])

        # 求解NLP
        sol = self.solver(
            x0=vars_init,
            lbg=0.0,
            ubg=0.0,
            p=np.concatenate([x0, ref_flat])
        )

        if self.solver.stats()['success']:
            # 提取控制输入
            vars_opt = sol['x'].full().flatten()
            U_opt = vars_opt[self.nx * (N + 1):].reshape((self.nu, N))
            return U_opt[:, 0]  # 仅返回第一个控制输入
        else:
            print("求解失败！使用备用控制")
            return np.zeros(self.nu)


# ======================
# 主仿真循环
# ======================
if __name__ == "__main__":
    mpc = CasadiMPC()

    # 初始状态
    x0 = np.array([0.0, 0.0, 0.0, 0.0])

    # 存储结果
    history = {'x': [], 'y': [], 'yaw': [], 'v': [], 't': []}

    plt.figure(figsize=(10, 6))

    for k in range(200):
        # 生成参考轨迹
        ref_traj = np.zeros((4, N + 1))
        for i in range(N + 1):
            t = k * dt + i * dt
            ref_traj[:, i] = generate_circular_ref(t)

        # 求解MPC
        u_opt = mpc.solve(x0, ref_traj)

        # 应用控制输入
        x0 = x0 + mpc.f(x0, u_opt).full().flatten() * dt

        # 记录数据
        history['x'].append(x0[0])
        history['y'].append(x0[1])
        history['yaw'].append(x0[2])
        history['v'].append(x0[3])
        history['t'].append(k * dt)

        # 可视化
        plt.cla()
        plt.plot(ref_traj[0, :], ref_traj[1, :], 'r--', label="Reference")
        plt.plot(history['x'], history['y'], 'b-', label="Actual")
        plt.axis("equal")
        plt.grid(True)
        plt.title(f"CasADi MPC Tracking (t={k * dt:.1f}s)")
        plt.legend()
        plt.pause(0.01)

plt.show()