# #!/usr/bin/env python3
# # ==================================================
# #  Fast MPC for 2-Wheel Self-Balancing Robot
# #  Raspberry Pi 4 → ODrive V3.6 (UART)
# #  Linear QP MPC, 100 Hz update rate
# # ==================================================

# import casadi as ca
# import numpy as np
# import time
# import serial
# from smbus2 import SMBus
# import math

# # ===============================
# #  Thông số hệ thống vật lý
# # ===============================
# dt = 0.01        # 100 Hz
# N = 10           # MPC horizon
# g = 9.81
# l = 0.15         # khoảng cách tâm khối lượng → trục bánh
# m = 3.0          # khối lượng robot (kg)
# r = 0.065        # bán kính bánh xe (m)
# L = 0.40         # khoảng cách giữa 2 bánh xe (m)

# # ===============================
# #  Cảm biến MPU6050
# # ===============================
# MPU_ADDR = 0x68
# bus = SMBus(1)

# def init_mpu():
#     bus.write_byte_data(MPU_ADDR, 0x6B, 0)  # bật cảm biến

# def read_mpu(phi_prev, alpha=0.98):
#     data = bus.read_i2c_block_data(MPU_ADDR, 0x3B, 14)
#     def bytes_to_int16(high, low):
#         val = (high << 8) | low
#         if val > 32767:
#             val -= 65536
#         return val

#     accX = bytes_to_int16(data[0], data[1])
#     accY = bytes_to_int16(data[2], data[3])
#     accZ = bytes_to_int16(data[4], data[5])
#     gyroX = bytes_to_int16(data[8], data[9])

#     # chuyển về đơn vị SI
#     accY_f = accY / 16384.0
#     accZ_f = accZ / 16384.0
#     gyroX_f = gyroX / 131.0  # °/s

#     phi_acc = math.atan2(accY_f, accZ_f)
#     dphi = gyroX_f * math.pi / 180.0  # rad/s

#     # bộ lọc bổ sung
#     phi = alpha * (phi_prev + dphi * dt) + (1 - alpha) * phi_acc
#     return phi, dphi

# # ===============================
# #  UART tới ODrive
# # ===============================
# ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.01)

# def set_velocity(axis, velocity):
#     cmd = f"v {axis} {velocity:.5f}\n".encode()
#     ser.write(cmd)

# def read_velocity(axis):
#     try:
#         ser.reset_input_buffer()
#     except:
#         pass
#     ser.write(f"r axis{axis}.encoder.vel_estimate\n".encode())
#     line = ser.readline().decode(errors='ignore').strip()
#     try:
#         return float(line)
#     except:
#         return 0.0

# # ===============================
# #  Mô hình tuyến tính hóa (discrete)
# # ===============================
# A = np.array([
#     [1, dt, 0, 0],
#     [(g/l)*dt, 1, 0, 0],
#     [0, 0, 1, dt],
#     [0, 0, 0, 1]
# ])
# B = np.array([
#     [0, 0],
#     [dt/(2*l), dt/(2*l)],
#     [r*dt/2, r*dt/2],
#     [r*dt/L, -r*dt/L]
# ])

# # ===============================
# #  Thiết lập QP MPC
# # ===============================
# nx = 4
# nu = 2

# Q = np.diag([1.0, 0.1, 0.1, 0.05])
# R = np.diag([0.05, 0.05])

# # Biến CasADi
# x = ca.SX.sym('x', nx)
# u = ca.SX.sym('u', nu)
# x_ref = ca.SX.sym('x_ref', nx)

# # Hàm chi phí 1 bước
# stage_cost = (x - x_ref).T @ Q @ (x - x_ref) + u.T @ R @ u
# f = ca.Function('f', [x, u, x_ref], [stage_cost])

# # Biến tối ưu
# U = ca.SX.sym('U', nu, N)
# X = ca.SX.sym('X', nx, N + 1)

# x0 = ca.SX.sym('x0', nx)
# g_eq = []
# cost = 0

# # Ràng buộc động học
# for k in range(N):
#     x_next = ca.mtimes(A, X[:, k]) + ca.mtimes(B, U[:, k])
#     g_eq.append(X[:, k + 1] - x_next)
#     cost += f(X[:, k], U[:, k], x_ref)

# # Tạo ràng buộc
# g_eq = ca.vertcat(*g_eq)

# # Giới hạn điều khiển
# umin = [-3, -3]
# umax = [3, 3]

# # QP Solver
# qp = {'x': ca.vertcat(ca.vec(U), ca.vec(X)), 'f': cost, 'g': g_eq}
# opts = {"print_time": 0, "osqp.verbose": False}
# solver = ca.qpsol('solver', 'osqp', qp, opts)

# # ===============================
# #  Hàm điều khiển
# # ===============================
# def mpc_step(x_state, x_ref_val):
#     lbx = np.tile(umin, N) + [-ca.inf]*(nx*(N+1))
#     ubx = np.tile(umax, N) + [ca.inf]*(nx*(N+1))
#     lbg = np.zeros(nx*N)
#     ubg = np.zeros(nx*N)

#     arg = {
#         'x0': np.zeros(U.size1() * U.size2() + X.size1() * X.size2()),
#         'lbx': lbx,
#         'ubx': ubx,
#         'lbg': lbg,
#         'ubg': ubg,
#         'p': []
#     }

#     sol = solver(**arg)
#     sol_vec = np.array(sol['x']).flatten()
#     u_cmd = sol_vec[0:nu]
#     return u_cmd

# # ===============================
# #  Chạy chính
# # ===============================
# init_mpu()
# phi = 0.0
# print("Fast MPC Controller running at 100 Hz...")

# try:
#     t_prev = time.time()
#     while True:
#         # Đọc góc nghiêng
#         phi, dphi = read_mpu(phi)

#         # Đọc vận tốc bánh
#         vL = read_velocity(0)
#         vR = read_velocity(1)
#         v_meas = (vL + vR) / 2.0

#         # Mục tiêu
#         v_target = 0.0
#         yaw_target = 0.0

#         # Trạng thái & tham chiếu
#         x_state = np.array([phi, dphi, v_meas, 0.0])
#         x_ref_val = np.array([0.0, 0.0, v_target, yaw_target])

#         # Giải MPC
#         u_cmd = mpc_step(x_state, x_ref_val)
#         vR_cmd, vL_cmd = float(u_cmd[0]), float(u_cmd[1])

#         # Gửi lệnh tới ODrive
#         set_velocity(0, -vL_cmd)
#         set_velocity(1, vR_cmd)

#         # Giữ tần số 100 Hz
#         t_now = time.time()
#         dt_loop = t_now - t_prev

#         print(dt_loop)
#         delay = dt - dt_loop
#         if delay > 0:
#             time.sleep(delay)
#         t_prev = time.time()

#         # Gỡ comment để debug tốc độ
#         # print(f"Loop: {dt_loop*1000:.2f} ms, phi={phi:.3f}, vR={vR_cmd:.3f}, vL={vL_cmd:.3f}")

# except KeyboardInterrupt:
#     set_velocity(0, 0)
#     set_velocity(1, 0)
#     print("\nStopped by user.")
# except Exception as e:
#     print("Error:", e)
#     set_velocity(0, 0)
#     set_velocity(1, 0)


#!/usr/bin/env python3
# ==================================================
#  Fast MPC for 2-Wheel Self-Balancing Robot
#  Raspberry Pi 4 → ODrive V3.6 (UART)
#  Linear QP MPC, 100 Hz update rate
#  Fixed: include x0 & x_ref as parameters; enforce X[:,0]==x0
# ==================================================

import casadi as ca
import numpy as np
import time
import serial
from smbus2 import SMBus
import math
TWO_PI = 2.0 * math.pi

# ===============================
#  Thông số hệ thống vật lý
# ===============================
dt = 0.01        # 100 Hz
N = 10          # MPC horizon
g = 9.81
l = 0.2         # khoảng cách tâm khối lượng → trục bánh
m = 3.0          # khối lượng robot (kg)
r = 0.08255        # bán kính bánh xe (m)
L = 0.40         # khoảng cách giữa 2 bánh xe (m)

# ===============================
#  Cảm biến MPU6050
# ===============================
MPU_ADDR = 0x68
bus = SMBus(1)

def init_mpu():
    bus.write_byte_data(MPU_ADDR, 0x6B, 0)  # bật cảm biến

def read_mpu(phi_prev, alpha=0.98):
    data = bus.read_i2c_block_data(MPU_ADDR, 0x3B, 14)
    def bytes_to_int16(high, low):
        val = (high << 8) | low
        if val > 32767:
            val -= 65536
        return val

    accX = bytes_to_int16(data[0], data[1])
    accY = bytes_to_int16(data[2], data[3])
    accZ = bytes_to_int16(data[4], data[5])
    gyroX = bytes_to_int16(data[8], data[9])

    # chuyển về đơn vị SI / chuẩn sensor scale
    accY_f = accY / 16384.0
    accZ_f = accZ / 16384.0
    gyroX_f = gyroX / 131.0  # °/s

    phi_acc = math.atan2(accY_f, accZ_f)
    dphi = gyroX_f * math.pi / 180.0  # rad/s

    # bộ lọc bổ sung (complementary)
    phi = alpha * (phi_prev + dphi * dt) + (1 - alpha) * phi_acc
    return phi, dphi

# ===============================
#  UART tới ODrive
# ===============================
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.01)

def set_velocity(axis, velocity):
    cmd = f"v {axis} {velocity:.5f}\n".encode()
    ser.write(cmd)

def set_torque(axis, torque):
    cmd = f"c {axis} {torque:.4f}\n".encode()
    ser.write(cmd)


def read_velocity(axis):
    try:
        ser.reset_input_buffer()
    except:
        pass
    ser.write(f"r axis{axis}.encoder.vel_estimate\n".encode())
    line = ser.readline().decode(errors='ignore').strip()
    try:
        return float(line)
    except:
        return 0.0

# ===============================
#  Mô hình tuyến tính hóa (discrete)
# ===============================
A = np.array([
    [1, dt, 0, 0],
    [(g/l)*dt, 1, 0, 0],
    [0, 0, 1, dt],
    [0, 0, 0, 1]
])
B = np.array([
    [0, 0],
    [dt/(2*l), dt/(2*l)],
    [r*dt/2, r*dt/2],
    [r*dt/L, -r*dt/L]
])

# ===============================
#  Thiết lập QP MPC
# ===============================
nx = 4
nu = 2

Q = np.diag([80.0, 1.0, 50, 10])
R = np.diag([0.13, 0.13])

# build stage cost as CasADi expressions
X_sym = ca.SX.sym('Xsym', nx)
U_sym = ca.SX.sym('Usym', nu)
Xref_sym = ca.SX.sym('Xref', nx)
stage_cost = ca.mtimes([(X_sym - Xref_sym).T, Q, (X_sym - Xref_sym)]) + ca.mtimes([U_sym.T, R, U_sym])
stage_cost_fun = ca.Function('stage_cost_fun', [X_sym, U_sym, Xref_sym], [stage_cost])

# Decision variables: U (nu x N) and X (nx x (N+1))
U = ca.SX.sym('U', nu, N)
X = ca.SX.sym('X', nx, N + 1)

# Parameters: initial state x0 and reference x_ref
x0 = ca.SX.sym('x0', nx)
x_ref = ca.SX.sym('x_ref', nx)

# Build constraints and cost
g = []
obj = 0

# Constraint: X[:,0] == x0
g.append(X[:, 0] - x0)

for k in range(N):
    # dynamics constraint
    x_next = ca.mtimes(A, X[:, k]) + ca.mtimes(B, U[:, k])
    g.append(X[:, k + 1] - x_next)
    # stage cost with reference x_ref
    obj += stage_cost_fun(X[:, k], U[:, k], x_ref)

# Stack constraints
g = ca.vertcat(*g)

# Decision vector (stacked)
dec_var = ca.vertcat(ca.vec(U), ca.vec(X))

# Build QP data structure for qpsol
# qpsol expects 'x','f','g' and optional 'p' (parameters)
qp = {
    'x': dec_var,
    'f': obj,
    'g': g,
    'p': ca.vertcat(x0, x_ref)
}

# Solver options
opts = {"print_time": 0, "osqp.verbose": False}

# Create solver
solver = ca.qpsol('solver', 'osqp', qp, opts)

# Pre-calc sizes for bounds and indexing
n_dec = int(dec_var.size1() * dec_var.size2())
# U occupies first nu*N entries (vec column-major)
nU = nu * N
nX = nx * (N + 1)

# Input bounds (for U)
umin = -3.0
umax = 3.0

# Build static lbx/ubx arrays (for decision variables order [vec(U); vec(X)])
lbx = np.concatenate((
    np.ones(nU) * umin,        # U lower bounds
    np.ones(nX) * (-1e20)      # X lower bounds (no bounds)
))
ubx = np.concatenate((
    np.ones(nU) * umax,        # U upper bounds
    np.ones(nX) * (1e20)       # X upper bounds (no bounds)
))

# equality constraints bounds (g equality = 0)
lbg = np.zeros(g.size1())
ubg = np.zeros(g.size1())

# ===============================
#  Hàm điều khiển MPC
# ===============================
def mpc_step(x_state, x_ref_val):
    # pack parameter vector p = [x0; x_ref]
    p = np.concatenate((x_state, x_ref_val))

    # initial guess for decision variables (zeros)
    x0_dec = np.zeros(n_dec)

    # call solver
    sol = solver(x0=x0_dec, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, p=p)

    sol_x = np.array(sol['x']).flatten()
    # First nu entries correspond to U(:,0) (since we used vec(U) col-major)
    u0 = sol_x[0:nu]
    return u0


# ===============================
#  Chạy chính
# ===============================

# --- Chuyển sang Torque control ---
ser.write(b"w axis0.controller.config.control_mode 1\n")
ser.write(b"w axis1.controller.config.control_mode 1\n")


# --- Chuyển sang v ---
# ser.write(b"w axis0.controller.config.control_mode 3\n")
# ser.write(b"w axis1.controller.config.control_mode 3\n")
time.sleep(0.1)

# --- Bật closed-loop control ---
ser.write(b"w axis0.requested_state 8\n")
ser.write(b"w axis1.requested_state 8\n")
time.sleep(0.1)

init_mpu()
phi = 0.0
print("Fast MPC Controller running at 100 Hz...")

try:
    t_prev = time.time()
    while True:
        # wait until next sample
        while True:
            t_now = time.time()
            if t_now - t_prev >= dt:
                break
        dt_loop = t_now - t_prev
        # print(dt_loop)
        t_prev = t_now

        # Đọc góc nghiêng
        phi, dphi = read_mpu(phi)

        # Đọc vận tốc bánh
        vL = read_velocity(0) * TWO_PI * r
        vR = read_velocity(1) * TWO_PI * r
        

        v_meas = (vL - vR) / 2.0
        yaw_rate = (r / L) * (vR + vL)

        # Mục tiêu
        v_target = 0.0
        yaw_target = 0.0

        # Trạng thái & tham chiếu
        x_state = np.array([phi, dphi, v_meas, yaw_rate])
        x_ref_val = np.array([-0.02, 0.0, v_target, yaw_target])

        # Giải MPC
        u_cmd = mpc_step(x_state, x_ref_val)   # returns nu-vector
        # u_cmd is [uR, uL] as per B definition ordering
        # In our B we used columns [col0 control, col1 control] corresponding to inputs
        # We'll treat u_cmd[0] -> vR_cmd, u_cmd[1] -> vL_cmd
        vR_cmd = float(u_cmd[0])
        vL_cmd = float(u_cmd[1])

        # Gửi lệnh tới ODrive
        # set_velocity(0, -vL_cmd)
        # set_velocity(1, vR_cmd)

        set_torque(0, -vL_cmd)
        set_torque(1, vR_cmd)

        # Debug timing
        print(f"Loop time: {dt_loop*1000:.2f} ms, phi={phi:.4f}, vR={vR_cmd:.3f}, vL={vL_cmd:.3f}")

except KeyboardInterrupt:
    # set_velocity(0, 0)
    # set_velocity(1, 0)

    set_torque(0, 0)
    set_torque(1, 0)
    print("\nStopped by user.")
except Exception as e:
    print("Error:", e)
    set_velocity(0, 0)
    set_velocity(1, 0)





#------------------------------- key ------------------------------------------


#!/usr/bin/env python3
# ==================================================
#  Fast MPC for 2-Wheel Self-Balancing Robot
#  Raspberry Pi 4 → ODrive V3.6 (UART)
#  Linear QP MPC, 100 Hz update rate
#  Fixed: include x0 & x_ref as parameters; enforce X[:,0]==x0
# ==================================================

#!/usr/bin/env python3
# ==================================================
#  Fast MPC for 2-Wheel Self-Balancing Robot
#  Raspberry Pi 4 → ODrive V3.6 (UART)
#  Linear QP MPC, 100 Hz update rate
#  Fixed: include x0 & x_ref as parameters; enforce X[:,0]==x0
# ==================================================

import casadi as ca
import numpy as np
import time
import serial
from smbus2 import SMBus
import math
import sys, tty, termios, threading
TWO_PI = 2.0 * math.pi

# ===============================
#  Thông số hệ thống vật lý
# ===============================
dt = 0.01        # 100 Hz
N = 10          # MPC horizon
g = 9.81
l = 0.2         # khoảng cách tâm khối lượng → trục bánh
m = 3.0          # khối lượng robot (kg)
r = 0.08255        # bán kính bánh xe (m)
L = 0.40         # khoảng cách giữa 2 bánh xe (m)

# ===============================
#  Cảm biến MPU6050
# ===============================
MPU_ADDR = 0x68
bus = SMBus(1)

def init_mpu():
    bus.write_byte_data(MPU_ADDR, 0x6B, 0)  # bật cảm biến

def read_mpu(phi_prev, alpha=0.98):
    data = bus.read_i2c_block_data(MPU_ADDR, 0x3B, 14)
    def bytes_to_int16(high, low):
        val = (high << 8) | low
        if val > 32767:
            val -= 65536
        return val

    accX = bytes_to_int16(data[0], data[1])
    accY = bytes_to_int16(data[2], data[3])
    accZ = bytes_to_int16(data[4], data[5])
    gyroX = bytes_to_int16(data[8], data[9])

    # chuyển về đơn vị SI / chuẩn sensor scale
    accY_f = accY / 16384.0
    accZ_f = accZ / 16384.0
    gyroX_f = gyroX / 131.0  # °/s

    phi_acc = math.atan2(accY_f, accZ_f)
    dphi = gyroX_f * math.pi / 180.0  # rad/s

    # bộ lọc bổ sung (complementary)
    phi = alpha * (phi_prev + dphi * dt) + (1 - alpha) * phi_acc
    return phi, dphi

# ===============================
#  UART tới ODrive
# ===============================
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.01)

def set_velocity(axis, velocity):
    cmd = f"v {axis} {velocity:.5f}\n".encode()
    ser.write(cmd)

def set_torque(axis, torque):
    cmd = f"c {axis} {torque:.4f}\n".encode()
    ser.write(cmd)


def read_velocity(axis):
    try:
        ser.reset_input_buffer()
    except:
        pass
    ser.write(f"r axis{axis}.encoder.vel_estimate\n".encode())
    line = ser.readline().decode(errors='ignore').strip()
    try:
        return float(line)
    except:
        return 0.0

# ===============================
#  Mô hình tuyến tính hóa (discrete)
# ===============================
A = np.array([
    [1, dt, 0, 0],
    [(g/l)*dt, 1, 0, 0],
    [0, 0, 1, dt],
    [0, 0, 0, 1]
])
B = np.array([
    [0, 0],
    [dt/(2*l), dt/(2*l)],
    [r*dt/2, r*dt/2],
    [r*dt/L, -r*dt/L]
])

# ===============================
#  Thiết lập QP MPC
# ===============================
nx = 4
nu = 2

Q = np.diag([80.0, 1.0, 220, 10.0])
R = np.diag([0.21, 0.16])

# build stage cost as CasADi expressions
X_sym = ca.SX.sym('Xsym', nx)
U_sym = ca.SX.sym('Usym', nu)
Xref_sym = ca.SX.sym('Xref', nx)
stage_cost = ca.mtimes([(X_sym - Xref_sym).T, Q, (X_sym - Xref_sym)]) + ca.mtimes([U_sym.T, R, U_sym])
stage_cost_fun = ca.Function('stage_cost_fun', [X_sym, U_sym, Xref_sym], [stage_cost])

# Decision variables: U (nu x N) and X (nx x (N+1))
U = ca.SX.sym('U', nu, N)
X = ca.SX.sym('X', nx, N + 1)

# Parameters: initial state x0 and reference x_ref
x0 = ca.SX.sym('x0', nx)
x_ref = ca.SX.sym('x_ref', nx)

# Build constraints and cost
g = []
obj = 0

# Constraint: X[:,0] == x0
g.append(X[:, 0] - x0)

for k in range(N):
    # dynamics constraint
    x_next = ca.mtimes(A, X[:, k]) + ca.mtimes(B, U[:, k])
    g.append(X[:, k + 1] - x_next)
    # stage cost with reference x_ref
    obj += stage_cost_fun(X[:, k], U[:, k], x_ref)

# Stack constraints
g = ca.vertcat(*g)

# Decision vector (stacked)
dec_var = ca.vertcat(ca.vec(U), ca.vec(X))

# Build QP data structure for qpsol
# qpsol expects 'x','f','g' and optional 'p' (parameters)
qp = {
    'x': dec_var,
    'f': obj,
    'g': g,
    'p': ca.vertcat(x0, x_ref)
}

# Solver options
opts = {"print_time": 0, "osqp.verbose": False}

# Create solver
solver = ca.qpsol('solver', 'osqp', qp, opts)

# Pre-calc sizes for bounds and indexing
n_dec = int(dec_var.size1() * dec_var.size2())
# U occupies first nu*N entries (vec column-major)
nU = nu * N
nX = nx * (N + 1)

# Input bounds (for U)
umin = -3.0
umax = 3.0

# Build static lbx/ubx arrays (for decision variables order [vec(U); vec(X)])
lbx = np.concatenate((
    np.ones(nU) * umin,        # U lower bounds
    np.ones(nX) * (-1e20)      # X lower bounds (no bounds)
))
ubx = np.concatenate((
    np.ones(nU) * umax,        # U upper bounds
    np.ones(nX) * (1e20)       # X upper bounds (no bounds)
))

# equality constraints bounds (g equality = 0)
lbg = np.zeros(g.size1())
ubg = np.zeros(g.size1())

# ===============================
#  Hàm điều khiển MPC
# ===============================
def mpc_step(x_state, x_ref_val):
    # pack parameter vector p = [x0; x_ref]
    p = np.concatenate((x_state, x_ref_val))

    # initial guess for decision variables (zeros)
    x0_dec = np.zeros(n_dec)

    # call solver
    sol = solver(x0=x0_dec, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, p=p)

    sol_x = np.array(sol['x']).flatten()
    # First nu entries correspond to U(:,0) (since we used vec(U) col-major)
    u0 = sol_x[0:nu]
    return u0


# ===============================
#  Keyboard Control Thread
# ===============================
v_target = 0.0
yaw_target = 0.0
v_step = 0.1
yaw_step = 0.1
v_max = 0.4
yaw_max = math.pi

def keyboard_thread():
    global v_target, yaw_target
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        while True:
            key = sys.stdin.read(1)
            if key == 'w':
                v_target = min(v_target + v_step, v_max)
            elif key == 's':
                v_target = max(v_target - v_step, -v_max)
            elif key == 'a':
                yaw_target = max(yaw_target - yaw_step, -yaw_max)
            elif key == 'd':
                yaw_target = min(yaw_target + yaw_step, yaw_max)
            elif key == ' ':
                v_target = 0.0
                # yaw_target = 0.0
            elif key == 'q':
                raise KeyboardInterrupt
            # print(f"[KEY] v={v_target:.2f} m/s, yaw={yaw_target:.2f} rad/s")
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ===============================
#  Chạy chính
# ===============================

# --- Chuyển sang Torque control ---
# ser.write(b"w axis0.controller.config.control_mode 1\n")
# ser.write(b"w axis1.controller.config.control_mode 1\n")


# --- Chuyển sang v ---
ser.write(b"w axis0.controller.config.control_mode 3\n")
ser.write(b"w axis1.controller.config.control_mode 3\n")
time.sleep(0.1)

# --- Bật closed-loop control ---
ser.write(b"w axis0.requested_state 8\n")
ser.write(b"w axis1.requested_state 8\n")
time.sleep(0.1)

init_mpu()
phi = 0.0
print("Fast MPC Controller running at 100 Hz...")
# Chạy thread bàn phím song song
threading.Thread(target=keyboard_thread, daemon=True).start()

try:
    t_prev = time.time()
    yaw_rate = 0.0
    while True:
        # wait until next sample
        while True:
            t_now = time.time()
            if t_now - t_prev >= dt:
                break
        dt_loop = t_now - t_prev
        # print(dt_loop)
        t_prev = t_now

        # Đọc góc nghiêng
        phi, dphi = read_mpu(phi)

        # Đọc vận tốc bánh
        vL = read_velocity(0) * TWO_PI * r
        vR = read_velocity(1) * TWO_PI * r
        

        v_meas = (vL - vR) / 2.0
        # yaw_rate = ((r / L) * (vL - vR))
        yaw_rate = yaw_rate + (vL + vR)/L *dt_loop

        # Mục tiêu
        # v_target = 0.0
        # yaw_target = 0.0

        # Trạng thái & tham chiếu
        x_state = np.array([phi, dphi, v_meas, yaw_rate])
        x_ref_val = np.array([-0.06, 0.0, v_target, yaw_target])

        # Giải MPC
        u_cmd = mpc_step(x_state, x_ref_val)   # returns nu-vector
        # u_cmd is [uR, uL] as per B definition ordering
        # In our B we used columns [col0 control, col1 control] corresponding to inputs
        # We'll treat u_cmd[0] -> vR_cmd, u_cmd[1] -> vL_cmd
        vR_cmd = float(u_cmd[0])*1.0
        vL_cmd = float(u_cmd[1])*1.0

        # Gửi lệnh tới ODrive
        set_velocity(0, -vL_cmd)
        set_velocity(1, vR_cmd)

        # set_torque(0, -vL_cmd)
        # set_torque(1, vR_cmd)

        # Debug timing
        # print(f"Loop time: {dt_loop*1000:.2f} ms, phi={phi:.4f}, vR={vR_cmd:.3f}, vL={vL_cmd:.3f}")
        print(f"Loop time: {dt_loop*1000:.2f} ms, phi={phi:.4f}, yaw={yaw_rate:.3f}")

except KeyboardInterrupt:
    set_velocity(0, 0)
    set_velocity(1, 0)

    # set_torque(0, 0)
    # set_torque(1, 0)
    print("\nStopped by user.")
except Exception as e:
    print("Error:", e)
    set_velocity(0, 0)
    set_velocity(1, 0)

