import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

# Tham số mô hình
L = 0.25       # chiều dài trục xe
N = 20         # prediction horizon
dt = 0.1       # bước thời gian
v0 = 0.0      # tốc độ ban đầu

# e_pos = 0.0465
# e_ang = 0.1371


# Tạo bài toán tối ưu
opti = ca.Opti()

X = opti.variable(4, N+1)  # [x, y, theta, v]
U = opti.variable(2, N)    # [a, delta]

x0 = opti.parameter(4)     # trạng thái ban đầu
ref = opti.parameter(4)    # ref lý tưởng


# rhs = ca.vertcat(
#     v * ca.cos(theta),
#     v * ca.sin(theta),
#     v / L * ca.tan(delta),
#     a
# )

# Mô hình động học xe đạp
for k in range(N):
    x_next = X[0,k] + dt * X[3,k] * ca.cos(X[2,k])
    y_next = X[1,k] + dt * X[3,k] * ca.sin(X[2,k])
    theta_next = X[2,k] + dt * X[3,k] / L * ca.tan(U[1,k])
    v_next = X[3,k] + dt * U[0,k]
    
    opti.subject_to(X[0,k+1] == x_next)
    opti.subject_to(X[1,k+1] == y_next)
    opti.subject_to(X[2,k+1] == theta_next)
    opti.subject_to(X[3,k+1] == v_next)

# Ràng buộc đầu
opti.subject_to(X[:,0] == x0)

# Ràng buộc đầu vào
opti.subject_to(opti.bounded(-1.0, U[0,:], 1.0))     # giới hạn gia tốc
opti.subject_to(opti.bounded(-0.7, U[1,:], 0.7))     # giới hạn góc lái

# Hàm mục tiêu
Q = np.diag([20, 20, 10, 1]) # X, Y, DELTA, Van toc
R = np.diag([0.1, 0.1])      # gia toc, goc lai
cost = 0
for k in range(N):
    err = X[:,k] - ref
    cost += ca.mtimes([err.T, Q, err]) + ca.mtimes([U[:,k].T, R, U[:,k]])
opti.minimize(cost)


# Gán giá trị
e_pos = 1.5  # 1.5 mét phía trước bên trái
e_ang = -0.0314


target_x = e_pos * np.cos(e_ang)
target_y = e_pos * np.sin(e_ang)



print(np.atan2(target_x, target_y))
opti.set_value(x0, [0, 0, 0, v0])
opti.set_value(ref, [target_x, target_y, np.arctan2(target_y, target_x), 1])

# Giải
opti.solver('ipopt')
sol = opti.solve()

# Kết quả điều khiển đầu ra
a, delta = sol.value(U[:,0])
for i in range(10):
    print(sol.value(U[:,i]))
# print("a =", a, "delta =", delta

# print(" gia tri value tat ca")
# print(sol.value(X[0, :]))
# Lấy kết quả trạng thái dự đoán
x_pred = sol.value(X[0, :])
y_pred = sol.value(X[1, :])
theta_pred = sol.value(X[2, :])

print("x_pred")
print(x_pred)
print("y_pred")
print(y_pred)


# Vẽ quỹ đạo dự đoán
plt.figure(figsize=(8, 6))
plt.plot(x_pred, y_pred, 'b-o', label='Quỹ đạo dự đoán')
plt.plot(target_x, target_y, 'rx', markersize=12, label='Điểm đích (ref)')
plt.plot(0, 0, 'go', label='Vị trí bắt đầu')

# Vẽ hướng xe tại mỗi điểm
for i in range(0, len(x_pred), 1):
    dx = 0.1 * np.cos(theta_pred[i])
    dy = 0.1 * np.sin(theta_pred[i])
    plt.arrow(x_pred[i], y_pred[i], dx, dy, head_width=0.05, color='gray')

plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.title('Quỹ đạo xe dự đoán từ MPC')
plt.show()