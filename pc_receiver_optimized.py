# pc_receiver_optimized.py
# Phiên bản tối ưu của pc_receiver.py:
# - Khởi tạo bài toán MPC 1 lần ngoài vòng lặp (OpTI) và reuse solver (warm-start)
# - Giảm prediction horizon, cho phép khởi tạo nghiệm trước
# - Tách hiển thị ảnh sang 1 thread riêng để giảm blocking I/O
# - Tối giản xử lý ảnh (lấy contour lớn nhất)
# - Bắt lỗi solver và dùng nghiệm dự phòng khi cần

import socket
import cv2
import pickle
import struct
import numpy as np
import math
import casadi as ca
import time
import threading
from collections import deque

# ---------------------------
# CẤU HÌNH MPC (tạo 1 lần)
# ---------------------------
def build_mpc(N=12, dt=0.1, L=0.25):
    opti = ca.Opti()
    X = opti.variable(4, N+1)  # [x, y, theta, v]
    U = opti.variable(2, N)    # [a, delta]
    a     = U[0, :]
    delta = U[1, :]
    x0 = opti.parameter(4)
    ref = opti.parameter(4)
    max_d_delta = np.deg2rad(15)
    max_a = 0.1
    # dynamics
    for k in range(N-1):
        x_next = X[0,k] + dt * X[3,k] * ca.cos(X[2,k])
        y_next = X[1,k] + dt * X[3,k] * ca.sin(X[2,k])
        theta_next = X[2,k] + dt * X[3,k] / L * ca.tan(U[1,k])
        v_next = X[3,k] + dt * U[0,k]
        opti.subject_to(X[:,k+1] == ca.vertcat(x_next, y_next, theta_next, v_next))
        opti.subject_to(opti.bounded(-max_d_delta,delta[k+1] - delta[k],max_d_delta))           # giới hạn thay đổi góc lái mỗi bước
        opti.subject_to(opti.bounded(-max_a,a[k+1] - a[k],max_a))                       # giới hạn thay đổi gia tốc mỗi bước
    # constraints
    opti.subject_to(X[:,0] == x0)
    opti.subject_to(opti.bounded(-1.0, U[0,:], 1.0))
    opti.subject_to(opti.bounded(-0.5, U[1,:], 0.5))

    # cost
    Q = np.diag([10, 10, 5, 1])
    R = np.diag([0.1, 0.1])
    cost = 0
    for k in range(N):
        err = X[:,k] - ref
        cost += ca.mtimes([err.T, Q, err]) + ca.mtimes([U[:,k].T, R, U[:,k]])
    opti.minimize(cost)

    # solver options -- minimize verbosity and enable JIT if available
    try:
        opti.solver('ipopt', {'ipopt':{'print_level':0}, 'print_time':0})
    except Exception:
        # fallback to simple solver call if platform doesn't accept extra args
        opti.solver('ipopt')

    return opti, X, U, x0, ref

# Build MPC once
opti, X_var, U_var, x0_par, ref_par = build_mpc(N=12, dt=0.1, L=0.25)

# store previous solution for warm start
X_prev = np.zeros((4, 12+1))
U_prev = np.zeros((2, 12))

# ---------------------------
# HỖ TRỢ XỬ LÝ ẢNH (nhanh hơn)
# ---------------------------

def mask_yellow_line(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([22, 60, 75])
    upper_yellow = np.array([48, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    return mask

def detect_error_mpc(frame):
    # frame = cv2.resize(frame, (640, 480))
    # x1, y1, x2, y2 = 0, 75, 320, 125
    x1, y1, x2, y2 = 0, 90, 320, 140
    cropped_img = frame[y1:y2, x1:x2]
    # Lọc màu vàng
    mask = mask_yellow_line(cropped_img)
    contours,hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    centrer = []
    previous_e_pos = 0.0
    previous_e_ang = 0.0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area>500:
            # cv2.drawContours(cropped_img, cnt, -1, (0, 255, 0), 3)
            M = cv2.moments(cnt)

            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])  # tọa độ x trọng tâm
                centrer.append(cx)
                cy = int(M["m01"] / M["m00"])  # tọa độ y trọng tâm
                centrer.append(cy)
                cv2.circle(cropped_img, (cx, cy), 5, (255, 0, 0), -1)
                cv2.putText(cropped_img, f"Line Center: ({cx}, {cy})", (cx+10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
    if len(centrer) ==4: # kiểm tra nếu có nhiều hơn 2 tọa độ trọng tâm thì không tính
        x_tb = (centrer[0] - centrer[2])/2 + centrer[2] + x1
        y_tb = centrer[1] + y1
        # print (x_tb, y_tb)
        cv2.circle(frame,(int(x_tb), int(y_tb)), 5, (255, 0, 0), -1)
        cv2.line(frame, (160, 0), (160, 240), (0,0,255),3)
        cv2.line(frame, (int(x_tb), int(y_tb)), (160, 240), (0,255,0),3)

        if x_tb > 160:
            error_x = x_tb - 160
            # angle_rad = np.arctan2(error_x, 145)
            angle_rad = np.arctan2(error_x, 130)
        else:
            error_x = 160 -x_tb
            # angle_rad = np.arctan2((x_tb -160), 145)
            angle_rad = np.arctan2((x_tb -160), 130)
        # if abs(angle_rad) < 0.01:
        #     angle_rad = 0.0

        # angle_deg = math.degrees(angle_rad)
        # error_x_m = 0.33/(centrer[0] - centrer[2])*error_x
        error_x_m = 0.0022449*error_x

        previous_e_pos = error_x_m
        previous_e_ang = angle_rad

        return error_x_m, angle_rad, frame
    else: 
        return previous_e_pos, previous_e_ang, frame
'''    
def detect_error_mpc(frame):
    # Chỉ crop ROI nhỏ và lấy contour lớn nhất
    x1, y1, x2, y2 = 0, 75, 320, 125
    cropped_img = frame[y1:y2, x1:x2]
    mask = mask_yellow_line(cropped_img)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return 0.0, 0.0, frame

    # chọn contour lớn nhất
    cnt = max(contours, key=cv2.contourArea)
    if cv2.contourArea(cnt) < 500:
        return 0.0, 0.0, frame

    M = cv2.moments(cnt)
    if M['m00'] == 0:
        return 0.0, 0.0, frame

    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])

    x_tb = cx + x1
    y_tb = cy + y1

    # tính lỗi ngang và góc đơn giản
    error_x = x_tb - 160
    angle_rad = math.atan2(error_x, 145)
    error_x_m = 0.0022449 * abs(error_x)

    # vẽ nhẹ nhàng (chỉ circle + line) - nếu bạn muốn tối đa tốc, tắt phần vẽ
    cv2.circle(frame,(int(x_tb), int(y_tb)), 4, (255, 0, 0), -1)
    cv2.line(frame, (160, 0), (160, 240), (0,0,255),1)
    cv2.line(frame, (int(x_tb), int(y_tb)), (160, 240), (0,255,0),1)

    return error_x_m, angle_rad, frame
'''

# ---------------------------
# HÀM MPC (chỉ set param + warm-start)
# ---------------------------

def mpc_solve_fast(e_pos, e_ang, v0, X_init=None, U_init=None):
    # compute target in world frame
    target_x = e_pos * math.cos(e_ang)
    target_y = e_pos * math.sin(e_ang)
    # prevent degenerate target
    ang = math.atan2(target_y, target_x) if not (abs(target_x) < 1e-6 and abs(target_y) < 1e-6) else 0.0

    # set parameters
    opti.set_value(x0_par, [0.0, 0.0, 0.0, float(v0)])
    opti.set_value(ref_par, [float(target_x), float(target_y), float(ang), 1.0])

    # warm start
    if X_init is not None:
        try:
            opti.set_initial(X_var, X_init)
        except Exception:
            pass
    if U_init is not None:
        try:
            opti.set_initial(U_var, U_init)
        except Exception:
            pass

    # solve with timeout guard
    try:
        sol = opti.solve()
        a0, delta0 = sol.value(U_var[:,0])
        X_sol = sol.value(X_var)
        U_sol = sol.value(U_var)
        # return float(a0), float(delta0), X_sol, U_sol
        return float(a0), float(delta0), X_sol, U_sol
    except Exception as e:
        # nếu solver fail, trả nghiệm dự phòng (0 control)
        # và reuse previous solution
        print('MPC solve failed:', e)
        if U_prev is not None:
            return float(U_prev[0,0]), float(U_prev[1,0]), X_prev, U_prev
        else:
            return 0.0, 0.0, X_prev, U_prev

# ---------------------------
# Hiển thị frame trong thread riêng
# ---------------------------

display_queue = deque(maxlen=1)
stop_display = False

def display_thread_func():
    global stop_display
    while not stop_display:
        if display_queue:
            frame = display_queue[-1]
            cv2.imshow('Receiving', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_display = True
                break
        else:
            time.sleep(0.005)

# Khởi chạy thread hiển thị
disp_thread = threading.Thread(target=display_thread_func, daemon=True)
disp_thread.start()

# ---------------------------
# Socket server (giữ logic của bạn, nhưng gọi mpc_solve_fast)
# ---------------------------
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(('0.0.0.0', 8485))
server_socket.listen(1)
print('Listening on port 8485...')
conn, addr = server_socket.accept()
print('Connected by', addr)

data = b""
payload_size = struct.calcsize("Q")

try:
    while True:
        while len(data) < payload_size:
            packet = conn.recv(4*1024)
            if not packet:
                raise ConnectionError('Socket closed')
            data += packet
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack("Q", packed_msg_size)[0]

        while len(data) < msg_size:
            packet = conn.recv(4*1024)
            if not packet:
                raise ConnectionError('Socket closed')
            data += packet

        frame_data = data[:msg_size]
        data = data[msg_size:]
        frame, velocity = pickle.loads(frame_data)

        cv2.putText(frame, f"Velocity: {velocity:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        e_pos, e_ang, frame = detect_error_mpc(frame)
        # print(e_pos, e_ang)

        # gọi MPC (warm start bằng nghiệm trước nếu có)
        a, delta, X_sol, U_sol = mpc_solve_fast(e_pos, e_ang, velocity, X_init=X_prev, U_init=U_prev)
        print(a,delta)

        # update previous solution shifted by one step for next warm start
        try:
            X_prev = np.hstack((X_sol[:,1:], X_sol[:,-1:].copy()))
            U_prev = np.hstack((U_sol[:,1:], U_sol[:,-1:].copy()))
        except Exception:
            # nếu X_sol/U_sol không hợp lệ, giữ nguyên prev
            pass

        # Vẽ path dự đoán (nhanh)
        X_sol = np.array(X_sol, ndmin=2)
        for i in range(min(X_sol.shape[1], 20)-1):
            x_hoz = int(160 + X_sol[1,i] * 435)
            y_hoz = int(240 - X_sol[0,i] * 435)
            cv2.circle(frame, (x_hoz, y_hoz), 2, (255, 250, 10), -1)

        # for i in range(len(X_sol)- 1): 
        #     x_hoz, y_hoz = world_to_pixel(X_sol[i], y_pred[i])
        #     cv2.circle(frame,(int(x_hoz), int(y_hoz)), 5, (255, 0, 0), -1)

        # vẽ heading
        robot_heading = (int(160 + 50*math.sin(delta)), int(240 - 50*math.cos(delta)))
        cv2.arrowedLine(frame, (160, 240), robot_heading, (255, 250, 10), 2)

        # đẩy frame vào queue hiển thị
        display_queue.append(frame)

        # gửi control về client
        a_f32 = float(round(a, 4))
        delta_f32 = float(round(delta, 4))
        conn.sendall(struct.pack('ff', a_f32, delta_f32))

        # sleep nhẹ (thời gian vòng lặp chính)
        time.sleep(0.05)

except Exception as e:
    print('Main loop ended:', e)
finally:
    stop_display = True
    disp_thread.join(timeout=1.0)
    try:
        conn.close()
    except:
        pass
    try:
        server_socket.close()
    except:
        pass
    cv2.destroyAllWindows()
