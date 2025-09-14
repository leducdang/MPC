import socket
import cv2
import pickle
import struct
import time
import random  # chỉ dùng để mô phỏng vận tốc
from i2cInterface import send_two_floats, read_float

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.0.117', 8485))
# client_socket.connect(('192.168.1.5', 8485))

cam = cv2.VideoCapture(0)

while True:
    ret, frame = cam.read()
    frame = cv2.flip(frame, -1)
    frame = cv2.resize(frame, (320, 240))

    # velocity = random.uniform(-1.0, 1.0)
    # velocity = 0.0
    velocity = read_float()

    data = pickle.dumps((frame, velocity))

    message = struct.pack("Q", len(data)) + data

    client_socket.sendall(message)

    time.sleep(0.075)  # Giới hạn gửi ~20 fps

    
    received = client_socket.recv(8)  # 2 floats = 4 bytes x 2 = 8 bytes
    if len(received) == 8:
        a, theta = struct.unpack('ff', received)
        print(f"Received a = {a:.2f}, theta = {theta:.2f}")
        send_two_floats(a, theta)
        # send_one_floats(theta)
    
    


# sudo python3 /home/ubuntu/Code/TCP_testdata.py
