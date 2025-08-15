import smbus
import time
import struct

bus = smbus.SMBus(1)
address = 0x08

def send_two_floats(f1, f2):
    data = struct.pack('ff', f1, f2)
    byte_list = list(data)
    bus.write_i2c_block_data(address, 0x00, byte_list)  # gửi nguyên 8 byte 1 lần
    # for i in range(len(byte_list)):
    #     bus.write_byte(address, byte_list[i])
def read_float():
    try:
        data = bus.read_i2c_block_data(address, 0, 4)  # đọc 4 byte
        # Chuyển thành float
        float_val = struct.unpack('f', bytes(data))[0]
        return float_val
    except Exception as e:
        print("Read error:", e)
        return None

# def send_one_floats(f1):
#     data = struct.pack('f', f1)
#     byte_list = list(data)
#     # bus.write_i2c_block_data(address, 0x00, byte_list)  # gửi nguyên 8 byte 1 lần
#     for i in range(len(byte_list)):
#         bus.write_byte(0x09, byte_list[i])
     
# while True:

    # send_two_floats(3.14, 0.5)
#     # bus.write_byte(address, byte_list[i])
    # time.sleep(0.5)
#     #print("Sent: 3.14, 2.718")
    # value = read_float()
    # print(value)
    # # send_one_floats(-0.5)
    # time.sleep(0.5)


# sudo python3 /home/ubuntu/Code/interface_I2C.py