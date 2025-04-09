import socket
import numpy as np
import cv2
from ultralytics import YOLO
import os
import struct
import time
from collections import defaultdict

print(os.getcwd())
model = YOLO('yolo11s.pt')

# 定义服务器和ESP32地址配置 - 与ESP32代码保持一致
SERVER_IP = "192.168.34.254"       # 服务器IP地址(本机)
SERVER_PORT = 8080                 # 服务器UDP端口
CAMERA_ESP32_IP = "192.168.34.153" # 摄像头ESP32的预期IP
CAMERA_ESP32_PORT = 9090           # 摄像头ESP32的UDP端口
SERVO_ESP32_IP = "192.168.34.154"  # 舵机控制ESP32的预期IP
SERVO_ESP32_PORT = 9091            # 舵机控制ESP32的UDP端口

# 定义包头结构
HEADER_FORMAT = 'III'  # 3个无符号整数
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

class ImageReceiver:
    def __init__(self):
        self.buffer = defaultdict(list)  # 用于存储接收到的包
        self.current_frame = None
        self.last_cleanup = time.time()
        
    def add_packet(self, sequence, total_packets, data):
        self.buffer[sequence] = data
        
        # 定期清理旧的不完整帧
        if time.time() - self.last_cleanup > 5:
            self._cleanup_old_packets()
            self.last_cleanup = time.time()
        
        # 检查是否可以组装完整帧
        if len(self.buffer) == total_packets:
            # 按序号排序并组装数据
            frame_data = bytearray()
            for i in range(total_packets):
                if i in self.buffer:
                    frame_data.extend(self.buffer[i])
            
            # 清空缓冲区
            self.buffer.clear()
            return frame_data
        return None
        
    def _cleanup_old_packets(self):
        # 清理5秒前的包
        current_time = time.time()
        old_sequences = [seq for seq in self.buffer.keys()]
        for seq in old_sequences:
            self.buffer.pop(seq, None)

# 设置UDP接收
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65507)
addr = (SERVER_IP, SERVER_PORT)
s.bind(addr)

# 创建两个UDP发送socket
camera_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
servo_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 两个ESP32的地址
camera_esp32_addr = (CAMERA_ESP32_IP, CAMERA_ESP32_PORT)
servo_esp32_addr = (SERVO_ESP32_IP, SERVO_ESP32_PORT)

# 设置socket超时
s.settimeout(0.5)

# 创建图像接收器
image_receiver = ImageReceiver()

# 创建显示窗口
cv2.namedWindow('Detection', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Detection', 1280, 720)

# 添加性能统计
frame_count = 0
start_time = time.time()
fps = 0

# 添加设备状态
current_led_state = False
current_servo_state = False
last_detection_time = time.time()
DETECTION_TIMEOUT = 3.0  # 3秒无人检测后复位

def send_control_command(socket, state, esp32_addr):
    """发送控制命令到指定ESP32"""
    try:
        command = b'51' if state else b'52'
        socket.sendto(command, esp32_addr)
        return True
    except Exception as e:
        print(f"发送控制命令失败: {str(e)}")
        return False

print("=" * 50)
print("ESP32摄像头图像接收与识别程序")
print("=" * 50)
print(f"本机IP地址和端口: {SERVER_IP}:{SERVER_PORT}")
print(f"摄像头ESP32地址: {CAMERA_ESP32_IP}:{CAMERA_ESP32_PORT}")
print(f"舵机控制ESP32地址: {SERVO_ESP32_IP}:{SERVO_ESP32_PORT}")
print("等待接收图像数据...")
print("-" * 50)

try:
    while True:
        try:
            data, addr = s.recvfrom(65507)
            
            if b'Frame Over' in data:
                continue
                
            # 解析包头
            if len(data) > HEADER_SIZE:
                header = struct.unpack(HEADER_FORMAT, data[:HEADER_SIZE])
                sequence, total_packets, packet_size = header
                
                # 处理图像数据
                image_data = data[HEADER_SIZE:]
                frame_data = image_receiver.add_packet(sequence, total_packets, image_data)
                
                if frame_data:
                    try:
                        # 解码图像
                        receive_data = np.frombuffer(frame_data, dtype='uint8')
                        r_img = cv2.imdecode(receive_data, cv2.IMREAD_COLOR)
                        
                        if r_img is not None:
                            # 更新FPS
                            frame_count += 1
                            if frame_count % 30 == 0:
                                current_time = time.time()
                                fps = 30 / (current_time - start_time)
                                start_time = current_time
                            
                            # 进行目标检测
                            results = model(r_img)
                            
                            # 检查是否检测到人
                            detected_person = False
                            person_confidence = 0.0
                            
                            for result in results:
                                boxes = result.boxes
                                for box in boxes:
                                    if model.names[int(box.cls)] == 'person':
                                        conf = float(box.conf[0])
                                        if conf > person_confidence:
                                            person_confidence = conf
                                        detected_person = True
                            
                            # 控制逻辑
                            current_time = time.time()
                            if detected_person and person_confidence > 0.3:  # 置信度阈值
                                # 控制LED
                                if not current_led_state:
                                    if send_control_command(camera_socket, True, camera_esp32_addr):
                                        current_led_state = True
                                        print(f"检测到人 (置信度: {person_confidence:.2f})，打开LED")
                                
                                # 控制舵机
                                if not current_servo_state:
                                    if send_control_command(servo_socket, True, servo_esp32_addr):
                                        current_servo_state = True
                                        print("控制舵机旋转到180度")
                                        
                                last_detection_time = current_time
                                
                            elif current_time - last_detection_time > DETECTION_TIMEOUT:
                                # 控制LED
                                if current_led_state:
                                    if send_control_command(camera_socket, False, camera_esp32_addr):
                                        current_led_state = False
                                        print("超过3秒未检测到人，关闭LED")
                                
                                # 控制舵机
                                if current_servo_state:
                                    if send_control_command(servo_socket, False, servo_esp32_addr):
                                        current_servo_state = False
                                        print("控制舵机返回0度")
                            
                            # 在图像上绘制检测结果
                            for result in results:
                                boxes = result.boxes
                                for box in boxes:
                                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                                    cls = int(box.cls[0])
                                    conf = float(box.conf[0])
                                    label = f'{model.names[cls]} {conf:.2f}'
                                    
                                    color = (0, 0, 255) if model.names[cls] == 'person' else (0, 255, 0)
                                    
                                    cv2.rectangle(r_img, (x1, y1), (x2, y2), color, 2)
                                    text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                                    cv2.rectangle(r_img, (x1, y1 - 20), (x1 + text_size[0], y1), color, -1)
                                    cv2.putText(r_img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                            
                            # 添加状态显示
                            status_text = f"FPS: {fps:.1f} | LED: {'ON' if current_led_state else 'OFF'} | Servo: {'180°' if current_servo_state else '0°'}"
                            cv2.putText(r_img, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                            
                            # 显示检测结果
                            cv2.imshow('Detection', r_img)
                            
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                print("用户按下Q键，程序退出")
                                break
                                
                    except Exception as e:
                        print(f"处理图像数据时发生错误: {str(e)}")
                        
        except socket.timeout:
            continue
        except Exception as e:
            print(f"发生错误: {str(e)}")
            continue

except KeyboardInterrupt:
    print("\n程序被用户中断")
finally:
    print("清理资源...")
    s.close()
    camera_socket.close()
    servo_socket.close()
    cv2.destroyAllWindows()
    print("程序结束")
