from flask import Flask, jsonify, send_from_directory
from flask_cors import CORS
import socket
import threading
import json
import logging
import time
import os
import math
import numpy as np
import torch
try:
    if torch.cuda.is_available():
        USE_GPU = True
        DEVICE = torch.device('cuda')
        print("GPU加速已启用 - 使用设备:", torch.cuda.get_device_name(0))
    else:
        USE_GPU = False
        DEVICE = torch.device('cpu')
        print("未找到可用的GPU，使用CPU模式")
except Exception as e:
    USE_GPU = False
    DEVICE = torch.device('cpu')
    print(f"GPU初始化失败: {e}，使用CPU模式")

app = Flask(__name__, static_folder='static')
CORS(app)

# 自定义日志过滤器
class DuplicateFilter(logging.Filter):
    def __init__(self, interval=1):
        self.last_log = {}
        self.interval = interval
        
    def filter(self, record):
        current_time = time.time()
        msg = record.getMessage()
        if msg in self.last_log:
            if current_time - self.last_log[msg] < self.interval:
                    return False
        self.last_log[msg] = current_time
        return True

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
logger.addFilter(DuplicateFilter())

# 为Werkzeug日志添加过滤器
werkzeug_logger = logging.getLogger('werkzeug')
werkzeug_logger.addFilter(DuplicateFilter())

class KalmanFilter:
    def __init__(self, process_variance=1e-4, measurement_variance=1e-2):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = 0.0
        self.estimate_error = 1.0
        
    def update(self, measurement):
        # 预测步骤
        prediction_error = self.estimate_error + self.process_variance
        
        # 更新步骤
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.estimate_error = (1 - kalman_gain) * prediction_error
        
        return self.estimate

class MPUProcessor:
    def __init__(self):
        self.last_time = time.time()
        self.velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.orientation = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.last_gyro = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.last_accel = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # 优化：预分配数组
        self.data_buffer = []
        self.buffer_size = 10  # 缓冲区大小
        
        # 卡尔曼滤波器参数优化
        self.kalman_filters = {
            'ax': KalmanFilter(process_variance=1e-4, measurement_variance=1e-2),
            'ay': KalmanFilter(process_variance=1e-4, measurement_variance=1e-2),
            'az': KalmanFilter(process_variance=1e-4, measurement_variance=1e-2),
            'gx': KalmanFilter(process_variance=1e-5, measurement_variance=1e-3),
            'gy': KalmanFilter(process_variance=1e-5, measurement_variance=1e-3),
            'gz': KalmanFilter(process_variance=1e-5, measurement_variance=1e-3)
        }
        
        # 优化滤波器参数
        self.alpha = 0.1  # 降低加速度低通滤波系数
        self.beta = 0.9   # 提高角速度互补滤波系数

    def process_data(self, data):
        current_time = time.time()
        dt = min(current_time - self.last_time, 0.05)  # 限制最大时间步长为50ms
        self.last_time = current_time

        if USE_GPU:
            # GPU加速处理
            return self._process_data_gpu(data, dt)
        else:
            # CPU处理
            return self._process_data_cpu(data, dt)

    def _process_data_gpu(self, data, dt):
        try:
            # 将数据转换为PyTorch张量并移至GPU
            accel = torch.tensor([data['ax'], data['ay'], data['az']], device=DEVICE, dtype=torch.float32)
            gyro = torch.tensor([data['gx'], data['gy'], data['gz']], device=DEVICE, dtype=torch.float32)
            
            # GPU上进行滤波计算
            filtered_accel = torch.zeros_like(accel)
            filtered_gyro = torch.zeros_like(gyro)
            
            # 批量处理滤波
            for i in range(3):
                filtered_accel[i] = self.kalman_filters[f'a{"xyz"[i]}'].update(float(accel[i].cpu()))
                filtered_gyro[i] = self.kalman_filters[f'g{"xyz"[i]}'].update(float(gyro[i].cpu()))
            
            # 更新姿态（在GPU上计算）
            orientation = torch.tensor([self.orientation['x'], self.orientation['y'], self.orientation['z']], 
                                    device=DEVICE, dtype=torch.float32)
            orientation += filtered_gyro * dt
            orientation = ((orientation + 180) % 360) - 180
            
            # 将结果转回CPU
            orientation = orientation.cpu().numpy()
            filtered_accel = filtered_accel.cpu().numpy()
            filtered_gyro = filtered_gyro.cpu().numpy()
            
            # 更新状态
            for i, axis in enumerate(['x', 'y', 'z']):
                self.orientation[axis] = float(orientation[i])
                self.last_accel[axis] = float(filtered_accel[i])
                self.last_gyro[axis] = float(filtered_gyro[i])
            
            return {
                'acceleration': {axis: float(filtered_accel[i]) for i, axis in enumerate(['x', 'y', 'z'])},
                'gyro': {axis: float(filtered_gyro[i]) for i, axis in enumerate(['x', 'y', 'z'])},
                'orientation': self.orientation.copy(),
                'velocity': self.velocity.copy(),
                'position': self.position.copy()
            }
            
        except Exception as e:
            logger.error(f"GPU处理错误: {e}")
            return self._process_data_cpu(data, dt)

    def _process_data_cpu(self, data, dt):
        # 原有的CPU处理逻辑，但进行了优化
        accel = np.array([data['ax'], data['ay'], data['az']])
        gyro = np.array([data['gx'], data['gy'], data['gz']])
        
        # 批量滤波处理
        filtered_accel = np.zeros_like(accel)
        filtered_gyro = np.zeros_like(gyro)
        
        for i, axis in enumerate(['x', 'y', 'z']):
            filtered_accel[i] = self.kalman_filters[f'a{axis}'].update(accel[i])
            filtered_gyro[i] = self.kalman_filters[f'g{axis}'].update(gyro[i])
            
            # 更新姿态
            self.orientation[axis] += filtered_gyro[i] * dt
            self.orientation[axis] = ((self.orientation[axis] + 180) % 360) - 180
            
            # 更新状态
            self.last_accel[axis] = filtered_accel[i]
            self.last_gyro[axis] = filtered_gyro[i]
        
        return {
            'acceleration': {axis: float(filtered_accel[i]) for i, axis in enumerate(['x', 'y', 'z'])},
            'gyro': {axis: float(filtered_gyro[i]) for i, axis in enumerate(['x', 'y', 'z'])},
            'orientation': self.orientation.copy(),
            'velocity': self.velocity.copy(),
            'position': self.position.copy()
        }

class TCPServer:
    def __init__(self, host='0.0.0.0', port=8080):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.is_running = False
        self.mpu_data = {'x': 0, 'y': 0, 'z': 0}
        self.last_update = 0
        self.lock = threading.Lock()
        self.mpu_processor = MPUProcessor()  # 添加MPU处理器

    def start(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self.is_running = True
        logger.info(f"TCP服务器启动在 {self.host}:{self.port}")

        while self.is_running:
            try:
                self.client_socket, self.client_address = self.server_socket.accept()
                logger.info(f"ESP32已连接: {self.client_address}")
                self.handle_client()
            except Exception as e:
                logger.error(f"处理客户连接时出错: {e}")
                if self.client_socket:
                    self.client_socket.close()

    def handle_client(self):
        buffer = ""
        while self.is_running and self.client_socket:
            try:
                data = self.client_socket.recv(1024).decode('utf-8')
                if not data:
                    break

                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self.process_data(line.strip())

            except Exception as e:
                logger.error(f"处理数据时出错: {e}")
                break

        if self.client_socket:
            self.client_socket.close()
            logger.info("ESP32断开连接")

    def process_data(self, data):
        try:
            if data.startswith('m:'):
                values = data[2:].split(',')
                if len(values) == 6:  # 现在我们期望6个值：ax,ay,az,gx,gy,gz
                    with self.lock:
                        raw_data = {
                            'ax': float(values[0]),
                            'ay': float(values[1]),
                            'az': float(values[2]),
                            'gx': float(values[3]),
                            'gy': float(values[4]),
                            'gz': float(values[5])
                        }
                        # 打印接收到的原始数据
                        logger.info(f"接收到MPU数据: {raw_data}")
                        
                        # 处理MPU数据
                        processed_data = self.mpu_processor.process_data(raw_data)
                        self.mpu_data = processed_data
                        self.last_update = time.time()
                        
                        # 打印处理后的数据
                        logger.info(f"处理后的数据: {processed_data}")
                else:
                    logger.warning(f"MPU数据格式错误: {data}")
            else:
                logger.warning(f"未知数据格式: {data}")
        except Exception as e:
            logger.error(f"处理MPU数据时出错: {data} - {e}")

    def get_mpu_data(self):
        with self.lock:
            return self.mpu_data.copy()

    def stop(self):
        self.is_running = False
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()

# 创建TCP服务器实例
tcp_server = TCPServer()

# 在新线程中启动TCP服务器
server_thread = threading.Thread(target=tcp_server.start)
server_thread.daemon = True
server_thread.start()

@app.route('/')
def index():
    return send_from_directory('static', 'index.html')

@app.route('/gyro')
def get_gyro_data():
    if not tcp_server.mpu_data:
        logger.warning("没有可用的MPU数据")
        return jsonify({'error': 'No MPU data available'})
    try:
        data = tcp_server.mpu_data
        # 打印发送到前端的数据
        logger.info(f"发送到前端的数据: {data}")
        return jsonify(data)  # 直接返回已处理的数据
    except Exception as e:
        logger.error(f"获取MPU数据时出错: {e}")
        return jsonify({'error': str(e)})

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=80, debug=False)
    finally:
        tcp_server.stop()