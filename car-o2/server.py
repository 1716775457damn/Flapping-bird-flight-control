# 导入所需模块
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
from queue import Queue, Empty, Full
import torch.nn as nn
import torch.optim as optim
from collections import deque
import atexit
import sys
import signal
import psutil

# 初始化GPU
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

# 配置日志
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# 初始化Flask应用
app = Flask(__name__, static_folder='static')
CORS(app, resources={r"/*": {"origins": "*"}}, supports_credentials=True)

class KalmanFilter:
    def __init__(self, process_variance=1e-4, measurement_variance=1e-2):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = 0.0
        self.estimate_error = 1.0
        
    def update(self, measurement):
        prediction_error = self.estimate_error + self.process_variance
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.estimate_error = (1 - kalman_gain) * prediction_error
        return self.estimate

class MotionPredictor(nn.Module):
    def __init__(self, input_size=9, hidden_size=32, num_layers=2, output_size=9):
        super(MotionPredictor, self).__init__()
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        
        self.lstm = nn.LSTM(input_size, hidden_size, num_layers, batch_first=True)
        self.fc = nn.Linear(hidden_size, output_size)
        
    def forward(self, x):
        h0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to(x.device)
        c0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to(x.device)
        
        out, _ = self.lstm(x, (h0, c0))
        out = self.fc(out[:, -1, :])
        return out

class MPUProcessor:
    def __init__(self):
        self.last_time = time.time()
        self.data_lock = threading.Lock()
        self.last_processed_data = None
        self.connection_status = False
        self.last_update_time = time.time()
        self.update_timeout = 1.0  # 1秒超时
        
        # 初始化卡尔曼滤波器
        self.kalman_filters = {
            'ax': KalmanFilter(process_variance=1e-5, measurement_variance=1e-2),
            'ay': KalmanFilter(process_variance=1e-5, measurement_variance=1e-2),
            'az': KalmanFilter(process_variance=1e-5, measurement_variance=1e-2),
            'roll': KalmanFilter(process_variance=1e-5, measurement_variance=1e-2),
            'pitch': KalmanFilter(process_variance=1e-5, measurement_variance=1e-2),
            'yaw': KalmanFilter(process_variance=1e-5, measurement_variance=1e-2)
        }
        
        logger.info("MPU处理器初始化完成")

    def process_data(self, data):
        try:
            current_time = time.time()
            self.last_time = current_time
            self.last_update_time = current_time

            # 应用卡尔曼滤波
            accel = {
                'x': self.kalman_filters['ax'].update(data['ax']),
                'y': self.kalman_filters['ay'].update(data['ay']),
                'z': self.kalman_filters['az'].update(data['az'])
            }

            # 处理欧拉角数据
            orientation = {
                'x': self.kalman_filters['roll'].update(data['roll']),   # roll
                'y': self.kalman_filters['pitch'].update(data['pitch']), # pitch
                'z': self.kalman_filters['yaw'].update(data['yaw'])      # yaw
            }

            processed_data = {
                'acceleration': accel,
                'gyro': {'x': 0, 'y': 0, 'z': 0},  # 不再使用角速度
                'orientation': orientation,
                'is_static': True  # 静态检测已在ESP32端完成
            }

            with self.data_lock:
                self.last_processed_data = processed_data
                self.connection_status = True

            return processed_data

        except Exception as e:
            logger.error(f"处理MPU数据时出错: {e}", exc_info=True)
            return None

class TCPServer:
    def __init__(self, host='0.0.0.0', port=8080):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.is_running = False
        self.connection_status = False
        self.mpu_processor = MPUProcessor()
        self.data_queue = Queue(maxsize=1000)
        self.last_processed_data = None
        self.data_lock = threading.Lock()
        self.last_data_time = time.time()
        self.data_timeout = 1.0  # 1秒超时
        
        logger.info(f"TCP服务器初始化完成 - 主机: {host}, 端口: {port}")

    def start(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            self.is_running = True
            logger.info(f"TCP服务器启动在 {self.host}:{self.port}")

            while self.is_running:
                try:
                    logger.debug("等待客户端连接...")
                    self.client_socket, self.client_address = self.server_socket.accept()
                    self.client_socket.settimeout(5.0)
                    self.connection_status = True
                    logger.info(f"ESP32已连接: {self.client_address}")
                    self.handle_client()
                except socket.timeout:
                    logger.warning("连接超时")
                    continue
                except Exception as e:
                    logger.error(f"处理客户端连接时出错: {e}", exc_info=True)
                    self.connection_status = False
                    if self.client_socket:
                        self.client_socket.close()
        except Exception as e:
            logger.error(f"TCP服务器启动错误: {e}", exc_info=True)
        finally:
            self.stop()

    def handle_client(self):
        buffer = ""
        while self.is_running and self.client_socket:
            try:
                data = self.client_socket.recv(1024).decode('utf-8')
                if not data:
                    logger.warning("客户端断开连接")
                    self.connection_status = False
                    break

                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self.process_data(line.strip())

            except socket.timeout:
                logger.debug("等待数据超时")
                continue
            except Exception as e:
                logger.error(f"处理数据时出错: {e}", exc_info=True)
                self.connection_status = False
                break

        if self.client_socket:
            self.client_socket.close()
            self.connection_status = False
            logger.info("ESP32断开连接")

    def process_data(self, data):
        try:
            if data.startswith('m:'):
                values = data[2:].split(',')
                if len(values) == 6:
                    raw_data = {
                        'ax': float(values[0]),
                        'ay': float(values[1]),
                        'az': float(values[2]),
                        'roll': float(values[3]),   # 欧拉角 - roll
                        'pitch': float(values[4]),  # 欧拉角 - pitch
                        'yaw': float(values[5])     # 欧拉角 - yaw
                    }
                    
                    processed_data = self.mpu_processor.process_data(raw_data)
                    if processed_data:
                        with self.data_lock:
                            self.last_processed_data = processed_data
                            self.connection_status = True
                            self.last_data_time = time.time()
                    
        except Exception as e:
            logger.error(f"处理MPU数据时出错: {data} - {e}", exc_info=True)

    def get_mpu_data(self):
        try:
            current_time = time.time()
            with self.data_lock:
                # 检查数据是否超时
                if current_time - self.last_data_time > self.data_timeout:
                    self.connection_status = False
                    logger.debug("数据更新超时，设置连接状态为False")

                if self.last_processed_data is None:
                    return {
                        'acceleration': {'x': 0, 'y': 0, 'z': 0},
                        'gyro': {'x': 0, 'y': 0, 'z': 0},
                        'orientation': {'x': 0, 'y': 0, 'z': 0},
                        'connection_status': self.connection_status
                    }
                return {**self.last_processed_data, 'connection_status': self.connection_status}
        except Exception as e:
            logger.error(f"获取MPU数据时出错: {e}", exc_info=True)
            return None

    def get_connection_info(self):
        return {
            'is_running': self.is_running,
            'connection_status': self.connection_status,
            'client_address': self.client_address,
            'queue_size': self.data_queue.qsize()
        }

    def stop(self):
        self.is_running = False
        self.connection_status = False
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()
        logger.info("TCP服务器已停止")

# 全局变量
tcp_server = None

def create_tcp_server():
    """创建并初始化TCP服务器"""
    global tcp_server
    try:
        logger.info("创建TCP服务器实例...")
        tcp_server = TCPServer(host='0.0.0.0', port=8080)
        
        logger.info("启动TCP服务器线程...")
        server_thread = threading.Thread(target=tcp_server.start)
        server_thread.daemon = True
        server_thread.start()
        
        time.sleep(1)
        connection_info = tcp_server.get_connection_info()
        logger.info(f"TCP服务器状态: {connection_info}")
        
        return tcp_server
    except Exception as e:
        logger.error(f"创建TCP服务器失败: {e}", exc_info=True)
        return None

# 初始化TCP服务器
tcp_server = create_tcp_server()

@app.route('/')
def index():
    return send_from_directory('static', 'index.html')

@app.route('/gyro')
def get_gyro_data():
    try:
        if tcp_server is None:
            logger.error("TCP服务器未初始化")
            return jsonify({
                'error': 'TCP server not initialized',
                'connection_status': False,
                'acceleration': {'x': 0, 'y': 0, 'z': 0},
                'gyro': {'x': 0, 'y': 0, 'z': 0},
                'orientation': {'x': 0, 'y': 0, 'z': 0}
            })

        logger.debug(f"收到/gyro请求，TCP服务器状态: {tcp_server.connection_status}, 最后更新时间: {time.time() - tcp_server.last_data_time:.2f}秒前")
        data = tcp_server.get_mpu_data()
        
        if not data:
            logger.warning("无法获取MPU数据")
            return jsonify({
                'error': 'No MPU data available',
                'connection_status': False,
                'acceleration': {'x': 0, 'y': 0, 'z': 0},
                'gyro': {'x': 0, 'y': 0, 'z': 0},
                'orientation': {'x': 0, 'y': 0, 'z': 0}
            })
        
        # 添加CORS头
        response = jsonify(data)
        response.headers.add('Access-Control-Allow-Origin', '*')
        response.headers.add('Access-Control-Allow-Headers', 'Content-Type')
        response.headers.add('Access-Control-Allow-Methods', 'GET')
        
        logger.debug(f"返回数据: {data}")
        return response

    except Exception as e:
        logger.error(f"处理/gyro请求时出错: {str(e)}", exc_info=True)
        return jsonify({
            'error': str(e),
            'connection_status': False,
            'acceleration': {'x': 0, 'y': 0, 'z': 0},
            'gyro': {'x': 0, 'y': 0, 'z': 0},
            'orientation': {'x': 0, 'y': 0, 'z': 0}
        })

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except Exception as e:
        logger.error(f"服务器启动错误: {e}", exc_info=True)
    finally:
        if tcp_server:
            tcp_server.stop()