# 扑翼鸟飞行控制系统 (Flapping Bird Flight Control System)

![扑翼鸟飞行控制系统](https://github.com/user/Flapping-bird-flight-control/raw/main/images/bird_image.jpg)

## 项目简介

本项目是一个基于ESP32和Arduino的扑翼鸟飞行控制系统，结合了舵机控制、陀螺仪姿态感应和计算机视觉技术，实现了对扑翼鸟的远程控制、自动平衡和智能目标检测功能。该系统可用于模拟鸟类飞行、机器人研究和教育演示等场景。

## 主要功能

- **无线远程控制**：通过WiFi连接，使用Web界面远程控制扑翼鸟的飞行
- **实时姿态感应**：基于MPU6050陀螺仪，使用卡尔曼滤波算法实现精确的姿态检测
- **自动平衡控制**：使用PID控制算法保持扑翼鸟的飞行稳定性
- **计算机视觉**：集成YOLO目标检测模型，可识别人或其他目标
- **自适应控制**：根据目标检测结果自动调整飞行参数和行为

## 系统架构

该项目包含以下主要组件：

1. **飞行控制模块**（`car/car.ino`）：
   - 基于ESP32微控制器
   - 负责舵机控制、WiFi通信和姿态感应
   - 实现Web控制界面

2. **视觉识别模块**（`can1/`）：
   - 基于YOLO目标检测模型
   - Python处理图像并发送控制命令
   - UDP通信与控制模块交互

3. **控制算法**：
   - PID控制器用于稳定飞行
   - 卡尔曼滤波器用于姿态估计
   - 自定义信号处理算法

## 硬件要求

- ESP32开发板 (推荐使用ESP32-WROOM或ESP32-WROVER)
- MPU6050陀螺仪/加速度计模块
- 舵机 (建议使用高扭矩金属齿舵机)
- 电机和电机驱动器
- 电源系统 (建议使用3.7V锂电池，带有升压模块)
- (可选) ESP32-CAM模块，用于视觉处理

## 软件依赖

- Arduino IDE (1.8.x或更高版本)
- ESP32 Arduino核心库
- Python 3.6+（用于视觉处理模块）
- 以下Arduino库：
  - WiFi.h
  - WiFiClient.h
  - WebServer.h
  - ESPmDNS.h
  - Wire.h
  - MPU6050.h
  - CircularBuffer.hpp

- 以下Python库：
  - opencv-python
  - numpy
  - ultralytics (YOLO)
  - socket
  - struct

## 安装指南

### 飞行控制模块安装

1. 安装Arduino IDE和ESP32核心库
2. 安装所有必要的Arduino库
3. 打开`car/car.ino`文件
4. 选择正确的ESP32开发板和端口
5. 编译并上传代码到ESP32

### 视觉处理模块安装

1. 安装Python 3.6+
2. 安装所需Python库：
   ```
   pip install opencv-python numpy ultralytics
   ```
3. 将`yolo11s.pt`模型文件放置在与`v11.py`相同的目录中
4. 运行Python脚本：
   ```
   python v11.py
   ```

## 使用说明

### 控制界面

1. 将设备连接到ESP32创建的WiFi网络（默认SSID: "xiaocheche", 密码: "12345678"）
2. 在浏览器中访问`http://192.168.4.1`
3. 使用Web界面上的操纵杆控制扑翼鸟：
   - 左侧操纵杆：控制油门/扑翼速度
   - 右侧操纵杆：控制方向舵机/转向

### 视觉识别功能

1. 确保ESP32和运行Python脚本的计算机在同一网络
2. 运行`v11.py`脚本开始视觉识别
3. 当识别到目标（如人）时，系统会自动触发相应的控制动作

## 调试与故障排除

- **WiFi连接问题**：检查WiFi SSID和密码设置
- **舵机控制异常**：确认接线正确，检查舵机PWM参数
- **姿态感应不准确**：校准MPU6050，调整卡尔曼滤波参数
- **控制不稳定**：调整PID控制器参数以获得更平滑的响应

## 项目结构

```
.
├── car/                    # 主控制模块
│   └── car.ino             # ESP32飞行控制代码
├── can1/                   # 视觉识别模块
│   ├── can1.ino            # ESP32-CAM控制代码
│   ├── v11.py              # Python视觉处理代码
│   └── yolo11s.pt          # YOLO模型文件
└── README.md               # 项目文档
```

## 进一步开发

- 添加多模式飞行控制（如悬停、巡航、追踪模式）
- 实现多机协同飞行
- 优化能源效率和飞行时间
- 增强视觉识别系统，支持更多目标类型
- 开发移动应用替代Web控制界面

## 贡献指南

欢迎贡献代码、报告问题或提出改进建议。请参照以下步骤：

1. Fork本仓库
2. 创建您的特性分支：`git checkout -b my-new-feature`
3. 提交您的更改：`git commit -am 'Add some feature'`
4. 推送到分支：`git push origin my-new-feature`
5. 提交Pull Request

## 许可证

本项目采用MIT许可证 - 详见LICENSE文件

## 联系方式

如有问题或建议，请通过以下方式联系项目维护者：

- 电子邮件：1716775457@qq.com
- GitHub Issues: https://github.com/1716775457damn/Flapping-bird-flight-control/issues
