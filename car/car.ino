#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <stdio.h>
#include <Wire.h>
#include <MPU6050.h>
#include <CircularBuffer.hpp>

/**
 * 舵机控制相关
 */
class LedcServo
{
public:
  float freq = 50;
  int resolution = 8;
  float pwmBaseScale;
  float pwmMin;
  float pwmMax;
  int channel;
  int scale = 1;
  void setup(float freq, int resolution, int channel);
  /* 0 < scale <= 1 */
  void setScale(float scale);
  void attachPin(int pin);
  void write(float value, float min, float max);
};
void LedcServo::setup(float f, int r, int c)
{
  this->freq = f;
  this->resolution = r;
  this->pwmBaseScale = this->freq * pow(2, this->resolution) / 1000;
  this->pwmMin = 1 * this->pwmBaseScale;
  this->pwmMax = 2 * this->pwmBaseScale;
  this->channel = c;
  ledcSetup(this->channel, this->freq, this->resolution);
}
void LedcServo::setScale(float s)
{
  if (s <= 0)
    throw "s 不能小于等于0";
  if (s > 1)
    throw "s 不能大于1";

  this->scale = s;
  this->pwmMin = (1.5 - s * 0.5) * this->pwmBaseScale;
  this->pwmMax = (1.5 + s * 0.5) * this->pwmBaseScale;
}
void LedcServo::attachPin(int p)
{
  ledcAttachPin(p, this->channel);
}
void LedcServo::write(float v, float min, float max)
{
  float newV = v;
  if (max > min)
  {
    if (v > max)
      newV = max;
    if (v < min)
      newV = min;
  }
  else
  {
    if (v > min)
      newV = min;
    if (v < max)
      newV = max;
  }
  ledcWrite(this->channel, map(newV, min, max, this->pwmMin, this->pwmMax));
}

float sendMin = -100;
float sendMax = 100;
float sendHaf = 0;

float rxC1 = sendHaf;
float rxC2 = sendHaf;

// 设置为-1
float lastRxC1 = rxC1;
float lastRxC2 = rxC2;

LedcServo rxC1Servo;
LedcServo rxC1ServoHaf;
LedcServo rxC2Servo;
LedcServo rxC2ServoHaf;

/**
 * 接收信息的web server 监听80端口
 */
WebServer server(80);

unsigned long timeNow = 0;
unsigned long lastDataTickTime = 0;

const int LED_PIN = 2;
bool ledShow = false;
int ledLoopTick = -1;

MPU6050 mpu;

float alpha = 0.5; // 滤波系数，0 < alpha < 1
int16_t filteredAx = 0, filteredAy = 0, filteredAz = 0;
int16_t filteredGx = 0, filteredGy = 0, filteredGz = 0;

// 在全局范围声明isHolding变量
bool isHolding = false; // 添加全局变量来跟踪持续状态

// 定义一个简单的信号结构
struct Signal
{
  float throttle;
  float steering;
  unsigned long timestamp;
  bool isHolding;
};

// 创建一个环形缓冲区
CircularBuffer<Signal, 10> signalBuffer; // 存储最近10个信号

// 添加滤波器类
class ValueFilter
{
private:
  float alpha;
  float filteredValue;

public:
  ValueFilter(float alpha = 0.1) : alpha(alpha), filteredValue(0) {}

  float update(float newValue)
  {
    filteredValue = alpha * newValue + (1 - alpha) * filteredValue;
    return filteredValue;
  }

  void reset()
  {
    filteredValue = 0;
  }
};

// 创建滤波器实例
ValueFilter throttleFilter(0.2); // 油门滤波器
ValueFilter steeringFilter(0.2); // 转向滤波器

// 添加信号队列类
class SignalQueue
{
private:
  static const int QUEUE_SIZE = 5; // 减小队列大小
  struct SignalData
  {
    float throttle;
    float steering;
    bool valid;
  };
  SignalData queue[QUEUE_SIZE];
  int writeIndex;
  int readIndex;

public:
  SignalQueue() : writeIndex(0), readIndex(0)
  {
    for (int i = 0; i < QUEUE_SIZE; i++)
    {
      queue[i].valid = false;
    }
  }

  void push(float throttle, float steering)
  {
    queue[writeIndex].throttle = throttle;
    queue[writeIndex].steering = steering;
    queue[writeIndex].valid = true;
    writeIndex = (writeIndex + 1) % QUEUE_SIZE;
    if (writeIndex == readIndex)
    {
      readIndex = (readIndex + 1) % QUEUE_SIZE; // 覆盖最老的数据
    }
  }

  bool pop(float &throttle, float &steering)
  {
    if (readIndex == writeIndex || !queue[readIndex].valid)
    {
      return false;
    }
    throttle = queue[readIndex].throttle;
    steering = queue[readIndex].steering;
    queue[readIndex].valid = false;
    readIndex = (readIndex + 1) % QUEUE_SIZE;
    return true;
  }
};

SignalQueue signalQueue; // 创建信号队列实例

void handleRoot()
{
  String c = server.arg("c");
  float newThrottle, newSteering;
  if (sscanf(c.c_str(), "%f,%f", &newThrottle, &newSteering) == 2)
  {
    signalQueue.push(newThrottle, newSteering);
    lastDataTickTime = millis();
  }
  server.send(200, "text/plain", "success");
}

void handleWebPage()
{
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html lang="zh-cn">
      <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>RC Controller</title>
        <style>
          body { 
            font-family: Arial, sans-serif;
            background-color: #f0f2f5;
            margin: 0;
            padding: 20px;
            color: #333;
          }
          .container {
            max-width: 800px;
            margin: 0 auto;
          }
          .header {
            text-align: center;
            margin-bottom: 30px;
          }
          .header h1 {
            color: #2c3e50;
            margin: 0;
            padding: 20px 0;
          }
          .gyro-panel {
            background: #fff;
            border-radius: 10px;
            padding: 20px;
            margin-bottom: 20px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
          }
          .gyro-data {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 15px;
            text-align: center;
          }
          .gyro-value {
            background: #f8f9fa;
            padding: 15px;
            border-radius: 8px;
            transition: background-color 0.3s;
          }
          .gyro-value.active {
            background: #e3ffe3;
          }
          .gyro-label {
            font-size: 14px;
            color: #666;
            margin-bottom: 5px;
          }
          .gyro-number {
            font-size: 24px;
            font-weight: bold;
            color: #2c3e50;
          }
          .controller-wrap {
            display: flex;
            justify-content: space-between;
            margin-top: 30px;
            gap: 20px;
          }
          .joystick-container {
            background: #fff;
            border-radius: 10px;
            padding: 20px;
            flex: 1;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
          }
          .joystick {
            width: 150px;
            height: 150px;
            background: #eee;
            border-radius: 50%;
            position: relative;
            margin: 0 auto;
            border: 2px solid #ddd;
          }
          .joystick .handle {
            width: 50px;
            height: 50px;
            background: #2c3e50;
            border-radius: 50%;
            position: absolute;
            top: 50px;
            left: 50px;
            cursor: pointer;
            transition: all 0.1s;
            box-shadow: 0 2px 5px rgba(0,0,0,0.2);
          }
          .joystick .handle:hover {
            background: #34495e;
          }
          .joystick-label {
            text-align: center;
            margin-top: 15px;
            font-size: 16px;
            color: #666;
          }
          @media (max-width: 768px) {
            .controller-wrap {
              flex-direction: column;
            }
            .joystick {
              width: 120px;
              height: 120px;
            }
            .joystick .handle {
              width: 40px;
              height: 40px;
              top: 40px;
              left: 40px;
            }
          }
          
          .data-monitor {
            background: #fff;
            border-radius: 10px;
            padding: 20px;
            margin-top: 20px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
          }
          
          .data-row {
            display: flex;
            justify-content: space-between;
            padding: 10px;
            border-bottom: 1px solid #eee;
            font-family: monospace;
          }
          
          .data-row:last-child {
            border-bottom: none;
          }
          
          .data-label {
            color: #666;
          }
          
          .data-value {
            color: #2c3e50;
            font-weight: bold;
          }
        </style>
      </head>
      <body>
        <div class="container">
          <div class="header">
            <h1>RC Controller</h1>
          </div>
          
          <div class="gyro-panel">
            <div class="gyro-data">
              <div class="gyro-value">
                <div class="gyro-label">X Axis</div>
                <div class="gyro-number" id="gyroX">0.00°</div>
              </div>
              <div class="gyro-value">
                <div class="gyro-label">Y Axis</div>
                <div class="gyro-number" id="gyroY">0.00°</div>
              </div>
              <div class="gyro-value">
                <div class="gyro-label">Z Axis</div>
                <div class="gyro-number" id="gyroZ">0.00°</div>
              </div>
            </div>
          </div>

          <div class="controller-wrap">
            <div class="joystick-container">
              <div class="joystick" id="joystick1">
                <div class="handle"></div>
              </div>
              <div class="joystick-label">Throttle</div>
            </div>
            <div class="joystick-container">
              <div class="joystick" id="joystick2">
                <div class="handle"></div>
              </div>
              <div class="joystick-label">Steering</div>
            </div>
          </div>
          
          <div class="data-monitor">
            <div class="data-row">
              <span class="data-label">油门值:</span>
              <span class="data-value" id="throttleValue">0</span>
            </div>
            <div class="data-row">
              <span class="data-label">舵机值:</span>
              <span class="data-value" id="steeringValue">0</span>
            </div>
            <div class="data-row">
              <span class="data-label">MPU6050-X:</span>
              <span class="data-value" id="mpuX">0</span>
            </div>
            <div class="data-row">
              <span class="data-label">MPU6050-Y:</span>
              <span class="data-value" id="mpuY">0</span>
            </div>
            <div class="data-row">
              <span class="data-label">MPU6050-Z:</span>
              <span class="data-value" id="mpuZ">0</span>
            </div>
          </div>
        </div>

        <script>
          const joystick1 = document.getElementById('joystick1');
          const handle1 = joystick1.querySelector('.handle');
          let lastSentValue = 0;
          let isMoving = false;
          
          // 添加触摸事件支持
          joystick1.addEventListener('touchstart', handleStart);
          joystick1.addEventListener('touchmove', handleMove);
          joystick1.addEventListener('touchend', handleEnd);
          joystick1.addEventListener('mousedown', handleStart);
          joystick1.addEventListener('mousemove', handleMove);
          joystick1.addEventListener('mouseup', handleEnd);
          joystick1.addEventListener('mouseleave', handleEnd);

          function handleStart(e) {
            isMoving = true;
            updatePosition(e);
          }

          function handleMove(e) {
            if (!isMoving) return;
            e.preventDefault();
            updatePosition(e);
          }

          function handleEnd() {
            isMoving = false;
            if (!isHolding) {
              handle1.style.top = '100px';
              sendThrottle(-100, false);
            }
          }

          function updatePosition(e) {
            const rect = joystick1.getBoundingClientRect();
            const clientY = e.type.includes('touch') ? e.touches[0].clientY : e.clientY;
            const y = clientY - rect.top - handle1.offsetHeight / 2;
            
            if (y >= 0 && y <= rect.height - handle1.offsetHeight) {
              handle1.style.top = `${y}px`;
              const throttle = map(y, 0, rect.height - handle1.offsetHeight, 100, -100);
              sendThrottle(throttle, isMoving);
            }
          }

          function map(value, inMin, inMax, outMin, outMax) {
            return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
          }

          // 简化发送函数，减少延迟
          function sendThrottle(value, holding) {
            fetch(`/control?c=${value},0&hold=${holding ? 1 : 0}`, {
              method: 'GET',
              cache: 'no-cache',
            }).catch(console.error);
          }

          // 状态指示器
          const statusIndicator = document.createElement('div');
          statusIndicator.style.cssText = `
            position: fixed;
            top: 10px;
            right: 10px;
            width: 10px;
            height: 10px;
            border-radius: 50%;
            background: red;
          `;
          document.body.appendChild(statusIndicator);

          // 检查连接状态
          setInterval(() => {
            fetch('/status')
              .then(response => {
                statusIndicator.style.background = 'green';
              })
              .catch(() => {
                statusIndicator.style.background = 'red';
              });
          }, 1000);

          // 优化MPU数据更新函数
          function updateMPUData() {
              const mpuX = document.getElementById('mpuX');
              const mpuY = document.getElementById('mpuY');
              const mpuZ = document.getElementById('mpuZ');
              
              async function fetchMPUData() {
                  try {
                      const response = await fetch('/gyro', {
                          method: 'GET',
                          cache: 'no-cache',
                          headers: {
                              'Accept': 'application/json'
                          }
                      });
                      
                      if (!response.ok) throw new Error('Network response was not ok');
                      
                      const data = await response.json();
                      mpuX.textContent = `${parseFloat(data.x).toFixed(2)}°`;
                      mpuY.textContent = `${parseFloat(data.y).toFixed(2)}°`;
                      mpuZ.textContent = `${parseFloat(data.z).toFixed(2)}°`;
                      
                      // 调试输出
                      console.log('MPU Data:', data);
                  } catch (error) {
                      console.error('Error fetching MPU data:', error);
                  }
              }
              
              // 立即执行一次
              fetchMPUData();
              
              // 设置更新间隔
              setInterval(fetchMPUData, 100);  // 每100ms更新一次
          }
          
          // 确保在页面加载完成后启动数据更新
          document.addEventListener('DOMContentLoaded', () => {
              console.log('Starting MPU data updates...');
              updateMPUData();
          });

          // 添加右侧遥感控制
          const joystick2 = document.getElementById('joystick2');
          const handle2 = joystick2.querySelector('.handle');
          let lastSentValueSteer = 0;
          let isMovingSteer = false;
          
          // 添加触摸事件支持
          joystick2.addEventListener('touchstart', handleStartSteer);
          joystick2.addEventListener('touchmove', handleMoveSteer);
          joystick2.addEventListener('touchend', handleEndSteer);
          joystick2.addEventListener('mousedown', handleStartSteer);
          joystick2.addEventListener('mousemove', handleMoveSteer);
          joystick2.addEventListener('mouseup', handleEndSteer);
          joystick2.addEventListener('mouseleave', handleEndSteer);

          function handleStartSteer(e) {
            isMovingSteer = true;
            updatePositionSteer(e);
          }

          function handleMoveSteer(e) {
            if (!isMovingSteer) return;
            e.preventDefault();
            updatePositionSteer(e);
          }

          function handleEndSteer() {
            isMovingSteer = false;
            if (!isHolding) {
              handle2.style.left = '50px';
              sendControl(lastSentValue, 0, isMoving, false);
            }
          }

          function updatePositionSteer(e) {
            const rect = joystick2.getBoundingClientRect();
            const clientX = e.type.includes('touch') ? e.touches[0].clientX : e.clientX;
            const x = clientX - rect.left - handle2.offsetWidth / 2;
            
            if (x >= 0 && x <= rect.width - handle2.offsetWidth) {
              handle2.style.left = `${x}px`;
              const steer = map(x, 0, rect.width - handle2.offsetWidth, -100, 100);
              sendControl(lastSentValue, steer, isMoving, isMovingSteer);
            }
          }

          // 修改发送函数以支持两个值
          function sendControl(throttle, steer, holdingThrottle, holdingSteer) {
            fetch(`/control?c=${throttle},${steer}&holdT=${holdingThrottle ? 1 : 0}&holdS=${holdingSteer ? 1 : 0}`, {
              method: 'GET',
              cache: 'no-cache',
            }).catch(console.error);
          }

          // 修改原来的sendThrottle函数
          function sendThrottle(value, holding) {
            sendControl(value, lastSentValueSteer, holding, isMovingSteer);
            lastSentValue = value;
          }

          // 添加CSS样式使手柄可以水平移动
          handle2.style.cssText += `
            left: 50px;
            top: 50% !important;
            transform: translateY(-50%);
          `;
          
          // 更新数据显示
          function updateDataMonitor(throttle, steering) {
            document.getElementById('throttleValue').textContent = throttle.toFixed(2);
            document.getElementById('steeringValue').textContent = steering.toFixed(2);
          }
          
          // 修改sendControl函数
          function sendControl(throttle, steer, holdingThrottle, holdingSteer) {
            const url = `/control?c=${throttle},${steer}&holdT=${holdingThrottle ? 1 : 0}&holdS=${holdingSteer ? 1 : 0}`;
            
            fetch(url, {
              method: 'GET',
              cache: 'no-cache',
            })
            .then(response => response.text())
            .then(data => {
              updateDataMonitor(throttle, steer);
            })
            .catch(console.error);
          }
        </script>
      </body>
    </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void handleGyroData()
{
  // 确保使用最新的波数据
  String json = "{";
  json += "\"x\":" + String(filteredAx, 2) + ",";
  json += "\"y\":" + String(filteredAy, 2) + ",";
  json += "\"z\":" + String(filteredAz, 2);
  json += "}";

  // 添加CORS头和缓存控制
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.send(200, "application/json", json);
}

void handleStatus()
{
  server.send(200, "text/plain", "ok");
}

void registerEvent()
{
  server.on("/", handleWebPage);
  server.on("/control", handleRoot);
  server.on("/gyro", handleGyroData);
  server.on("/status", handleStatus);

  server.enableCORS();
  server.begin();
  Serial.println("HTTP server started");
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Starting setup...");

  Wire.begin(21, 22);
  Serial.println("I2C initialized");

  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  if (mpu.testConnection())
  {
    Serial.println("MPU6050 connection successful");
  }
  else
  {
    Serial.println("MPU6050 connection failed");
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  rxC1Servo.setup(100, 10, 8);
  rxC1Servo.attachPin(23); // 动力电机控制信号
  rxC1Servo.setScale(1);
  rxC1Servo.write(0, -1, 1);

  rxC1ServoHaf.setup(100, 10, 9);
  rxC1ServoHaf.attachPin(22); // 动力电机控制信号（半强度）
  rxC1ServoHaf.setScale(0.3);
  rxC1ServoHaf.write(0, -1, 1);

  rxC2Servo.setup(500, 10, 10);
  rxC2Servo.attachPin(26); // 方向舵机信号
  rxC2Servo.setScale(0.7); // 转向角度限制
  rxC2Servo.write(0, -1, 1);

  rxC2ServoHaf.setup(500, 10, 11);
  rxC2ServoHaf.attachPin(25); // 方向舵机信号（半强度）
  rxC2ServoHaf.setScale(0.3);
  rxC2ServoHaf.write(0, -1, 1);

  Serial.println();
  Serial.print("Configuring access point...");

  // WiFi设置
  WiFi.softAP("xiaocheche", "12345678");
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  if (MDNS.begin("esp32"))
  {
    Serial.println("MDNS responder started");
  }

  registerEvent();
  Serial.println("Setup completed");
}

// 修改updateServo函数
void updateServo()
{
  // 设置死区值为±20
  const float deadzone = 20.0;

  // 油门舵机
  if (abs(lastRxC1 - rxC1) > deadzone)
  {
    // 对于超出死区的值，进行映射处理
    float mappedThrottle = rxC1;
    if (abs(rxC1) <= deadzone)
    {
      mappedThrottle = 0; // 在死区内的值归零
    }
    else if (rxC1 > deadzone)
    {
      // 将大于死区的值映射到0-100范围，并反转方向
      mappedThrottle = map(rxC1, deadzone, 100, 0, -100);
    }
    else
    {
      // 将小于-死区的值映射到-100-0范围，并反转方向
      mappedThrottle = map(rxC1, -100, -deadzone, 100, 0);
    }

    rxC1Servo.write(mappedThrottle, sendMax, sendMin);
    rxC1ServoHaf.write(mappedThrottle, sendMax, sendMin);
    lastRxC1 = rxC1;
  }

  // 转向舵机
  if (abs(lastRxC2 - rxC2) > deadzone)
  {
    // 对于超出死区的值，进行映射处理
    float mappedSteering = rxC2;
    if (abs(rxC2) <= deadzone)
    {
      mappedSteering = 0; // 在死区内的值归零
    }
    else if (rxC2 > deadzone)
    {
      // 将大于死区的值映射到0-100范围
      mappedSteering = map(rxC2, deadzone, 100, 0, 100);
    }
    else
    {
      // 将小于-死区的值映射到-100-0范围
      mappedSteering = map(rxC2, -100, -deadzone, -100, 0);
    }

    rxC2Servo.write(mappedSteering, sendMax, sendMin);
    rxC2ServoHaf.write(mappedSteering, sendMax, sendMin);
    lastRxC2 = rxC2;
  }
}

// 辅助函数：浮点数映射
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// 添加MPU6050处理相关代码
class KalmanFilter
{
private:
  float Q_angle;
  float Q_bias;
  float R_measure;
  float angle;
  float bias;
  float P[2][2];

public:
  KalmanFilter()
  {
    Q_angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;
    angle = 0.0f;
    bias = 0.0f;
    P[0][0] = 0.0f;
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
  }

  float update(float newAngle, float newRate, float dt)
  {
    angle += dt * (newRate - bias);
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    float y = newAngle - angle;
    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    angle += K[0] * y;
    bias += K[1] * y;
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
  }
};

// 创建滤波器实例
KalmanFilter kalmanX, kalmanY, kalmanZ;
float lastMPUTime = 0;

void processMPU6050Data()
{
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float dt = (millis() - lastMPUTime) / 1000.0f;
  lastMPUTime = millis();

  // 将原始数据转换为角度
  float angleX = atan2(ay, az) * 180 / PI;
  float angleY = atan2(ax, az) * 180 / PI;
  float angleZ = atan2(ay, ax) * 180 / PI;

  // 应用卡尔曼滤波
  filteredAx = kalmanX.update(angleX, gx / 131.0f, dt);
  filteredAy = kalmanY.update(angleY, gy / 131.0f, dt);
  filteredAz = kalmanZ.update(angleZ, gz / 131.0f, dt);
}

// PID控制器类
class PIDController
{
private:
  float kp, ki, kd;
  float lastError;
  float integral;
  float outputMin, outputMax;
  unsigned long lastTime;

public:
  PIDController(float p, float i, float d, float min, float max)
      : kp(p), ki(i), kd(d), lastError(0), integral(0),
        outputMin(min), outputMax(max), lastTime(0) {}

  float compute(float setpoint, float input)
  {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    if (lastTime == 0)
    {
      dt = 0;
    }
    lastTime = now;

    // 计算误差
    float error = setpoint - input;

    // 积分项
    integral += error * dt;

    // 限制积分范围，防积分饱和
    integral = constrain(integral, outputMin / ki, outputMax / ki);

    // 微分项
    float derivative = dt > 0 ? (error - lastError) / dt : 0;
    lastError = error;

    // 计算输出
    float output = kp * error + ki * integral + kd * derivative;

    // 限制输出范围
    return constrain(output, outputMin, outputMax);
  }

  void reset()
  {
    lastError = 0;
    integral = 0;
    lastTime = 0;
  }
};

// 创建PID控制器实例
// 参数顺序：P, I, D, 最小输出, 最大输出
PIDController throttlePID(4.0, 0.2, 0.1, -100, 100);  // 增加P值使响应更快
PIDController steeringPID(3.0, 0.1, 0.05, -100, 100); // 增加P值使响应更快

// 当前实际值
float currentThrottle = 0;
float currentSteering = 0;

// 修改信号处理函数
void processSignals()
{
  float targetThrottle, targetSteering;
  if (signalQueue.pop(targetThrottle, targetSteering))
  {
    // 简化处理流程，减少延迟
    rxC1 = targetThrottle; // 直接更新值
    rxC2 = targetSteering;

    // 立即更新舵机
    updateServo();
  }
}

// 修改loop函数
void loop()
{
  static unsigned long lastProcessTime = 0;
  static unsigned long lastMPUTime = 0;
  const unsigned long processInterval = 1; // 减少到1ms以获得最快的响应
  const unsigned long mpuInterval = 20;    // MPU更新间隔保持20ms

  unsigned long currentTime = millis();

  // 优先处理遥感信号
  if (currentTime - lastProcessTime >= processInterval)
  {
    processSignals();
    lastProcessTime = currentTime;
  }

  // MPU6050数据处理
  if (currentTime - lastMPUTime >= mpuInterval)
  {
    processMPU6050Data();
    lastMPUTime = currentTime;
  }

  // Web请求处理
  server.handleClient();

  yield(); // 让出CPU时间片
}