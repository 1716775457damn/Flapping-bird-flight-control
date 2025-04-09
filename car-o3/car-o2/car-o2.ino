#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_task_wdt.h>

// WiFi配置
const char *ssid = "ovo";          // 改回原来的ssid
const char *password = "twx20051"; // 改回原来的password

// 服务器配置
const char *serverIP = "192.168.133.254"; // 改为你电脑的IP地址
const int serverPort = 8080;

// OLED显示屏配置
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// MPU6050对象
MPU6050 mpu;

// TCP客户端
WiFiClient client;

// 数据发送间隔
const unsigned long DATA_INTERVAL = 100; // 100ms
unsigned long lastDataTime = 0;

// 显示更新间隔
const unsigned long DISPLAY_INTERVAL = 200; // 200ms
unsigned long lastDisplayTime = 0;

// MPU6050数据
float filteredAx = 0;
float filteredAy = 0;
float filteredAz = 0;

// 添加陀螺仪零漂补偿相关变量
const int CALIBRATION_SAMPLES = 1000; // 开机校准采样数
const float STATIC_THRESHOLD = 0.05;  // 静态检测阈值
float gyroXOffset = 0;
float gyroYOffset = 0;
float gyroZOffset = 0;

// 添加新的MPU6050数据结构
struct MPUData
{
  float acc[3];  // 加速度数据 [x,y,z]
  float gyro[3]; // 陀螺仪数据 [x,y,z]
  bool isStatic; // 静态状态标志
};

// 添加舵机控制相关类和变量
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
  void setScale(float scale);
  void attachPin(int pin);
  void write(float value, float min, float max);

  // 添加PWM范围设置方法
  void setPWMRange(uint16_t min_us, uint16_t max_us)
  {
    // 将微秒转换为PWM值
    this->pwmMin = (min_us * this->freq * pow(2, this->resolution)) / 1000000;
    this->pwmMax = (max_us * this->freq * pow(2, this->resolution)) / 1000000;
  }
};

// 添加舵机控制变量
float sendMin = -100;
float sendMax = 100;
float sendHaf = 0;

float rxC1 = sendHaf; // 油门值
float rxC2 = sendHaf; // 转向值

float lastRxC1 = rxC1;
float lastRxC2 = rxC2;

float currentMotorSpeed = 0;  // 添加当前电机速度变量

LedcServo rxC1Servo;    // 动力电机
LedcServo rxC1ServoHaf; // 动力电机(半强度)
LedcServo rxC2Servo;    // 转向舵机
LedcServo rxC2ServoHaf; // 转向舵机(半强度)

// 在全局变量区域添加
bool isHolding = false; // 添加变量来跟踪持续状态

// 4. 添加电机状态结构
struct MotorState
{
  float throttle;               // 油门值 (-100 到 100)
  float steering;               // 转向值 (-100 到 100)
  bool isActive;                // 电机是否激活
  unsigned long lastUpdateTime; // 最后更新时间
};

// 5. 添加系统状态结构
struct SystemState
{
  bool isConnected;           // TCP连接状态
  bool isMPUCalibrated;       // MPU校准状态
  unsigned long lastDataTime; // 最后数据接收时间
  String lastError;           // 最后错误信息
};

// 6. 添加全局状态变量
MotorState motorState = {0, 0, false, 0};
SystemState sysState = {false, false, 0, ""};

// 修改PWM相关的定义
#define ESC_FREQ 50       // 电调需要50Hz信号
#define ESC_RESOLUTION 16 // 使用16位分辨率以获得更精确的控制
#define ESC_MIN_US 1000   // 最小脉宽(停止)
#define ESC_MAX_US 2000   // 最大脉宽(全速)
#define ESC_MID_US 1500   // 中间位置(零点)

// PWM输出引脚定义 (使用ESP32-S3的PWM引脚)
#define PIN_ESC_MAIN 7   // 主动力电机PWM (GPIO7)
#define PIN_ESC_HALF 6   // 半强度动力PWM (GPIO6)
#define PIN_SERVO_MAIN 5 // 主转向舵机PWM (GPIO5)
#define PIN_SERVO_HALF 4 // 半强度转向PWM (GPIO4)

const int MAX_RECONNECT_ATTEMPTS = 5;
const unsigned long RECONNECT_DELAY = 5000;  // 5秒
unsigned long lastReconnectAttempt = 0;

#define WDT_TIMEOUT 5  // 5秒超时

const float ALPHA = 0.96;  // 互补滤波系数
float compAngleX = 0;
float compAngleY = 0;

const unsigned long MOTOR_TIMEOUT = 1000;  // 1秒超时
const float MAX_ACCELERATION = 50.0;  // 最大加速度变化率

const int WIFI_TIMEOUT = 20000;  // WiFi连接超时时间（20秒）
const int WIFI_RETRY_DELAY = 500;  // WiFi重试延迟

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting setup...");

  // 打印WiFi模块信息
  Serial.print("WiFi MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("WiFi capabilities: ");
  Serial.println(ESP.getChipModel());
  Serial.print("Flash size: ");
  Serial.println(ESP.getFlashChipSize());

  // 初始化看门狗
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  // 初始化I2C
  Wire.begin(8, 9);
  delay(100);
  Serial.println("I2C initialized");

  // 初始化OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C) && !display.begin(SSD1306_SWITCHCAPVCC, 0x3D))
  {
    Serial.println(F("SSD1306 initialization failed"));
    // 添加I2C扫描
    scanI2CDevices();
  } else {
    Serial.println(F("SSD1306 initialization succeeded"));
    // 测试显示
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("OLED Test");
    display.display();
    delay(2000);
  }

  // 初始化MPU6050
  mpu.initialize();
  if (!mpu.testConnection())
  {
    Serial.println("MPU6050初始化失败");
    while (1)
    {
      delay(500);
    }
  }
  Serial.println("MPU6050初始化成功");

  // 配置MPU6050
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setDLPFMode(MPU6050_DLPF_BW_5);

  // 初始化舵机控制
  setupServos();

  // 完全重置WiFi配置
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);

  // 连接WiFi（添加超时处理）
  if (!connectWiFi()) {
    Serial.println("WiFi connection failed, restarting...");
    // 在重启前等待一段时间并打印更多信息
    Serial.println("Last WiFi error details:");
    Serial.printf("SSID: %s\n", ssid);
    Serial.printf("Password length: %d\n", strlen(password));
    Serial.printf("WiFi mode: %d\n", WiFi.getMode());
    delay(5000);
    ESP.restart();
  }

  // 连接TCP服务器
  connectServer();

  // 校准陀螺仪
  Serial.println("开始陀螺仪零漂校准...");
  calibrateGyro();
  Serial.println("陀螺仪零漂校准完成");

  Serial.println("Setup completed");
}

void loop()
{
  static unsigned long lastProcessTime = 0;
  static unsigned long lastMPUTime = 0;
  const unsigned long PROCESS_INTERVAL = 10; // 10ms
  const unsigned long MPU_INTERVAL = 20;     // 20ms

  unsigned long currentTime = millis();

  // 喂狗
  esp_task_wdt_reset();

  // 处理MPU6050数据
  if (currentTime - lastMPUTime >= MPU_INTERVAL)
  {
    processMPUData();
    lastMPUTime = currentTime;
  }

  // 处理电机控制信号
  if (currentTime - lastProcessTime >= PROCESS_INTERVAL)
  {
    if (motorState.isActive)
    {
      processSignals();
    }
    lastProcessTime = currentTime;
  }

  // 定时发送数据
  if (currentTime - lastDataTime >= DATA_INTERVAL)
  {
    lastDataTime = currentTime;
    sendMPUData();
  }

  // 定时更新显示
  if (currentTime - lastDisplayTime >= DISPLAY_INTERVAL)
  {
    lastDisplayTime = currentTime;
    updateOLED();
  }

  static unsigned long lastWiFiCheck = 0;
  const unsigned long WIFI_CHECK_INTERVAL = 5000;  // 每5秒检查一次WiFi状态

  // 定期检查WiFi状态
  if (millis() - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {
    lastWiFiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi connection lost!");
      // 打印当前WiFi状态
      Serial.printf("Current WiFi status: %d\n", WiFi.status());
      Serial.printf("Signal strength: %d dBm\n", WiFi.RSSI());
      // 尝试重新连接前先断开
      WiFi.disconnect(true);
      delay(1000);
      connectWiFi();
    }
  }

  // 检查TCP连接和数据
  if (client.connected())
  {
    while (client.available())
    {
      String data = client.readStringUntil('\n');
      if (data.length() > 0)
      {
        processIncomingData(data);
      }
    }
  }
  else
  {
    Serial.println("TCP connection lost, attempting to reconnect...");
    connectServer();
  }

  yield();
}

void connectServer()
{
  static int reconnectCount = 0;
  
  if (!WiFi.isConnected()) {
    Serial.println("WiFi disconnected, reconnecting...");
    if (!connectWiFi()) {
      Serial.println("WiFi reconnection failed!");
      return;
    }
    return;
  }
  
  // 检查重连间隔
  if (millis() - lastReconnectAttempt < RECONNECT_DELAY) {
    return;
  }
  
  Serial.println("Connecting to server...");
  Serial.printf("Server IP: %s, Port: %d\n", serverIP, serverPort);
  
  // 设置连接超时
  client.setTimeout(5000);  // 5秒超时
  
  if (client.connect(serverIP, serverPort)) {
    Serial.println("Server connected!");
    client.setNoDelay(true);
    reconnectCount = 0;
    // 发送初始化消息
    client.println("{\"type\":\"init\",\"device\":\"esp32\"}");
  } else {
    reconnectCount++;
    Serial.printf("Connection failed (%d/%d) - ", reconnectCount, MAX_RECONNECT_ATTEMPTS);
    // 检查WiFi连接状态
    if (!WiFi.isConnected()) {
      Serial.println("WiFi disconnected");
    } else {
      Serial.printf("Write error: %d\n", client.getWriteError());
    }
    if (reconnectCount >= MAX_RECONNECT_ATTEMPTS) {
      Serial.println("Max retry attempts reached, restarting ESP32");
      ESP.restart();
    }
  }
  
  lastReconnectAttempt = millis();
}

void calibrateGyro()
{
  float sumX = 0, sumY = 0, sumZ = 0;

  // 采集静态数据
  for (int i = 0; i < CALIBRATION_SAMPLES; i++)
  {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);

    // 转换为实际值
    const float gyroScale = 1.0 / 131.0; // ±250°/s范围
    sumX += gx * gyroScale;
    sumY += gy * gyroScale;
    sumZ += gz * gyroScale;

    delay(1);
  }

  // 计算平均值作为零漂补偿
  gyroXOffset = sumX / CALIBRATION_SAMPLES;
  gyroYOffset = sumY / CALIBRATION_SAMPLES;
  gyroZOffset = sumZ / CALIBRATION_SAMPLES;

  Serial.printf("陀螺仪零漂: X=%.2f, Y=%.2f, Z=%.2f\n",
                gyroXOffset, gyroYOffset, gyroZOffset);
}

bool isStatic(float ax, float ay, float az, float gx, float gy, float gz)
{
  // 检查加速度和角速度是否在阈值范围内
  const float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);
  const float gyroMagnitude = sqrt(gx * gx + gy * gy + gz * gz);

  // 加速度应该接近1g，角速度应该接近0
  return (abs(accelMagnitude - 1.0) < STATIC_THRESHOLD) &&
         (gyroMagnitude < STATIC_THRESHOLD);
}

void sendMPUData()
{
  MPUData data = getMPUData();

  // 创建JSON格式的数据
  char jsonData[256];
  snprintf(jsonData, sizeof(jsonData),
           "{\"type\":\"mpu_data\",\"data\":{\"acceleration\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f},\"gyro\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}}}",
           data.acc[0], data.acc[1], data.acc[2],
           data.gyro[0], data.gyro[1], data.gyro[2]);

  // 发送数据到服务器
  if (client.connected())
  {
    client.println(jsonData);
    // 添加调试输出
    Serial.print("Sent: ");
    Serial.println(jsonData);
  }

  // 更新OLED显示
  updateOLEDWithData(data);
}

void updateOLED()
{
  MPUData data = getMPUData();
  updateOLEDWithData(data);
}

void updateOLEDWithData(const MPUData &data)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // 显示标题
  display.setCursor(0, 0);
  display.println("MPU6050 Data:");

  // 显示加速度数据
  char buffer[32];
  display.setCursor(0, 16);
  snprintf(buffer, sizeof(buffer), "X: %.2f g", data.acc[0]);
  display.println(buffer);

  display.setCursor(0, 26);
  snprintf(buffer, sizeof(buffer), "Y: %.2f g", data.acc[1]);
  display.println(buffer);

  display.setCursor(0, 36);
  snprintf(buffer, sizeof(buffer), "Z: %.2f g", data.acc[2]);
  display.println(buffer);

  // 显示角速度数据
  display.setCursor(64, 16);
  snprintf(buffer, sizeof(buffer), "gX: %.1f", data.gyro[0]);
  display.println(buffer);

  display.setCursor(64, 26);
  snprintf(buffer, sizeof(buffer), "gY: %.1f", data.gyro[1]);
  display.println(buffer);

  display.setCursor(64, 36);
  snprintf(buffer, sizeof(buffer), "gZ: %.1f", data.gyro[2]);
  display.println(buffer);

  // 显示连接状态
  display.setCursor(0, 52);
  if (client.connected())
  {
    display.println("TCP: Connected");
  }
  else
  {
    display.println("TCP: Disconnected");
  }

  display.display();
}

MPUData getMPUData()
{
  MPUData data;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  const float accelScale = 1.0 / 16384.0; // ±2g范围
  data.acc[0] = ax * accelScale;
  data.acc[1] = ay * accelScale;
  data.acc[2] = az * accelScale;

  const float gyroScale = 1.0 / 131.0; // ±250°/s范围
  data.gyro[0] = gx * gyroScale - gyroXOffset;
  data.gyro[1] = gy * gyroScale - gyroYOffset;
  data.gyro[2] = gz * gyroScale - gyroZOffset;

  data.isStatic = isStatic(data.acc[0], data.acc[1], data.acc[2],
                           data.gyro[0], data.gyro[1], data.gyro[2]);

  return data;
}

void processMPUData()
{
  MPUData data = getMPUData();
  
  // 计算加速度计角度
  float accAngleX = atan2(data.acc[1], data.acc[2]) * RAD_TO_DEG;
  float accAngleY = atan2(-data.acc[0], sqrt(data.acc[1] * data.acc[1] + data.acc[2] * data.acc[2])) * RAD_TO_DEG;
  
  // 互补滤波
  float dt = 0.01;  // 假设采样间隔为10ms
  compAngleX = ALPHA * (compAngleX + data.gyro[0] * dt) + (1 - ALPHA) * accAngleX;
  compAngleY = ALPHA * (compAngleY + data.gyro[1] * dt) + (1 - ALPHA) * accAngleY;
  
  // 更新滤波后的加速度值
  filteredAx = data.acc[0];
  filteredAy = data.acc[1];
  filteredAz = data.acc[2];

  updateOLED();
}

void processSignals()
{
  static unsigned long lastCommandTime = 0;
  static float lastThrottle = 0;
  
  // 检查命令超时
  if (millis() - lastCommandTime > MOTOR_TIMEOUT) {
    rxC1 = 0;  // 停止电机
    rxC2 = 0;  // 回正方向
  }
  
  // 限制加速度
  float throttleDelta = rxC1 - lastThrottle;
  if (abs(throttleDelta) > MAX_ACCELERATION) {
    rxC1 = lastThrottle + (throttleDelta > 0 ? MAX_ACCELERATION : -MAX_ACCELERATION);
  }

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

  // 更新当前电机速度
  currentMotorSpeed = rxC1;

  lastThrottle = rxC1;
  lastCommandTime = millis();
}

// 添加舵机控制方法实现
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

// 修改setupServos函数
void setupServos()
{
  // 初始化主动力电调
  rxC1Servo.setup(ESC_FREQ, ESC_RESOLUTION, 8);
  rxC1Servo.attachPin(PIN_ESC_MAIN);
  rxC1Servo.setPWMRange(ESC_MIN_US, ESC_MAX_US);       // 设置PWM范围
  rxC1Servo.write(ESC_MID_US, ESC_MIN_US, ESC_MAX_US); // 初始化到中间位置

  // 初始化半强度动力电调
  rxC1ServoHaf.setup(ESC_FREQ, ESC_RESOLUTION, 9);
  rxC1ServoHaf.attachPin(PIN_ESC_HALF);
  rxC1ServoHaf.setPWMRange(ESC_MIN_US, ESC_MAX_US);
  rxC1ServoHaf.write(ESC_MID_US, ESC_MIN_US, ESC_MAX_US);

  // 初始化转向舵机（保持不变）
  rxC2Servo.setup(50, 16, 10);
  rxC2Servo.attachPin(PIN_SERVO_MAIN);
  rxC2Servo.setScale(0.7);
  rxC2Servo.write(0, -1, 1);

  rxC2ServoHaf.setup(50, 16, 11);
  rxC2ServoHaf.attachPin(PIN_SERVO_HALF);
  rxC2ServoHaf.setScale(0.3);
  rxC2ServoHaf.write(0, -1, 1);

  // 电调校准程序
  calibrateESC();
}

// 添加电调校准函数
void calibrateESC()
{
  Serial.println("Starting ESC calibration...");
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("ESC Calibrating...");
  display.display();

  delay(5000);

  // 发送最大油门信号
  rxC1Servo.write(ESC_MAX_US, ESC_MIN_US, ESC_MAX_US);
  rxC1ServoHaf.write(ESC_MAX_US, ESC_MIN_US, ESC_MAX_US);
  delay(2000);

  // 发送最小油门信号
  rxC1Servo.write(ESC_MIN_US, ESC_MIN_US, ESC_MAX_US);
  rxC1ServoHaf.write(ESC_MIN_US, ESC_MIN_US, ESC_MAX_US);
  delay(2000);

  // 回到中间位置
  rxC1Servo.write(ESC_MID_US, ESC_MIN_US, ESC_MAX_US);
  rxC1ServoHaf.write(ESC_MID_US, ESC_MIN_US, ESC_MAX_US);

  Serial.println("ESC calibration complete");
  display.clearDisplay();
  display.println("ESC Calibrated!");
  display.display();
  delay(1000);
}

// 修改数据处理部分
void processIncomingData(String data)
{
  // 检查数据格式是否为 m:x,y,z,rx,ry,rz 或 c:throttle,steering
  if (data.startsWith("m:"))
  {
    // 处理MPU数据
    // ... 保持原有MPU数据处理逻辑 ...
  }
  else if (data.startsWith("c:"))
  {
    // 处理控制数据
    data = data.substring(2); // 移除"c:"前缀
    int commaIndex = data.indexOf(',');
    if (commaIndex > 0)
    {
      String throttleStr = data.substring(0, commaIndex);
      String steeringStr = data.substring(commaIndex + 1);

      // 转换为浮点数
      float throttle = throttleStr.toFloat();
      float steering = steeringStr.toFloat();

      // 更新控制值
      rxC1 = throttle;
      rxC2 = steering;

      // 立即处理控制信号
      processSignals();
    }
  }
}

// 添加map函数用于值映射
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// 修改updateOLEDDisplay函数
void updateOLEDDisplay(bool forceUpdate = false)
{
  static unsigned long lastDebugTime = 0;
  static unsigned long lastUpdateTime = 0;
  static float lastAx = 0, lastAy = 0, lastAz = 0;
  static float lastGx = 0, lastGy = 0, lastGz = 0;
  static int lastMotorSpeed = -999;
  static bool lastWiFiStatus = false;
  
  // 检查是否需要更新
  if (!forceUpdate && millis() - lastUpdateTime < 100) {
    return;
  }

  // 每5秒打印一次调试信息
  if (millis() - lastDebugTime > 5000) {
    Serial.println("OLED Update Debug:");
    Serial.printf("WiFi Status: %s\n", WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("Acc: %.2f, %.2f, %.2f\n", filteredAx, filteredAy, filteredAz);
    Serial.printf("Motor Speed: %d%%\n", currentMotorSpeed);
    lastDebugTime = millis();
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  
  // 显示内容
  display.print("WiFi: ");
  display.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
  
  if (WiFi.status() == WL_CONNECTED) {
    display.print("IP: ");
    display.println(WiFi.localIP().toString());
  }
  
  // 显示MPU6050数据
  display.print("Acc: ");
  display.print(String(filteredAx, 1));
  display.print(",");
  display.print(String(filteredAy, 1));
  display.print(",");
  display.println(String(filteredAz, 1));
  
  display.print("Gyro: ");
  display.print(String(mpu.getRotationX() / 131.0, 1));
  display.print(",");
  display.print(String(mpu.getRotationY() / 131.0, 1));
  display.print(",");
  display.println(String(mpu.getRotationZ() / 131.0, 1));
  
  // 显示电机状态
  display.print("Motor: ");
  display.print(currentMotorSpeed);
  display.println("%");
  
  // 显示TCP状态
  display.print("TCP: ");
  display.println(client.connected() ? "Connected" : "Disconnected");
  
  display.display();
  yield();
  
  // 更新状态变量
  lastUpdateTime = millis();
  lastAx = filteredAx;
  lastAy = filteredAy;
  lastAz = filteredAz;
  lastGx = mpu.getRotationX();
  lastGy = mpu.getRotationY();
  lastGz = mpu.getRotationZ();
  lastMotorSpeed = currentMotorSpeed;
  lastWiFiStatus = (WiFi.status() == WL_CONNECTED);
}

void scanI2CDevices() {
  Serial.println("Scanning I2C devices...");
  byte error, address;
  int nDevices = 0;

  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.println("I2C scan complete");
  }
}

bool connectWiFi() {
  Serial.printf("Connecting to WiFi: %s\n", ssid);
  
  // 完全重置WiFi配置
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  
  WiFi.begin(ssid, password);
  
  unsigned long startAttemptTime = millis();
  
  while (WiFi.status() != WL_CONNECTED && 
         millis() - startAttemptTime < WIFI_TIMEOUT) {
    Serial.print(".");
    // 添加WiFi状态诊断
    switch(WiFi.status()) {
      case WL_IDLE_STATUS:
        Serial.println("WiFi IDLE");
        break;
      case WL_NO_SSID_AVAIL:
        Serial.println("WiFi SSID not found");
        break;
      case WL_SCAN_COMPLETED:
        Serial.println("WiFi scan completed");
        break;
      case WL_CONNECT_FAILED:
        Serial.println("WiFi connect failed");
        break;
      case WL_CONNECTION_LOST:
        Serial.println("WiFi connection lost");
        break;
      case WL_DISCONNECTED:
        Serial.println("WiFi disconnected");
        break;
    }
    delay(WIFI_RETRY_DELAY);
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    // 打印连接信息
    Serial.print("Signal strength (RSSI): ");
    Serial.println(WiFi.RSSI());
    Serial.print("MAC address: ");
    Serial.println(WiFi.macAddress());
    return true;
  } else {
    Serial.println("\nWiFi connection failed!");
    Serial.printf("WiFi status: %d\n", WiFi.status());
    // 打印失败原因
    Serial.println("Possible reasons:");
    Serial.println("1. Wrong password");
    Serial.println("2. WiFi network not in range");
    Serial.println("3. Router not responding");
    return false;
  }
}