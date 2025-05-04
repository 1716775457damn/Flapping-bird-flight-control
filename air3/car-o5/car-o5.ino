#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <stdio.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <CircularBuffer.hpp>
#include <driver/ledc.h> // 添加ESP-IDF LEDC驱动头文件
#include <esp_pm.h> // 添加电源管理头文件

// ESP32S3特定配置
#define CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ 240 // 设置CPU频率为240MHz

// 用于检查ESP32S3芯片
void checkAndPrintESP32Info() {
  Serial.println("\n=== ESP32硬件信息 ===");
  
  // 检查芯片类型
  #if CONFIG_IDF_TARGET_ESP32S3
    Serial.println("芯片类型: ESP32-S3");
  #elif CONFIG_IDF_TARGET_ESP32
    Serial.println("芯片类型: ESP32");
  #elif CONFIG_IDF_TARGET_ESP32S2
    Serial.println("芯片类型: ESP32-S2");
  #elif CONFIG_IDF_TARGET_ESP32C3
    Serial.println("芯片类型: ESP32-C3");
  #else
    Serial.println("芯片类型: 未知");
  #endif
  
  // 打印CPU频率
  Serial.print("CPU频率: ");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.println(" MHz");
  
  // 打印闪存大小
  Serial.print("闪存大小: ");
  Serial.print(ESP.getFlashChipSize() / 1024 / 1024);
  Serial.println(" MB");
  
  // 打印可用RAM
  Serial.print("可用RAM: ");
  Serial.print(ESP.getFreeHeap() / 1024);
  Serial.println(" KB");
  
  // 打印芯片版本
  Serial.print("芯片版本: ");
  Serial.println(ESP.getChipRevision());
  
  Serial.println("=== 硬件信息完成 ===\n");
}

// 配置ESP32S3电源管理以获得稳定的PWM输出
void configureESP32S3PowerManagement() {
  // 禁用自动光功能以确保稳定的CPU频率
  #if CONFIG_IDF_TARGET_ESP32S3
    // 配置性能模式
    esp_pm_config_esp32s3_t pm_config = {
      .max_freq_mhz = CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ,
      .min_freq_mhz = CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ,
      .light_sleep_enable = false
    };
    esp_err_t err = esp_pm_configure(&pm_config);
    if (err != ESP_OK) {
      Serial.println("电源管理配置失败");
    } else {
      Serial.println("电源管理已配置为性能模式");
    }
  #endif
}

// 定义用于与JY901通信的串口
HardwareSerial jy901Serial(2); // 使用ESP32的硬件串口2

// JY901数据结构
struct JY901Data {
  float acc[3];    // 加速度
  float gyro[3];   // 角速度
  float angle[3];  // 角度
  int mag[3];      // 磁场
} sensorData;

// 接收JY901数据的缓冲区
unsigned char ucRxBuffer[250];
unsigned char ucRxCnt = 0;

// 从esp32s3_test.ino添加的ESP-IDF LEDC控制函数
// 配置定时器和通道 - 优化ESP32S3版本
bool setupLedcESP_IDF(uint8_t pin, uint8_t channel, uint32_t freq, uint8_t resolution) {
  // 配置定时器 - 避免使用初始化列表方式，改用逐个字段赋值
  ledc_timer_config_t ledc_timer;
  memset(&ledc_timer, 0, sizeof(ledc_timer)); // 先清零所有字段
  
  // ESP32S3特定配置
  ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_timer.timer_num = (ledc_timer_t)(channel / 8);
  ledc_timer.freq_hz = freq;
  ledc_timer.duty_resolution = (ledc_timer_bit_t)resolution;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;
  
  esp_err_t err = ledc_timer_config(&ledc_timer);
  if (err != ESP_OK) {
    Serial.print("LEDC定时器配置失败: ");
    Serial.println(err);
    return false;
  }
  
  // 配置通道 - 使用ESP32S3特定设置
  ledc_channel_config_t ledc_channel;
  memset(&ledc_channel, 0, sizeof(ledc_channel)); // 先清零所有字段
  
  ledc_channel.gpio_num = pin;
  ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_channel.channel = (ledc_channel_t)channel;
  ledc_channel.intr_type = LEDC_INTR_DISABLE;
  ledc_channel.timer_sel = (ledc_timer_t)(channel / 8);
  ledc_channel.duty = 0;         // 初始占空比设为0
  ledc_channel.hpoint = 0;       // ESP32S3可能需要这个值
  ledc_channel.flags.output_invert = 0;  // 确保输出不反转
  
  err = ledc_channel_config(&ledc_channel);
  if (err != ESP_OK) {
    Serial.print("LEDC通道配置失败: ");
    Serial.println(err);
    return false;
  }
  
  // ESP32S3特定：确保通道启用
  err = ledc_timer_rst(LEDC_LOW_SPEED_MODE, (ledc_timer_t)(channel / 8));
  if (err != ESP_OK) {
    Serial.print("LEDC定时器重置失败: ");
    Serial.println(err);
    // 继续执行，这不是关键错误
  }
  
  return true;
}

// 设置LEDC占空比 - 针对ESP32S3优化
void setDutyESP_IDF(uint8_t channel, uint32_t duty, uint8_t resolution) {
  // 计算最大占空比值
  uint32_t max_duty = (1 << resolution) - 1;
  
  // 限制占空比范围
  if (duty > max_duty) {
    duty = max_duty;
  }
  
  // 记录调试信息
  Serial.print("设置通道 ");
  Serial.print(channel);
  Serial.print(" 占空比: ");
  Serial.print(duty);
  Serial.print("/");
  Serial.println(max_duty);
  
  // 使用ESP-IDF APIs设置占空比
  esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, duty);
  if (err != ESP_OK) {
    Serial.print("设置占空比失败: ");
    Serial.println(err);
    return;
  }
  
  // 更新占空比 - 这一步实际应用设置的值
  err = ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel);
  if (err != ESP_OK) {
    Serial.print("更新占空比失败: ");
    Serial.println(err);
  }
  
  // ESP32S3特定：确保更新生效
  delay(1); // 给硬件一点时间更新
}

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
  int type = 0; // 0=普通舵机，1=连续旋转舵机
  int pin = -1; // 存储引脚编号
  
  void setup(float freq, int resolution, int channel, int servoType = 0);
  /* 0 < scale <= 1 */
  void setScale(float scale);
  void attachPin(int pin);
  void write(float value, float min, float max);
  void writeSpeed(float speed); // 添加专用于连续旋转舵机的控制方法
};

void LedcServo::setup(float f, int r, int c, int servoType)
{
  this->freq = f;
  this->resolution = r;
  this->pwmBaseScale = this->freq * pow(2, this->resolution) / 1000;
  this->pwmMin = 1 * this->pwmBaseScale;
  this->pwmMax = 2 * this->pwmBaseScale;
  this->channel = c;
  this->type = servoType;
  
  // 使用ESP-IDF LEDC API替代Arduino的ledcSetup
  // 注意：在attachPin时才真正配置PWM
}

void LedcServo::setScale(float s)
{
  if (s <= 0)
    throw "s 不能小于等于0";
  if (s > 2)
    throw "s 不能大于2";

  this->scale = s;
  this->pwmMin = (1.5 - s * 0.5) * this->pwmBaseScale;
  this->pwmMax = (1.5 + s * 0.5) * this->pwmBaseScale;
}

void LedcServo::attachPin(int p)
{
  // 保存引脚号
  this->pin = p;
  
  // 使用ESP-IDF LEDC API替代Arduino的ledcAttachPin
  setupLedcESP_IDF(p, this->channel, this->freq, this->resolution);
  
  // 输出调试信息
  Serial.print("舵机配置 - 引脚:");
  Serial.print(p);
  Serial.print(" 通道:");
  Serial.print(this->channel);
  Serial.print(" 频率:");
  Serial.print(this->freq);
  Serial.print(" 分辨率:");
  Serial.println(this->resolution);
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
  
  // 特别针对ESP32S3修改占空比计算
  // 标准舵机：1ms~2ms脉冲，对应-90~+90度
  // 计算需要的脉冲宽度(单位：微秒)
  float pulseWidth;
  float ratio = (newV - min) / (max - min); // 0~1之间的比例
  
  // 从目标比例计算脉冲宽度，1000~2000微秒
  pulseWidth = 1000.0 + ratio * 1000.0;
  
  // 将脉冲宽度(微秒)转换为ESP-IDF LEDC需要的占空比值
  uint32_t period = 1000000 / this->freq; // 周期(微秒)
  uint32_t duty = (1 << this->resolution) * pulseWidth / period;
  
  // 确保占空比不超过最大值
  uint32_t maxDuty = (1 << this->resolution) - 1;
  if (duty > maxDuty) duty = maxDuty;
  
  // 输出调试信息
  Serial.print("舵机写入 - 值:");
  Serial.print(newV);
  Serial.print(" 脉冲宽度:");
  Serial.print(pulseWidth);
  Serial.print("微秒 占空比:");
  Serial.print(duty);
  Serial.print("/");
  Serial.println(maxDuty);
  
  // 使用ESP-IDF LEDC API设置占空比
  setDutyESP_IDF(this->channel, duty, this->resolution);
}

// 添加连续旋转舵机的速度控制方法
void LedcServo::writeSpeed(float speed)
{
  // 限制速度范围在-100到100之间
  speed = constrain(speed, -100, 100);
  
  // 计算脉冲宽度(微秒)
  float pulseWidth;
  
  // 对于连续旋转舵机，中值(1.5ms)停止，大于中值正转，小于中值反转
  if (abs(speed) < 5) {
    // 如果接近零则完全停止(1.5ms)
    pulseWidth = 1500.0;
  } else if (speed > 0) {
    // 正转：1.5ms到2.0ms
    pulseWidth = 1500.0 + (speed / 100.0) * 500.0;
  } else {
    // 反转：1.5ms到1.0ms
    pulseWidth = 1500.0 - (abs(speed) / 100.0) * 500.0;
  }
  
  // 将脉冲宽度(微秒)转换为ESP-IDF LEDC需要的占空比值
  uint32_t period = 1000000 / this->freq; // 周期(微秒)
  uint32_t duty = (1 << this->resolution) * pulseWidth / period;
  
  // 确保占空比不超过最大值
  uint32_t maxDuty = (1 << this->resolution) - 1;
  if (duty > maxDuty) duty = maxDuty;
  
  // 输出调试信息
  Serial.print("连续舵机速度 - 速度:");
  Serial.print(speed);
  Serial.print(" 脉冲宽度:");
  Serial.print(pulseWidth);
  Serial.print("微秒 占空比:");
  Serial.print(duty);
  Serial.print("/");
  Serial.println(maxDuty);
  
  // 使用ESP-IDF LEDC API设置PWM值
  setDutyESP_IDF(this->channel, duty, this->resolution);
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

// 在全局变量区域添加新的舵机对象
LedcServo steeringServo; // 新增转向舵机

// 在全局变量区域添加起飞状态变量
bool isTakingOff = false; // 用于跟踪起飞状态

/**
 * 接收信息的web server 监听80端口
 */
WebServer server(80);

unsigned long timeNow = 0;
unsigned long lastDataTickTime = 0;

const int LED_PIN = 2;
bool ledShow = false;
int ledLoopTick = -1;

// JY901角度数据
float filteredAx = 0, filteredAy = 0, filteredAz = 0;

// 在全局范围声明isHolding变量
bool isHolding = false; // 添加全局变量来跟踪持续状态

// 添加飞行器控制状态变量
bool isOutOfControl = false; // 飞行器是否失控
String outOfControlReason = ""; // 失控原因

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

// 添加倾角控制相关变量
float angleThreshold = 10.0;      // 开始响应的最小倾角(度)
float maxAngle = 45.0;            // 最大倾角(度)
float minSpeed = -100.0;          // 最小速度
float maxSpeed = 100.0;           // 最大速度
bool angleControlEnabled = false; // 倾角控制开关

// 在setup函数前添加全局变量
bool isLanding = false;
float landingSpeed = 0;
unsigned long lastLandingUpdateTime = 0;

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
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <meta name="apple-mobile-web-app-capable" content="yes">
  <meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
  <title>RC Controller</title>
  <style>
    /* Apple-inspired Design System */
    :root {
      --primary-color: #007AFF;
      --secondary-color: #5AC8FA;
      --success-color: #34C759;
      --warning-color: #FF9500;
      --danger-color: #FF3B30;
      --gray-1: #8E8E93;
      --gray-2: #AEAEB2;
      --gray-3: #C7C7CC;
      --gray-4: #D1D1D6;
      --gray-5: #E5E5EA;
      --gray-6: #F2F2F7;
      --background: #FFFFFF;
      --card-background: #FFFFFF;
      --text-primary: #000000;
      --text-secondary: #3C3C43;
      --text-tertiary: #3C3C4399;
      --border-radius: 12px;
      --section-border-radius: 20px;
      --shadow: 0 8px 24px rgba(0, 0, 0, 0.08);
      --card-shadow: 0 2px 14px rgba(0, 0, 0, 0.06);
    }
    
    @media (prefers-color-scheme: dark) {
      :root {
        --background: #000000;
        --card-background: #1C1C1E;
        --text-primary: #FFFFFF;
        --text-secondary: #EBEBF599;
        --text-tertiary: #EBEBF566;
        --shadow: 0 8px 24px rgba(0, 0, 0, 0.2);
        --card-shadow: 0 2px 14px rgba(0, 0, 0, 0.14);
      }
    }

    * {
      margin: 0;
      padding: 0;
      box-sizing: border-box;
      -webkit-tap-highlight-color: transparent;
      font-family: -apple-system, BlinkMacSystemFont, 'SF Pro Text', 'Helvetica Neue', sans-serif;
    }

    html, body {
      height: 100%;
      width: 100%;
      overflow-x: hidden;
      background-color: var(--gray-6);
      color: var(--text-primary);
    }

    .container {
      max-width: 100%;
      margin: 0 auto;
      padding: 0;
      position: relative;
      min-height: 100%;
      display: flex;
      flex-direction: column;
      gap: 20px;
    }

    .header {
      position: sticky;
      top: 0;
      z-index: 10;
      backdrop-filter: blur(10px);
      -webkit-backdrop-filter: blur(10px);
      background-color: rgba(var(--gray-6), 0.8);
      padding: 24px 20px 12px 20px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      border-bottom: 1px solid var(--gray-5);
    }

    .header h1 {
      font-size: 24px;
      font-weight: 700;
      color: var(--text-primary);
      margin: 0;
    }
    
    .status-indicator {
      width: 10px;
      height: 10px;
      border-radius: 50%;
      background-color: var(--danger-color);
      margin-left: auto;
    }

    .status-indicator.connected {
      background-color: var(--success-color);
    }

    .section {
      margin: 0 16px 20px;
      border-radius: var(--section-border-radius);
      overflow: hidden;
      background-color: var(--card-background);
      box-shadow: var(--card-shadow);
    }

    .section-header {
      padding: 16px;
      border-bottom: 1px solid var(--gray-5);
      display: flex;
      justify-content: space-between;
      align-items: center;
    }

    .section-header h2 {
      font-size: 18px;
      font-weight: 600;
    }

    .sensor-data {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 16px;
      padding: 16px;
    }

    .sensor-group {
      background-color: var(--gray-6);
      border-radius: var(--border-radius);
      padding: 16px;
      overflow: hidden;
    }

    .sensor-group h3 {
      font-size: 14px;
      font-weight: 600;
      color: var(--text-secondary);
      margin-bottom: 12px;
      text-transform: uppercase;
      letter-spacing: 0.5px;
    }

    .sensor-value {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 12px;
    }

    .sensor-axis {
      text-align: center;
      display: flex;
      flex-direction: column;
      align-items: center;
    }

    .axis-label {
      font-size: 14px;
      color: var(--text-tertiary);
      margin-bottom: 4px;
      font-weight: 500;
    }

    .axis-value {
      font-size: 16px;
      font-weight: 600;
      color: var(--text-primary);
      font-variant-numeric: tabular-nums;
    }

    .joystick-container {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 16px;
      padding: 16px;
    }

    .joystick-wrapper {
      display: flex;
      flex-direction: column;
      align-items: center;
    }

    .joystick {
      width: 140px;
      height: 140px;
      background: var(--gray-6);
      border-radius: 50%;
      position: relative;
      border: 2px solid var(--gray-4);
      overflow: hidden;
      touch-action: none;
    }

    .joystick::after {
      content: '';
      position: absolute;
      width: 2px;
      height: 2px;
      background-color: var(--gray-3);
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      border-radius: 50%;
    }

    .joystick .handle {
      width: 50px;
      height: 50px;
      background: var(--primary-color);
      border-radius: 50%;
      position: absolute;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      cursor: pointer;
      transition: box-shadow 0.2s;
      box-shadow: 0 4px 12px rgba(0, 122, 255, 0.4);
    }

    .joystick-label {
      margin-top: 12px;
      font-size: 16px;
      font-weight: 500;
      color: var(--text-secondary);
    }

    .control-panel {
      padding: 16px;
    }

    .control-group {
      margin-bottom: 16px;
    }

    .switch-container {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 12px;
      padding: 12px 0;
      border-bottom: 1px solid var(--gray-5);
    }

    .switch-label {
      font-size: 16px;
      font-weight: 500;
      color: var(--text-primary);
    }

    .switch {
      position: relative;
      display: inline-block;
      width: 51px;
      height: 31px;
    }

    .switch input {
      opacity: 0;
      width: 0;
      height: 0;
    }

    .slider {
      position: absolute;
      cursor: pointer;
      top: 0;
      left: 0;
      right: 0;
      bottom: 0;
      background-color: var(--gray-4);
      -webkit-transition: .2s;
      transition: .2s;
      border-radius: 34px;
    }

    .slider:before {
      position: absolute;
      content: "";
      height: 27px;
      width: 27px;
      left: 2px;
      bottom: 2px;
      background-color: white;
      -webkit-transition: .2s;
      transition: .2s;
      border-radius: 50%;
      box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
    }

    input:checked + .slider {
      background-color: var(--success-color);
    }

    input:checked + .slider:before {
      -webkit-transform: translateX(20px);
      -ms-transform: translateX(20px);
      transform: translateX(20px);
    }

    .action-buttons {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 12px;
      margin-top: 16px;
    }

    .button {
      padding: 14px 0;
      border-radius: var(--border-radius);
      font-size: 16px;
      font-weight: 600;
      text-align: center;
      border: none;
      cursor: pointer;
      transition: all 0.2s;
      color: white;
    }

    .button-primary {
      background-color: var(--primary-color);
      box-shadow: 0 4px 12px rgba(0, 122, 255, 0.2);
    }

    .button-success {
      background-color: var(--success-color);
      box-shadow: 0 4px 12px rgba(52, 199, 89, 0.2);
    }

    .button-danger {
      background-color: var(--danger-color);
      box-shadow: 0 4px 12px rgba(255, 59, 48, 0.2);
    }

    .button:active {
      transform: scale(0.97);
      box-shadow: none;
    }

    .data-monitor {
      padding: 16px;
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 12px;
    }

    .data-card {
      background-color: var(--gray-6);
      border-radius: var(--border-radius);
      padding: 16px;
      display: flex;
      flex-direction: column;
    }

    .data-label {
      font-size: 14px;
      color: var(--text-tertiary);
      margin-bottom: 4px;
      font-weight: 500;
    }

    .data-value {
      font-size: 24px;
      font-weight: 600;
      color: var(--text-primary);
      font-variant-numeric: tabular-nums;
    }
    
    .sensor-fullwidth {
      grid-column: 1 / -1;
    }
    
    .model-3d {
      width: 160px;
      height: 160px;
      background-color: var(--gray-6);
      margin: 16px auto;
      position: relative;
      transform-style: preserve-3d;
      transition: transform 0.1s ease;
    }

    .face {
      position: absolute;
      width: 100%;
      height: 100%;
      opacity: 0.8;
      display: flex;
      align-items: center;
      justify-content: center;
      font-weight: 600;
      border: 1px solid rgba(0,0,0,0.1);
    }

    .front {
      transform: translateZ(80px);
      background-color: rgba(255, 59, 48, 0.8);
    }

    .back {
      transform: translateZ(-80px) rotateY(180deg);
      background-color: rgba(0, 122, 255, 0.8);
    }

    .right {
      transform: translateX(80px) rotateY(90deg);
      background-color: rgba(52, 199, 89, 0.8);
    }

    .left {
      transform: translateX(-80px) rotateY(-90deg);
      background-color: rgba(255, 149, 0, 0.8);
    }

    .top {
      transform: translateY(-80px) rotateX(90deg);
      background-color: rgba(175, 82, 222, 0.8);
    }

    .bottom {
      transform: translateY(80px) rotateX(-90deg);
      background-color: rgba(90, 200, 250, 0.8);
    }
    
    .model-container {
      perspective: 800px;
      width: 160px;
      height: 160px;
      margin: 0 auto;
    }
    
    @media (max-width: 600px) {
      .sensor-data {
        grid-template-columns: 1fr;
      }
    }
    
    /* 添加失控警告样式 */
    .alert-container {
      margin-top: 16px;
      overflow: hidden;
      max-height: 0;
      transition: max-height 0.3s ease-out;
    }
    
    .alert-container.active {
      max-height: 80px;
    }
    
    .alert {
      background-color: var(--danger-color);
      color: white;
      padding: 16px;
      border-radius: var(--border-radius);
      display: flex;
      align-items: center;
      gap: 12px;
      font-weight: 600;
    }
    
    .alert-icon {
      width: 24px;
      height: 24px;
      background-color: white;
      border-radius: 50%;
      display: flex;
      align-items: center;
      justify-content: center;
      color: var(--danger-color);
      font-weight: bold;
    }
    
    @keyframes pulse {
      0% { transform: scale(1); }
      50% { transform: scale(1.05); }
      100% { transform: scale(1); }
    }
    
    .alert.pulse {
      animation: pulse 0.8s infinite;
    }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h1>RC Controller</h1>
      <div id="statusIndicator" class="status-indicator"></div>
    </div>
    
    <!-- 失控警告 -->
    <div id="alertContainer" class="alert-container">
      <div class="alert pulse">
        <div class="alert-icon">!</div>
        <div id="alertMessage">飞行器失控</div>
      </div>
    </div>
    
    <!-- JY901 传感器数据 -->
    <div class="section">
      <div class="section-header">
        <h2>JY901 传感器数据</h2>
      </div>
      <div class="sensor-data">
        <div class="sensor-group">
          <h3>角度 (°)</h3>
          <div class="sensor-value">
            <div class="sensor-axis">
              <div class="axis-label">Roll</div>
              <div id="angleRoll" class="axis-value">0.00</div>
            </div>
            <div class="sensor-axis">
              <div class="axis-label">Pitch</div>
              <div id="anglePitch" class="axis-value">0.00</div>
            </div>
            <div class="sensor-axis">
              <div class="axis-label">Yaw</div>
              <div id="angleYaw" class="axis-value">0.00</div>
            </div>
          </div>
        </div>
        <div class="sensor-group">
          <h3>加速度 (g)</h3>
          <div class="sensor-value">
            <div class="sensor-axis">
              <div class="axis-label">X</div>
              <div id="accX" class="axis-value">0.00</div>
            </div>
            <div class="sensor-axis">
              <div class="axis-label">Y</div>
              <div id="accY" class="axis-value">0.00</div>
            </div>
            <div class="sensor-axis">
              <div class="axis-label">Z</div>
              <div id="accZ" class="axis-value">0.00</div>
            </div>
          </div>
        </div>
        <div class="sensor-group">
          <h3>角速度 (°/s)</h3>
          <div class="sensor-value">
            <div class="sensor-axis">
              <div class="axis-label">X</div>
              <div id="gyroX" class="axis-value">0.00</div>
            </div>
            <div class="sensor-axis">
              <div class="axis-label">Y</div>
              <div id="gyroY" class="axis-value">0.00</div>
            </div>
            <div class="sensor-axis">
              <div class="axis-label">Z</div>
              <div id="gyroZ" class="axis-value">0.00</div>
            </div>
          </div>
        </div>
        <div class="sensor-group">
          <h3>磁场</h3>
          <div class="sensor-value">
            <div class="sensor-axis">
              <div class="axis-label">X</div>
              <div id="magX" class="axis-value">0</div>
            </div>
            <div class="sensor-axis">
              <div class="axis-label">Y</div>
              <div id="magY" class="axis-value">0</div>
            </div>
            <div class="sensor-axis">
              <div class="axis-label">Z</div>
              <div id="magZ" class="axis-value">0</div>
            </div>
          </div>
        </div>
        <div class="sensor-group sensor-fullwidth">
          <div class="model-container">
            <div id="model3D" class="model-3d">
              <div class="face front">前</div>
              <div class="face back">后</div>
              <div class="face right">右</div>
              <div class="face left">左</div>
              <div class="face top">上</div>
              <div class="face bottom">下</div>
            </div>
          </div>
        </div>
      </div>
    </div>
    
    <!-- 控制面板 -->
    <div class="section">
      <div class="section-header">
        <h2>控制面板</h2>
      </div>
      <div class="control-panel">
        <div class="control-group">
          <div class="switch-container">
            <span class="switch-label">倾角控制</span>
            <label class="switch">
              <input type="checkbox" id="angleControlSwitch">
              <span class="slider"></span>
            </label>
          </div>
        </div>
        <div class="action-buttons">
          <button id="takeoffButton" class="button button-success">一键起飞</button>
          <button id="landingButton" class="button button-danger">一键降落</button>
          <button id="stopButton" class="button button-primary">结束控制</button>
        </div>
      </div>
    </div>

    <!-- 遥感控制 -->
    <div class="section">
      <div class="section-header">
        <h2>遥感控制</h2>
      </div>
      <div class="joystick-container">
        <div class="joystick-wrapper">
          <div class="joystick" id="joystick1">
            <div class="handle"></div>
          </div>
          <div class="joystick-label">油门</div>
        </div>
        <div class="joystick-wrapper">
          <div class="joystick" id="joystick2">
            <div class="handle"></div>
          </div>
          <div class="joystick-label">转向</div>
        </div>
      </div>
    </div>
    
    <!-- 实时数据 -->
    <div class="section">
      <div class="section-header">
        <h2>实时数据</h2>
      </div>
      <div class="data-monitor">
        <div class="data-card">
          <div class="data-label">油门值</div>
          <div id="throttleValue" class="data-value">0</div>
        </div>
        <div class="data-card">
          <div class="data-label">舵机值</div>
          <div id="steeringValue" class="data-value">0</div>
        </div>
        <div class="data-card" style="grid-column: 1 / -1;">
          <div class="data-label">控制状态</div>
          <div id="controlStatus" class="data-value" style="color: var(--success-color);">正常</div>
        </div>
      </div>
    </div>
  </div>

  <script>
    // 状态指示灯
    const statusIndicator = document.getElementById('statusIndicator');
    let isConnected = false;
    
    // 失控警告元素
    const alertContainer = document.getElementById('alertContainer');
    const alertMessage = document.getElementById('alertMessage');
    
    // 3D模型元素
    const model3D = document.getElementById('model3D');
    
    // 遥感元素
    const joystick1 = document.getElementById('joystick1');
    const handle1 = joystick1.querySelector('.handle');
    
    // 数据元素
    const throttleValue = document.getElementById('throttleValue');
    const steeringValue = document.getElementById('steeringValue');
    const controlStatus = document.getElementById('controlStatus');
    
    // 角度数据
    const angleRoll = document.getElementById('angleRoll');
    const anglePitch = document.getElementById('anglePitch');
    const angleYaw = document.getElementById('angleYaw');
    
    // 加速度数据
    const accX = document.getElementById('accX');
    const accY = document.getElementById('accY');
    const accZ = document.getElementById('accZ');
    
    // 角速度数据
    const gyroX = document.getElementById('gyroX');
    const gyroY = document.getElementById('gyroY');
    const gyroZ = document.getElementById('gyroZ');
    
    // 磁场数据
    const magX = document.getElementById('magX');
    const magY = document.getElementById('magY');
    const magZ = document.getElementById('magZ');
    
    // 控制状态
    let lastSentValue = 0;
    let lastSentValueSteer = 0;
    let isMoving = false;
    let isMovingSteer = false;
    let isHolding = false;
    
    // 油门摇杆 - 全向控制
    
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
      handle1.style.top = '50%';
      handle1.style.left = '50%';
      handle1.style.transform = 'translate(-50%, -50%)';
      lastSentValue = 0;
      lastSentValueSteer = 0;
      sendControl(0, 0);
      throttleValue.textContent = '0';
      steeringValue.textContent = '0';
    }

    function updatePosition(e) {
      const rect = joystick1.getBoundingClientRect();
      const centerX = rect.width / 2;
      const centerY = rect.height / 2;
      
      // 获取触摸/鼠标位置
      let clientX, clientY;
      if (e.type.includes('touch')) {
        clientX = e.touches[0].clientX;
        clientY = e.touches[0].clientY;
      } else {
        clientX = e.clientX;
        clientY = e.clientY;
      }
      
      // 计算相对于摇杆中心的偏移
      const x = clientX - rect.left;
      const y = clientY - rect.top;
      const offsetX = x - centerX;
      const offsetY = y - centerY;
      
      // 计算距离和角度
      const distance = Math.sqrt(offsetX * offsetX + offsetY * offsetY);
      const maxDistance = rect.width / 2 - handle1.offsetWidth / 2;
      
      // 限制在圆形区域内
      const limitedDistance = Math.min(distance, maxDistance);
      const angle = Math.atan2(offsetY, offsetX);
      
      // 计算新位置
      const newX = centerX + limitedDistance * Math.cos(angle);
      const newY = centerY + limitedDistance * Math.sin(angle);
      
      // 更新手柄位置
      handle1.style.left = `${newX}px`;
      handle1.style.top = `${newY}px`;
      handle1.style.transform = 'translate(-50%, -50%)';
      
      // 计算油门值（上下方向，上 = 负，下 = 正）
      // 归一化为 -100 到 100
      const throttle = -Math.sin(angle) * (limitedDistance / maxDistance) * 100;
      
      // 计算转向值（左右方向，左 = 负，右 = 正）
      const steering = Math.cos(angle) * (limitedDistance / maxDistance) * 100;
      
      // 保存最后的值
      lastSentValue = throttle;
      lastSentValueSteer = steering;
      
      // 更新显示
      throttleValue.textContent = throttle.toFixed(0);
      steeringValue.textContent = steering.toFixed(0);
      
      // 发送控制命令
      sendControl(throttle, steering);
    }
    
    // 转向摇杆 - 全向控制（与油门摇杆功能相同）
    
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
      handle2.style.top = '50%';
      handle2.style.left = '50%';
      handle2.style.transform = 'translate(-50%, -50%)';
      lastSentValue = 0;
      lastSentValueSteer = 0;
      sendControl(0, 0);
      throttleValue.textContent = '0';
      steeringValue.textContent = '0';
    }

    function updatePositionSteer(e) {
      const rect = joystick2.getBoundingClientRect();
      const centerX = rect.width / 2;
      const centerY = rect.height / 2;
      
      // 获取触摸/鼠标位置
      let clientX, clientY;
      if (e.type.includes('touch')) {
        clientX = e.touches[0].clientX;
        clientY = e.touches[0].clientY;
      } else {
        clientX = e.clientX;
        clientY = e.clientY;
      }
      
      // 计算相对于摇杆中心的偏移
      const x = clientX - rect.left;
      const y = clientY - rect.top;
      const offsetX = x - centerX;
      const offsetY = y - centerY;
      
      // 计算距离和角度
      const distance = Math.sqrt(offsetX * offsetX + offsetY * offsetY);
      const maxDistance = rect.width / 2 - handle2.offsetWidth / 2;
      
      // 限制在圆形区域内
      const limitedDistance = Math.min(distance, maxDistance);
      const angle = Math.atan2(offsetY, offsetX);
      
      // 计算新位置
      const newX = centerX + limitedDistance * Math.cos(angle);
      const newY = centerY + limitedDistance * Math.sin(angle);
      
      // 更新手柄位置
      handle2.style.left = `${newX}px`;
      handle2.style.top = `${newY}px`;
      handle2.style.transform = 'translate(-50%, -50%)';
      
      // 计算油门值（上下方向，上 = 负，下 = 正）
      const throttle = -Math.sin(angle) * (limitedDistance / maxDistance) * 100;
      
      // 计算转向值（左右方向，左 = 负，右 = 正）
      const steering = Math.cos(angle) * (limitedDistance / maxDistance) * 100;
      
      // 保存最后的值
      lastSentValue = throttle;
      lastSentValueSteer = steering;
      
      // 更新显示
      throttleValue.textContent = throttle.toFixed(0);
      steeringValue.textContent = steering.toFixed(0);
      
      // 发送控制命令
      sendControl(throttle, steering);
    }
    
    // 转向摇杆 - 全向控制（与油门摇杆功能相同）
    
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
      handle2.style.top = '50%';
      handle2.style.left = '50%';
      handle2.style.transform = 'translate(-50%, -50%)';
      lastSentValue = 0;
      lastSentValueSteer = 0;
      sendControl(0, 0);
      throttleValue.textContent = '0';
      steeringValue.textContent = '0';
    }

    function updatePositionSteer(e) {
      const rect = joystick2.getBoundingClientRect();
      const centerX = rect.width / 2;
      const centerY = rect.height / 2;
      
      // 获取触摸/鼠标位置
      let clientX, clientY;
      if (e.type.includes('touch')) {
        clientX = e.touches[0].clientX;
        clientY = e.touches[0].clientY;
      } else {
        clientX = e.clientX;
        clientY = e.clientY;
      }
      
      // 计算相对于摇杆中心的偏移
      const x = clientX - rect.left;
      const y = clientY - rect.top;
      const offsetX = x - centerX;
      const offsetY = y - centerY;
      
      // 计算距离和角度
      const distance = Math.sqrt(offsetX * offsetX + offsetY * offsetY);
      const maxDistance = rect.width / 2 - handle2.offsetWidth / 2;
      
      // 限制在圆形区域内
      const limitedDistance = Math.min(distance, maxDistance);
      const angle = Math.atan2(offsetY, offsetX);
      
      // 计算新位置
      const newX = centerX + limitedDistance * Math.cos(angle);
      const newY = centerY + limitedDistance * Math.sin(angle);
      
      // 更新手柄位置
      handle2.style.left = `${newX}px`;
      handle2.style.top = `${newY}px`;
      handle2.style.transform = 'translate(-50%, -50%)';
      
      // 计算油门值（上下方向，上 = 负，下 = 正）
      const throttle = -Math.sin(angle) * (limitedDistance / maxDistance) * 100;
      
      // 计算转向值（左右方向，左 = 负，右 = 正）
      const steering = Math.cos(angle) * (limitedDistance / maxDistance) * 100;
      
      // 保存最后的值
      lastSentValue = throttle;
      lastSentValueSteer = steering;
      
      // 更新显示
      throttleValue.textContent = throttle.toFixed(0);
      steeringValue.textContent = steering.toFixed(0);
      
      // 发送控制命令
      sendControl(throttle, steering);
    }
    
    // 辅助函数：映射值的范围
    function map(value, inMin, inMax, outMin, outMax) {
      return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }
    
    // 发送控制命令
    function sendControl(throttle, steering) {
      fetch(`/control?c=${throttle},${steering}`, {
        method: 'GET',
        cache: 'no-cache',
      }).catch(console.error);
    }
    
    // 检查连接状态
    function checkConnection() {
      fetch('/status')
        .then(response => {
          if (response.ok) {
            statusIndicator.classList.add('connected');
            isConnected = true;
          } else {
            statusIndicator.classList.remove('connected');
            isConnected = false;
          }
        })
        .catch(() => {
          statusIndicator.classList.remove('connected');
          isConnected = false;
        });
    }
    
    // 更新传感器数据
    function updateSensorData() {
      if (!isConnected) return;
      
      fetch('/gyro', {
        method: 'GET',
        cache: 'no-cache',
        headers: {
          'Accept': 'application/json'
        }
      })
      .then(response => response.json())
      .then(data => {
        // 更新角度数据
        angleRoll.textContent = data.angle.roll.toFixed(2);
        anglePitch.textContent = data.angle.pitch.toFixed(2);
        angleYaw.textContent = data.angle.yaw.toFixed(2);
        
        // 更新加速度数据
        accX.textContent = data.acc.x.toFixed(2);
        accY.textContent = data.acc.y.toFixed(2);
        accZ.textContent = data.acc.z.toFixed(2);
        
        // 更新角速度数据
        gyroX.textContent = data.gyro.x.toFixed(2);
        gyroY.textContent = data.gyro.y.toFixed(2);
        gyroZ.textContent = data.gyro.z.toFixed(2);
        
        // 更新磁场数据
        magX.textContent = data.mag.x;
        magY.textContent = data.mag.y;
        magZ.textContent = data.mag.z;
        
        // 更新3D模型
        updateModel(data.angle.roll, data.angle.pitch, data.angle.yaw);
        
        // 更新控制状态显示
        if (data.control && data.control.isOutOfControl) {
          // 显示失控警告
          alertContainer.classList.add('active');
          alertMessage.textContent = data.control.reason || '飞行器失控';
          
          // 更新控制状态文本
          controlStatus.textContent = '失控';
          controlStatus.style.color = 'var(--danger-color)';
        } else {
          // 隐藏失控警告
          alertContainer.classList.remove('active');
          
          // 更新控制状态文本
          controlStatus.textContent = '正常';
          controlStatus.style.color = 'var(--success-color)';
        }
      })
      .catch(error => {
        console.error('获取数据失败:', error);
        isConnected = false;
        statusIndicator.classList.remove('connected');
      });
    }
    
    // 更新3D模型角度
    function updateModel(roll, pitch, yaw) {
      model3D.style.transform = `rotateX(${pitch}deg) rotateY(${yaw}deg) rotateZ(${roll}deg)`;
    }
    
    // 角度控制开关
    document.getElementById('angleControlSwitch').addEventListener('change', function(e) {
      const state = e.target.checked ? 'on' : 'off';
      fetch('/angle_control?state=' + state)
        .then(response => response.text())
        .then(data => console.log('角度控制:', data))
        .catch(console.error);
    });
    
    // 一键起飞按钮
    document.getElementById('takeoffButton').addEventListener('click', function() {
      fetch('/takeoff')
        .then(response => response.text())
        .then(data => {
          console.log('起飞响应:', data);
          throttleValue.textContent = '100';
          steeringValue.textContent = '100';
        })
        .catch(error => console.error('起飞请求失败:', error));
    });
    
    // 一键降落按钮
    document.getElementById('landingButton').addEventListener('click', function() {
      fetch('/landing')
        .then(response => response.text())
        .then(data => {
          console.log('降落响应:', data);
          throttleValue.textContent = '-100';
          steeringValue.textContent = '-100';
        })
        .catch(error => console.error('降落请求失败:', error));
    });
    
    // 结束控制按钮
    document.getElementById('stopButton').addEventListener('click', function() {
      fetch('/stop')
        .then(response => response.text())
        .then(data => {
          console.log('停止响应:', data);
          throttleValue.textContent = '0';
          steeringValue.textContent = '0';
          handle1.style.top = '50%';
          handle1.style.left = '50%';
          handle2.style.top = '50%';
          handle2.style.left = '50%';
        })
        .catch(error => console.error('停止请求失败:', error));
    });
    
    // 初始化
    function init() {
      // 检查连接状态
      checkConnection();
      setInterval(checkConnection, 2000);
      
      // 更新传感器数据
      updateSensorData();
      setInterval(updateSensorData, 100);
    }
    
    // 页面加载完成后初始化
    window.addEventListener('DOMContentLoaded', init);
  </script>
</body>
</html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void handleGyroData()
{
  // 返回所有JY901数据：加速度、角速度、角度、磁场
  String json = "{";
  
  // 角度数据
  json += "\"angle\":{";
  json += "\"roll\":" + String(sensorData.angle[0], 2) + ",";
  json += "\"pitch\":" + String(sensorData.angle[1], 2) + ",";
  json += "\"yaw\":" + String(sensorData.angle[2], 2);
  json += "},";
  
  // 加速度数据
  json += "\"acc\":{";
  json += "\"x\":" + String(sensorData.acc[0], 2) + ",";
  json += "\"y\":" + String(sensorData.acc[1], 2) + ",";
  json += "\"z\":" + String(sensorData.acc[2], 2);
  json += "},";
  
  // 角速度数据
  json += "\"gyro\":{";
  json += "\"x\":" + String(sensorData.gyro[0], 2) + ",";
  json += "\"y\":" + String(sensorData.gyro[1], 2) + ",";
  json += "\"z\":" + String(sensorData.gyro[2], 2);
  json += "},";
  
  // 磁场数据
  json += "\"mag\":{";
  json += "\"x\":" + String(sensorData.mag[0]) + ",";
  json += "\"y\":" + String(sensorData.mag[1]) + ",";
  json += "\"z\":" + String(sensorData.mag[2]);
  json += "},";
  
  // 控制状态数据
  json += "\"control\":{";
  json += "\"isOutOfControl\":" + String(isOutOfControl ? "true" : "false") + ",";
  json += "\"reason\":\"" + outOfControlReason + "\"";
  json += "}";
  
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

// 添加飞行器控制状态API
void handleControlStatus()
{
  String json = "{";
  json += "\"isOutOfControl\":" + String(isOutOfControl ? "true" : "false") + ",";
  json += "\"reason\":\"" + outOfControlReason + "\"";
  json += "}";
  
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.send(200, "application/json", json);
}

void handleAngleControl()
{
  String state = server.arg("state");
  if (state == "on")
  {
    // 先关闭其他模式
    isTakingOff = false;
    isLanding = false;
    
    // 停止A0090舵机
    steeringServo.writeSpeed(0);
    
    // 启用姿态控制
    angleControlEnabled = true;
    Serial.println("倾角控制已启用");
    server.send(200, "text/plain", "Angle control enabled");
  }
  else if (state == "off")
  {
    angleControlEnabled = false;
    rxC1 = 0; // 停止电机
    rxC2 = 0; // 转向回中
    
    // 停止A0090舵机
    steeringServo.writeSpeed(0);
    
    updateServo();
    Serial.println("倾角控制已禁用");
    server.send(200, "text/plain", "Angle control disabled");
  }
}

// 添加结束控制的处理函数
void handleStop() {
  Serial.println("执行结束控制命令");
  
  // 将所有控制回到中立位置
  rxC1 = 0;
  rxC2 = 0;
  
  // 重置状态标志
  isTakingOff = false;
  isLanding = false;
  angleControlEnabled = false; // 确保姿态控制也被关闭
  
  // A0090舵机停止
  steeringServo.writeSpeed(0);
  
  // 立即更新舵机
  updateServo();
  
  server.send(200, "text/plain", "Control stopped");
}

// 修改一键起飞函数，确保两个舵机协调动作
void handleTakeoff() {
  Serial.println("执行一键起飞命令");
  
  // 设置主动力舵机最大值
  rxC1 = 100; // 最大值
  
  // 设置左舵机（主方向舵机）正转
  rxC2 = 100; // 最大值
  
  // A0090舵机先停止，再反方向旋转
  steeringServo.writeSpeed(0);
  delay(100); // 短暂延迟确保停止
  
  // 短暂反方向旋转到位置后停止
  steeringServo.writeSpeed(-70); // 以70%速度反方向转动
  delay(500); // 旋转500ms
  steeringServo.writeSpeed(0);  // 停止
  
  // 设置起飞状态
  isTakingOff = true;
  
  // 立即更新舵机
  updateServo();
  
  server.send(200, "text/plain", "Takeoff command executed");
}

// 修改一键降落函数，确保两个舵机协调动作
void handleLanding() {
  Serial.println("执行一键降落命令");
  
  // 设置主动力舵机最小值
  rxC1 = -100; // 最小值
  
  // 设置左舵机（主方向舵机）反转
  rxC2 = -100; // 最小值
  
  // A0090舵机先停止，再正方向旋转
  steeringServo.writeSpeed(0);
  delay(100); // 短暂延迟确保停止
  
  // 短暂正方向旋转到位置后停止
  steeringServo.writeSpeed(70); // 以70%速度正方向转动
  delay(500); // 旋转500ms
  steeringServo.writeSpeed(0);  // 停止
  
  // 重置起飞状态
  isTakingOff = false;
  
  // 设置降落状态
  isLanding = true;
  
  // 立即更新舵机
  updateServo();
  
  server.send(200, "text/plain", "Landing command executed");
}

void registerEvent()
{
  server.on("/", handleWebPage);
  server.on("/control", handleRoot);
  server.on("/gyro", handleGyroData);
  server.on("/status", handleStatus);
  server.on("/angle_control", handleAngleControl);
  server.on("/takeoff", handleTakeoff); // 添加起飞端点
  server.on("/landing", handleLanding); // 添加降落端点
  server.on("/stop", handleStop);       // 添加结束控制端点
  server.on("/control_status", handleControlStatus); // 添加控制状态端点

  server.enableCORS();
  server.begin();
  Serial.println("HTTP server started");
}

void setup()
{
  Serial.begin(115200); // 提高串口速率增强稳定性
  delay(100); // 等待串口稳定
  Serial.println("\n\n=== ESP32S3车辆控制系统启动 ===");
  Serial.println("正在初始化硬件...");

  // 检查ESP32S3硬件信息
  checkAndPrintESP32Info();
  
  // 配置电源管理以获得稳定PWM
  configureESP32S3PowerManagement();

  Wire.begin(19, 18); // SDA=19, SCL=18
  Serial.println("I2C初始化完成");

  // 初始化JY901串口
  jy901Serial.begin(9600, SERIAL_8N1, 37, 36); // RX=GPIO37, TX=GPIO36
  Serial.println("JY901串口初始化完成");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // 测试ESP32S3 PWM功能
  testPWM();

  Serial.println("\n初始化舵机控制...");
  
  rxC1Servo.setup(100, 10, 8);
  rxC1Servo.attachPin(42); // 动力电机控制信号
  rxC1Servo.setScale(1);
  rxC1Servo.write(0, -1, 1);

  rxC1ServoHaf.setup(100, 10, 9);
  rxC1ServoHaf.attachPin(41); // 动力电机控制信号（半强度）
  rxC1ServoHaf.setScale(1.5);
  rxC1ServoHaf.write(0, -1, 1);

  rxC2Servo.setup(500, 10, 10);
  rxC2Servo.attachPin(40); // 方向舵机信号
  rxC2Servo.setScale(1.5); // 增加转向角度范围从0.7到0.9
  rxC2Servo.write(0, -1, 1);

  rxC2ServoHaf.setup(500, 10, 11);
  rxC2ServoHaf.attachPin(25); // 方向舵机信号（半强度）
  rxC2ServoHaf.setScale(0.4); // 增加半强度舵机的转向范围从0.3到0.4
  rxC2ServoHaf.write(0, -1, 1);

  // 初始化A0090全向舵机（GPIO33）- 使用连续旋转舵机类型
  steeringServo.setup(50, 8, 12, 1); // 参数：频率, 分辨率, 通道, 类型(1=连续旋转)
  steeringServo.attachPin(33);       // 使用GPIO33（A0090舵机）
  steeringServo.setScale(1);         // 设置为1以支持全向转动
  steeringServo.writeSpeed(0);       // 初始化时停止舵机

  Serial.println("舵机初始化完成");
  
  Serial.println("\n配置WiFi接入点...");
  
  // WiFi设置
  WiFi.softAP("xiaocheche", "12345678");
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP地址: ");
  Serial.println(myIP);

  if (MDNS.begin("esp32"))
  {
    Serial.println("MDNS响应器已启动");
  }

  registerEvent();
  Serial.println("\n=== 初始化完成，系统就绪 ===\n");
}

// 添加测试函数，用于验证ESP32S3 PWM输出
void testPWM() {
  Serial.println("\n=== 开始PWM测试 ===");
  
  // 测试引脚和通道
  const int testPin = 42;
  const int testChannel = 7;
  
  // 测试不同频率
  Serial.println("测试50Hz PWM输出...");
  
  // 使用ESP-IDF API直接配置
  bool success = setupLedcESP_IDF(testPin, testChannel, 50, 10);
  if (success) {
    Serial.println("PWM配置成功");
    
    // 测试不同占空比
    const uint32_t maxDuty = (1 << 10) - 1; // 10位分辨率的最大值
    
    // 测试25%占空比
    uint32_t duty25 = maxDuty / 4;
    Serial.print("设置25%占空比 (");
    Serial.print(duty25);
    Serial.print("/");
    Serial.print(maxDuty);
    Serial.println(")");
    setDutyESP_IDF(testChannel, duty25, 10);
    delay(2000);
    
    // 测试50%占空比
    uint32_t duty50 = maxDuty / 2;
    Serial.print("设置50%占空比 (");
    Serial.print(duty50);
    Serial.print("/");
    Serial.print(maxDuty);
    Serial.println(")");
    setDutyESP_IDF(testChannel, duty50, 10);
    delay(2000);
    
    // 测试75%占空比
    uint32_t duty75 = (maxDuty * 3) / 4;
    Serial.print("设置75%占空比 (");
    Serial.print(duty75);
    Serial.print("/");
    Serial.print(maxDuty);
    Serial.println(")");
    setDutyESP_IDF(testChannel, duty75, 10);
    delay(2000);
    
    // 测试改变频率
    Serial.println("\n测试100Hz PWM输出...");
    success = setupLedcESP_IDF(testPin, testChannel, 100, 10);
    if (success) {
      setDutyESP_IDF(testChannel, duty50, 10);
      delay(2000);
    }
  } else {
    Serial.println("PWM配置失败");
  }
  
  Serial.println("=== PWM测试完成 ===\n");
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

// 解析JY901数据
void parseJY901Data() {
  // JY901数据格式: 0x55 + ID + DATA(5bytes)
  // 数据帧长度固定为11字节
  for (int i = 0; i < ucRxCnt; i++) {
    if (ucRxBuffer[i] == 0x55 && i + 10 < ucRxCnt) {
      switch (ucRxBuffer[i+1]) {
        case 0x51: // 加速度数据
          sensorData.acc[0] = ((short)(ucRxBuffer[i+3]<<8 | ucRxBuffer[i+2])) / 32768.0 * 16;
          sensorData.acc[1] = ((short)(ucRxBuffer[i+5]<<8 | ucRxBuffer[i+4])) / 32768.0 * 16;
          sensorData.acc[2] = ((short)(ucRxBuffer[i+7]<<8 | ucRxBuffer[i+6])) / 32768.0 * 16;
          break;
        case 0x52: // 角速度数据
          sensorData.gyro[0] = ((short)(ucRxBuffer[i+3]<<8 | ucRxBuffer[i+2])) / 32768.0 * 2000;
          sensorData.gyro[1] = ((short)(ucRxBuffer[i+5]<<8 | ucRxBuffer[i+4])) / 32768.0 * 2000;
          sensorData.gyro[2] = ((short)(ucRxBuffer[i+7]<<8 | ucRxBuffer[i+6])) / 32768.0 * 2000;
          break;
        case 0x53: // 角度数据
          sensorData.angle[0] = ((short)(ucRxBuffer[i+3]<<8 | ucRxBuffer[i+2])) / 32768.0 * 180;
          sensorData.angle[1] = ((short)(ucRxBuffer[i+5]<<8 | ucRxBuffer[i+4])) / 32768.0 * 180;
          sensorData.angle[2] = ((short)(ucRxBuffer[i+7]<<8 | ucRxBuffer[i+6])) / 32768.0 * 180;
          break;
        case 0x54: // 磁场数据
          sensorData.mag[0] = ((short)(ucRxBuffer[i+3]<<8 | ucRxBuffer[i+2]));
          sensorData.mag[1] = ((short)(ucRxBuffer[i+5]<<8 | ucRxBuffer[i+4]));
          sensorData.mag[2] = ((short)(ucRxBuffer[i+7]<<8 | ucRxBuffer[i+6]));
          break;
      }
    }
  }
  
  // 清空缓冲区
  if (ucRxCnt > 100) ucRxCnt = 0;
}

// 处理JY901数据
void processJY901Data() {
  // 读取并解析JY901数据
  while (jy901Serial.available()) {
    ucRxBuffer[ucRxCnt++] = jy901Serial.read();
    if (ucRxCnt >= 250) ucRxCnt = 0;
    
    // 解析JY901数据
    parseJY901Data();
  }
  
  // 更新滤波后的角度数据，用于控制和显示
  filteredAx = sensorData.angle[0]; // Roll角 (绕X轴)
  filteredAy = sensorData.angle[1]; // Pitch角 (绕Y轴)
  filteredAz = sensorData.angle[2]; // Yaw角 (绕Z轴)
  
  // 每秒打印一次当前角度（避免Serial过载）
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 1000)
  {
    lastPrintTime = millis();
    Serial.print("JY901角度 - Roll(X): ");
    Serial.print(filteredAx);
    Serial.print(" Pitch(Y): ");
    Serial.print(filteredAy);
    Serial.print(" Yaw(Z): ");
    Serial.println(filteredAz);
  }
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

// 修改处理姿态控制的函数来实现X轴同时控制两个舵机
void processAngleControl()
{
  // 如果倾角控制未启用，直接返回
  if (!angleControlEnabled)
  {
    return;
  }

  // 添加调试输出
  Serial.println("倾角控制激活中...");
  Serial.print("Roll角(X): ");
  Serial.print(filteredAx);
  Serial.print(" Pitch角(Y): ");
  Serial.print(filteredAy);
  Serial.print(" Yaw角(Z): ");
  Serial.println(filteredAz);

  // 获取当前Roll角和Pitch角
  float currentRoll = filteredAx;
  float currentPitch = filteredAy;
  bool wasOutOfControl = isOutOfControl; // 保存之前的失控状态
  isOutOfControl = false; // 重置失控状态，由下面的逻辑决定是否设为失控
  
  // 处理Roll角度 (0-90度) 控制无刷电机 - 使用PID控制
  if (currentRoll >= 0 && currentRoll <= 90)
  {
    // Roll在正常范围内，使用PID控制无刷电机
    // 将Roll角映射到PID的目标值，这里假设Roll=45度对应目标值0
    // 即Roll=0时最小速度，Roll=90时最大速度
    float targetSpeed = map(currentRoll, 0, 90, minSpeed/2, maxSpeed);
    
    // 使用PID计算油门值
    float controlOutput = throttlePID.compute(targetSpeed, currentThrottle);
    rxC1 = controlOutput;
    
    // 更新当前实际值，用于下一次PID计算
    currentThrottle = rxC1;
    
    Serial.print("Roll正常范围，PID控制油门值: ");
    Serial.println(rxC1);
  }
  else
  {
    // Roll角超出范围，设置电机停止并标记失控
    rxC1 = 0;
    isOutOfControl = true;
    outOfControlReason = "Roll角度超出范围(0-90°)";
    Serial.println("警告：Roll角度超出范围，电机停止");
    
    // 重置PID控制器，防止积分项累积
    throttlePID.reset();
    currentThrottle = 0;
  }

  // 处理Pitch角度 (-45到45度) 控制舵机
  if (currentPitch >= -45 && currentPitch <= 45)
  {
    // Pitch在正常范围内，控制舵机
    // 将Pitch角映射到舵机控制范围
    rxC2 = map(currentPitch, -45, 45, -100, 100);
    
    // 控制A0090舵机 - 方向与主舵机相反
    float servoSpeed = map(currentPitch, -45, 45, 70, -70);
    steeringServo.writeSpeed(servoSpeed);
    
    // 更新当前实际值
    currentSteering = rxC2;
    
    Serial.print("Pitch正常范围，设置舵机值: ");
    Serial.println(rxC2);
  }
  else
  {
    // Pitch角超出范围，舵机回中并标记失控
    rxC2 = 0;
    steeringServo.writeSpeed(0);
    isOutOfControl = true;
    outOfControlReason = "Pitch角度超出范围(-45°到45°)";
    Serial.println("警告：Pitch角度超出范围，舵机回中");
    
    // 重置舵机控制相关变量
    steeringPID.reset();
    currentSteering = 0;
  }

  // 如果失控状态发生变化，输出日志
  if (isOutOfControl != wasOutOfControl)
  {
    if (isOutOfControl)
    {
      Serial.println("警告：飞行器失控！原因: " + outOfControlReason);
    }
    else
    {
      Serial.println("飞行器恢复正常控制");
    }
  }

  // 更新所有舵机
  updateServo();
}

// 在loop函数中添加防止A0090舵机随机转动的保护措施
void loop()
{
  static unsigned long lastProcessTime = 0;
  static unsigned long lastMPUTime = 0;
  static unsigned long lastServoCheckTime = 0; // 新增舵机检查时间
  const unsigned long processInterval = 1;
  const unsigned long mpuInterval = 20;
  const unsigned long servoCheckInterval = 1000; // 每1秒检查一次舵机状态

  unsigned long currentTime = millis();

  // 处理遥感信号
  if (currentTime - lastProcessTime >= processInterval)
  {
    if (!angleControlEnabled)
    { // 只在倾角控制关闭时处理遥感信号
      processSignals();
    }
    lastProcessTime = currentTime;
  }

  // JY901数据处理和倾角控制
  if (currentTime - lastMPUTime >= mpuInterval)
  {
    processJY901Data();
    if (angleControlEnabled)
    { // 在倾角控制开启时处理角度控制
      processAngleControl();
    }
    lastMPUTime = currentTime;
  }
  
  // 处理降落时的减速操作
  if (isLanding) {
    if (currentTime - lastLandingUpdateTime >= 50) { // 每50ms更新一次
      // 如果rxC1大于-100，逐渐减小值（减速）
      if (rxC1 > -100) {
        rxC1 -= 5; // 每次减少5
        if (rxC1 < -100) rxC1 = -100;
        
        // 更新舵机
        updateServo();
        lastLandingUpdateTime = currentTime;
        
        Serial.print("降落减速中: ");
        Serial.println(rxC1);
      } else {
        // 达到停止状态，关闭降落模式
        isLanding = false;
        Serial.println("降落完成");
      }
    }
  }
  
  // 定期检查A0090舵机状态，防止随机转动
  if (currentTime - lastServoCheckTime >= servoCheckInterval) {
    if (!angleControlEnabled && !isTakingOff && !isLanding) {
      // 只有在非控制状态下执行此操作
      steeringServo.writeSpeed(0); // 确保A0090舵机停止
    }
    lastServoCheckTime = currentTime;
  }

  server.handleClient();
  yield();
}