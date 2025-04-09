#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// WiFi配置
const char *ssid = "ovo";
const char *password = "twx20051";

// 服务器配置
const char *serverIP = "192.168.133.254";
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

void setup()
{
  // 初始化串口
  Serial.begin(115200);

  // 初始化I2C - 使用ESP32-S3的默认I2C引脚
  Wire.begin(8, 9); // SDA = GPIO8, SCL = GPIO9
  delay(100);       // 等待I2C稳定

  // 初始化OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306初始化失败"));
    while (1)
      ;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();

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
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  // 连接WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi已连接");

  // 连接TCP服务器
  connectServer();

  // 初始化MPU6050后添加零漂校准
  Serial.println("开始陀螺仪零漂校准...");
  calibrateGyro();
  Serial.println("陀螺仪零漂校准完成");
}

void loop()
{
  unsigned long currentTime = millis();

  // 读取MPU6050数据
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 将原始数据转换为实际值（加速度单位：g，陀螺仪单位：度/秒）
  const float accelScale = 1.0 / 16384.0; // ±2g范围
  float normAx = ax * accelScale;
  float normAy = ay * accelScale;
  float normAz = az * accelScale;

  // 简单的低通滤波
  const float alpha = 0.2;
  filteredAx = alpha * normAx + (1 - alpha) * filteredAx;
  filteredAy = alpha * normAy + (1 - alpha) * filteredAy;
  filteredAz = alpha * normAz + (1 - alpha) * filteredAz;

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

  // 检查TCP连接
  if (!client.connected())
  {
    connectServer();
  }
}

void connectServer()
{
  Serial.println("正在连接服务器...");
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("正在连接服务器...");
  display.display();

  int retryCount = 0;
  while (!client.connect(serverIP, serverPort) && retryCount < 5)
  {
    delay(1000);
    Serial.print(".");
    retryCount++;
  }

  if (client.connected())
  {
    Serial.println("\n服务器已连接");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("服务器已连接");
    display.display();
  }
  else
  {
    Serial.println("\n服务器连接失败");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("服务器连接失败");
    display.display();
  }
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
  // 读取MPU6050数据
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 将原始数据转换为实际值
  const float accelScale = 1.0 / 16384.0; // ±2g范围
  const float gyroScale = 1.0 / 131.0;    // ±250°/s范围

  float accX = ax * accelScale;
  float accY = ay * accelScale;
  float accZ = az * accelScale;

  // 应用零漂补偿
  float gyroX = gx * gyroScale - gyroXOffset;
  float gyroY = gy * gyroScale - gyroYOffset;
  float gyroZ = gz * gyroScale - gyroZOffset;

  // 静态检测
  if (isStatic(accX, accY, accZ, gyroX, gyroY, gyroZ))
  {
    // 在静止状态下，将角速度强制设为0
    gyroX = 0;
    gyroY = 0;
    gyroZ = 0;
  }

  // 创建JSON格式的数据
  char jsonData[256];
  snprintf(jsonData, sizeof(jsonData),
           "m:%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", // 格式: m:ax,ay,az,gx,gy,gz
           accX, accY, accZ, gyroX, gyroY, gyroZ);

  // 发送数据到服务器并打印调试信息
  if (client.connected())
  {
    client.println(jsonData);
    Serial.print("Sent: ");
    Serial.println(jsonData);
  }

  // 更新OLED显示
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // 显示标题
  display.setCursor(0, 0);
  display.println("MPU6050 Data:");

  // 显示加速度数据
  char buffer[32];
  display.setCursor(0, 16);
  snprintf(buffer, sizeof(buffer), "aX: %.2f", accX);
  display.println(buffer);

  display.setCursor(0, 26);
  snprintf(buffer, sizeof(buffer), "aY: %.2f", accY);
  display.println(buffer);

  display.setCursor(0, 36);
  snprintf(buffer, sizeof(buffer), "aZ: %.2f", accZ);
  display.println(buffer);

  // 显示角速度数据
  display.setCursor(64, 16);
  snprintf(buffer, sizeof(buffer), "gX: %.1f", gyroX);
  display.println(buffer);

  display.setCursor(64, 26);
  snprintf(buffer, sizeof(buffer), "gY: %.1f", gyroY);
  display.println(buffer);

  display.setCursor(64, 36);
  snprintf(buffer, sizeof(buffer), "gZ: %.1f", gyroZ);
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

void updateOLED()
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
  snprintf(buffer, sizeof(buffer), "X: %.2f g", filteredAx);
  display.println(buffer);

  display.setCursor(0, 26);
  snprintf(buffer, sizeof(buffer), "Y: %.2f g", filteredAy);
  display.println(buffer);

  display.setCursor(0, 36);
  snprintf(buffer, sizeof(buffer), "Z: %.2f g", filteredAz);
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