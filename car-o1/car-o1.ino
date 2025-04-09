#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// WiFi配置
const char *ssid = "ovo";
const char *password = "twx20051";

// 服务器配置
const char *serverIP = "192.168.235.254";
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

void setup()
{
  // 初始化串口
  Serial.begin(115200);

  // 初始化I2C
  Wire.begin();
  delay(100); // 等待I2C稳定

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
  Serial.println("Connecting...");
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Connecting...");
  display.display();

  while (!client.connect(serverIP, serverPort))
  {
    Serial.println("Failed, retry...");
    delay(1000);
  }

  Serial.println("Connected!");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connected!");
  display.display();
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

  float gyroX = gx * gyroScale;
  float gyroY = gy * gyroScale;
  float gyroZ = gz * gyroScale;

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