/*
 * ESP32读取JY901陀螺仪数据并通过网页显示
 * 
 * 连接方式：
 *     ESP32              JY901
 *      3.3V     ----      VCC
 *      RX2 (GPIO16)  ---  TX
 *      TX2 (GPIO17)  ---  RX
 *      GND      ----      GND
 * 
 * 功能：
 * 1. ESP32创建WiFi热点(AP模式)
 * 2. 设置静态IP为192.168.4.1
 * 3. 读取JY901陀螺仪数据
 * 4. 通过网页实时显示陀螺仪数据
 */

#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>

// WiFi设置
const char* ssid = "ESP32_JY901";  // WiFi热点名称
const char* password = "12345678"; // WiFi密码

// 创建网页服务器，端口号80
WebServer server(80);

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

void setup() {
  // 初始化串口监视器
  Serial.begin(115200);
  
  // 初始化JY901串口
  jy901Serial.begin(9600, SERIAL_8N1, 16, 17); // RX=GPIO16, TX=GPIO17
  
  // 设置WiFi为AP模式
  WiFi.mode(WIFI_AP);
  
  // 配置静态IP
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  
  // 配置AP
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password);
  
  Serial.println("WiFi热点已创建");
  Serial.print("IP地址: ");
  Serial.println(WiFi.softAPIP());
  
  // 设置网页服务器路由
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/styles.css", handleCSS);
  server.on("/script.js", handleJS);
  
  // 启动服务器
  server.begin();
  Serial.println("HTTP服务器已启动");
}

void loop() {
  // 处理HTTP客户端请求
  server.handleClient();
  
  // 读取JY901数据
  while (jy901Serial.available()) {
    ucRxBuffer[ucRxCnt++] = jy901Serial.read();
    if (ucRxCnt >= 250) ucRxCnt = 0;
    
    // 解析JY901数据
    parseJY901Data();
  }
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

// 处理根目录请求，返回HTML页面
void handleRoot() {
  String html = R"html(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>JY901 陀螺仪数据监控</title>
  <link rel="stylesheet" href="/styles.css">
</head>
<body>
  <div class="container">
    <h1>JY901 陀螺仪数据监控</h1>
    
    <div class="data-container">
      <div class="data-group">
        <h2>加速度 (g)</h2>
        <div class="data-row">
          <span>X: </span><span id="accX">0.000</span>
        </div>
        <div class="data-row">
          <span>Y: </span><span id="accY">0.000</span>
        </div>
        <div class="data-row">
          <span>Z: </span><span id="accZ">0.000</span>
        </div>
      </div>
      
      <div class="data-group">
        <h2>角速度 (°/s)</h2>
        <div class="data-row">
          <span>X: </span><span id="gyroX">0.000</span>
        </div>
        <div class="data-row">
          <span>Y: </span><span id="gyroY">0.000</span>
        </div>
        <div class="data-row">
          <span>Z: </span><span id="gyroZ">0.000</span>
        </div>
      </div>
      
      <div class="data-group">
        <h2>角度 (°)</h2>
        <div class="data-row">
          <span>Roll: </span><span id="angleX">0.000</span>
        </div>
        <div class="data-row">
          <span>Pitch: </span><span id="angleY">0.000</span>
        </div>
        <div class="data-row">
          <span>Yaw: </span><span id="angleZ">0.000</span>
        </div>
      </div>
      
      <div class="data-group">
        <h2>磁场</h2>
        <div class="data-row">
          <span>X: </span><span id="magX">0</span>
        </div>
        <div class="data-row">
          <span>Y: </span><span id="magY">0</span>
        </div>
        <div class="data-row">
          <span>Z: </span><span id="magZ">0</span>
        </div>
      </div>
    </div>
    
    <div class="model-container">
      <div class="model-wrapper">
        <div class="model" id="model"></div>
      </div>
    </div>
  </div>
  <script src="/script.js"></script>
</body>
</html>
  )html";
  
  server.send(200, "text/html", html);
}

// 处理数据请求，返回JSON格式的传感器数据
void handleData() {
  String json = "{";
  json += "\"acc\":[" + String(sensorData.acc[0], 3) + "," + String(sensorData.acc[1], 3) + "," + String(sensorData.acc[2], 3) + "],";
  json += "\"gyro\":[" + String(sensorData.gyro[0], 3) + "," + String(sensorData.gyro[1], 3) + "," + String(sensorData.gyro[2], 3) + "],";
  json += "\"angle\":[" + String(sensorData.angle[0], 3) + "," + String(sensorData.angle[1], 3) + "," + String(sensorData.angle[2], 3) + "],";
  json += "\"mag\":[" + String(sensorData.mag[0]) + "," + String(sensorData.mag[1]) + "," + String(sensorData.mag[2]) + "]";
  json += "}";
  
  server.send(200, "application/json", json);
}

// 处理CSS请求，返回样式表
void handleCSS() {
  String css = R"css(
* {
  margin: 0;
  padding: 0;
  box-sizing: border-box;
}

body {
  font-family: Arial, sans-serif;
  background-color: #f0f0f0;
  padding: 20px;
}

.container {
  max-width: 1200px;
  margin: 0 auto;
  background-color: white;
  border-radius: 10px;
  padding: 20px;
  box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
}

h1 {
  text-align: center;
  margin-bottom: 30px;
  color: #333;
}

.data-container {
  display: flex;
  flex-wrap: wrap;
  justify-content: space-around;
  margin-bottom: 30px;
}

.data-group {
  background-color: #f8f8f8;
  border-radius: 8px;
  padding: 15px;
  margin: 10px;
  min-width: 200px;
  box-shadow: 0 2px 5px rgba(0, 0, 0, 0.05);
}

.data-group h2 {
  font-size: 1.2rem;
  margin-bottom: 15px;
  color: #444;
  text-align: center;
}

.data-row {
  display: flex;
  justify-content: space-between;
  margin-bottom: 8px;
  font-size: 1.1rem;
}

.model-container {
  width: 100%;
  display: flex;
  justify-content: center;
  margin-top: 20px;
}

.model-wrapper {
  width: 300px;
  height: 300px;
  perspective: 800px;
}

.model {
  width: 100%;
  height: 100%;
  position: relative;
  transform-style: preserve-3d;
  transition: transform 0.1s;
}

.model::before {
  content: '';
  position: absolute;
  width: 200px;
  height: 100px;
  background-color: #3498db;
  transform: rotateX(90deg) translateZ(50px);
  top: 50px;
  left: 50px;
}

.model::after {
  content: '';
  position: absolute;
  width: 20px;
  height: 200px;
  background-color: #e74c3c;
  transform: translateZ(100px);
  top: 50px;
  left: 140px;
}

@media (max-width: 768px) {
  .data-container {
    flex-direction: column;
  }
  
  .data-group {
    width: 100%;
  }
}
  )css";
  
  server.send(200, "text/css", css);
}

// 处理JavaScript请求，返回客户端脚本
void handleJS() {
  String js = R"js(
// 获取数据元素
const accX = document.getElementById('accX');
const accY = document.getElementById('accY');
const accZ = document.getElementById('accZ');
const gyroX = document.getElementById('gyroX');
const gyroY = document.getElementById('gyroY');
const gyroZ = document.getElementById('gyroZ');
const angleX = document.getElementById('angleX');
const angleY = document.getElementById('angleY');
const angleZ = document.getElementById('angleZ');
const magX = document.getElementById('magX');
const magY = document.getElementById('magY');
const magZ = document.getElementById('magZ');
const model = document.getElementById('model');

// 定期获取数据
function fetchData() {
  fetch('/data')
    .then(response => response.json())
    .then(data => {
      // 更新显示数据
      accX.textContent = data.acc[0].toFixed(3);
      accY.textContent = data.acc[1].toFixed(3);
      accZ.textContent = data.acc[2].toFixed(3);
      
      gyroX.textContent = data.gyro[0].toFixed(3);
      gyroY.textContent = data.gyro[1].toFixed(3);
      gyroZ.textContent = data.gyro[2].toFixed(3);
      
      angleX.textContent = data.angle[0].toFixed(3);
      angleY.textContent = data.angle[1].toFixed(3);
      angleZ.textContent = data.angle[2].toFixed(3);
      
      magX.textContent = data.mag[0];
      magY.textContent = data.mag[1];
      magZ.textContent = data.mag[2];
      
      // 更新3D模型角度
      updateModel(data.angle[0], data.angle[1], data.angle[2]);
    })
    .catch(error => console.error('获取数据失败:', error));
}

// 更新3D模型
function updateModel(roll, pitch, yaw) {
  model.style.transform = `rotateX(${pitch}deg) rotateY(${yaw}deg) rotateZ(${roll}deg)`;
}

// 页面加载后开始获取数据
document.addEventListener('DOMContentLoaded', () => {
  // 每100毫秒获取一次数据
  setInterval(fetchData, 100);
});
  )js";
  
  server.send(200, "application/javascript", js);
} 