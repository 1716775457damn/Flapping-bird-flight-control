#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// 添加在文件开头的常量定义部分
const float TILT_THRESHOLD = 30.0;  // 倾斜保护阈值（度）
const float MIN_THROTTLE = 25.0;    // 最小油门值
const float MAX_THROTTLE = 70.0;    // 最大油门值
const float DEADZONE = 10.0;        // 死区范围（度）

// 添加电调PWM相关常量
const int ESC_MIN_PULSE = 1000;     // 最小脉冲宽度（微秒）
const int ESC_MAX_PULSE = 2000;     // 最大脉冲宽度（微秒）

// 创建MPU6050对象
MPU6050 mpu;

// 定义PWM参数
const int PWM_FREQ = 50;       // 50Hz
const int PWM_RESOLUTION = 12; // 使用12位分辨率
const int PWM_MAX = 4096;      // 2^12 - 1

// 舵机PWM参数（根据实际舵机调整）
const int SERVO_MIN_US = 544;  // 标准舵机最小脉冲
const int SERVO_MAX_US = 2400; // 标准舵机最大脉冲
const int SERVO_MID_US = 1450; // 中间位置脉冲宽度(微秒) - 90度

// 定义舵机引脚和通道
const int SERVO_PIN = 17;     // 使用GPIO17
const int SERVO_CHANNEL = 0;  // 使用PWM通道0

// MPU6050数据
int16_t ax, ay, az;
int16_t gx, gy, gz;

// 舵机角度范围
const int SERVO_MIN = 0;
const int SERVO_MAX = 180;
const int SERVO_CENTER = 90;

// PID参数
struct PIDController
{
    float Kp = 2.0;
    float Ki = 0.1;
    float Kd = 0.5;
    float previousError = 0;
    float integral = 0;
    float maxIntegral = 50.0; // 添加积分限幅
    float deadband = 1.0;     // 添加死区

    void reset()
    {
        previousError = 0;
        integral = 0;
    }

    float compute(float setpoint, float input, float dt)
    {
        float error = setpoint - input;

        // 死区处理
        if (abs(error) < deadband)
        {
            return 0;
        }

        integral += error * dt;
        integral = constrain(integral, -maxIntegral, maxIntegral);

        float derivative = (error - previousError) / dt;
        previousError = error;

        return (Kp * error) + (Ki * integral) + (Kd * derivative);
    }
};

// 为前后、左右方向创建PID控制器
PIDController pidX; // 前后方向
PIDController pidY; // 左右方向

// PID计算函数
float computePID(PIDController &pid, float setpoint, float input)
{
    float error = setpoint - input;
    pid.integral += error;
    float derivative = error - pid.previousError;

    // 限制积分项，防止积分饱和
    pid.integral = constrain(pid.integral, -100, 100);

    float output = (pid.Kp * error) + (pid.Ki * pid.integral) + (pid.Kd * derivative);
    pid.previousError = error;

    return output;
}

// 卡尔曼滤波器结构体
struct KalmanFilter
{
    float Q_angle = 0.001;  // 过程噪声协方差
    float Q_bias = 0.003;   // 过程噪声协方差
    float R_measure = 0.03; // 测量噪声协方差

    float angle = 0; // 角度
    float bias = 0;  // 角度偏差
    float rate = 0;  // 角速度

    float P[2][2] = {{0, 0}, {0, 0}}; // 误差协方差矩阵

    float getAngle(float newAngle, float newRate, float dt)
    {
        // 预测
        rate = newRate - bias;
        angle += dt * rate;

        // 更新误差协方差矩阵
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // 计算卡尔曼增益
        float S = P[0][0] + R_measure;
        float K[2] = {P[0][0] / S, P[1][0] / S};

        // 测量更新
        float y = newAngle - angle;
        angle += K[0] * y;
        bias += K[1] * y;

        // 更新误差协方差矩阵
        float P00_temp = P[0][0];
        float P01_temp = P[0][1];

        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle;
    }
};

// 创建X轴和Y轴的卡尔曼滤波器
KalmanFilter kalmanX;
KalmanFilter kalmanY;

// 时间相关变量
unsigned long previousTime = 0;
float dt = 0;

// 添加控制模式枚举（移到文件开头）
enum ControlMode
{
    MODE_NORMAL, // 普通模式
    MODE_SMOOTH, // 平滑模式
    MODE_PRECISE // 精确模式
};

// 添加模式相关变量（也移到文件开头）
ControlMode currentMode = MODE_NORMAL;
const float SENSITIVITY[] = {1.0, 0.5, 2.0}; // 不同模式的灵敏度

// 声明getModeMultiplier函数原型
float getModeMultiplier(ControlMode mode);

// 修改PWM控制函数
void setServoPWM(float angleDegrees)
{
    // 限制角度范围
    angleDegrees = constrain(angleDegrees, 0, 180);
    
    // 计算占空比
    int duty = map(angleDegrees, 0, 180, 102, 512);
    
    // 输出调试信息
    Serial.print("Angle:");
    Serial.print(angleDegrees);
    Serial.print(" Duty:");
    Serial.println(duty);
    
    ledcWrite(SERVO_CHANNEL, duty);
}

// 添加移动平均滤波
const int FILTER_SAMPLES = 5;
float angleXSamples[FILTER_SAMPLES];
float angleYSamples[FILTER_SAMPLES];
int sampleIndex = 0;

float getFilteredAngle(float newValue, float *samples)
{
    samples[sampleIndex] = newValue;
    float sum = 0;
    for (int i = 0; i < FILTER_SAMPLES; i++)
    {
        sum += samples[i];
    }
    sampleIndex = (sampleIndex + 1) % FILTER_SAMPLES;
    return sum / FILTER_SAMPLES;
}

// OLED显示屏设置
#define SCREEN_WIDTH 128    // OLED显示宽度，像素
#define SCREEN_HEIGHT 64    // OLED显示高度，像素
#define OLED_RESET -1       // Reset pin
#define SCREEN_ADDRESS 0x3C // I2C地址，通常是0x3C或0x3D

// 创建显示对象
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 修改SystemStatus结构体
struct SystemStatus
{
    bool mpuOK = false;
    bool servosOK = false;
    unsigned long lastUpdateTime = 0;
    float batteryVoltage = 0;
    float angleX = 0;
    float angleY = 0;

    void update(float x, float y)
    {
        lastUpdateTime = millis();
        angleX = x;
        angleY = y;
        // 可以添加电池电压监测等
    }

    void print()
    {
        Serial.print("System Status: MPU=");
        Serial.print(mpuOK);
        Serial.print(" Servos=");
        Serial.print(servosOK);
        Serial.print(" Uptime=");
        Serial.println(lastUpdateTime / 1000);
    }

    // 添加OLED显示函数
    void displayOLED()
    {
        display.clearDisplay();

        // 显示标题，使用大号字体
        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(25, 0);
        display.println("Car-O4");

        // 其余信息使用小号字体
        display.setTextSize(1);

        // 显示MPU6050状态和角度
        display.setCursor(0, 16);
        display.print("MPU:");
        display.print(mpuOK ? "OK" : "ERR");
        display.print(" X:");
        display.print(angleX, 1);
        display.print(" Y:");
        display.print(angleY, 1);

        // 显示舵机状态
        display.setCursor(0, 26);
        display.print("Servos:");
        display.println(servosOK ? "OK" : "ERR");

        // 绘制分隔线
        display.drawLine(0, 36, 128, 36, SSD1306_WHITE);

        // 显示运行时间
        display.setCursor(0, 40);
        int hours = lastUpdateTime / 3600000;
        int mins = (lastUpdateTime % 3600000) / 60000;
        int secs = (lastUpdateTime % 60000) / 1000;
        display.print("Time: ");
        if (hours < 10)
            display.print("0");
        display.print(hours);
        display.print(":");
        if (mins < 10)
            display.print("0");
        display.print(mins);
        display.print(":");
        if (secs < 10)
            display.print("0");
        display.print(secs);

        // 显示电池电压，带电池图标
        display.setCursor(0, 52);
        display.print("Batt: ");
        display.print(batteryVoltage, 1);
        display.print("V");

        // 绘制简单的电池图标
        int battLevel = map(batteryVoltage * 10, 30, 42, 0, 15); // 3.0V-4.2V
        battLevel = constrain(battLevel, 0, 15);
        display.drawRect(95, 52, 18, 10, SSD1306_WHITE);
        display.drawRect(113, 54, 2, 6, SSD1306_WHITE);
        display.fillRect(97, 54, battLevel, 6, SSD1306_WHITE);

        display.display();
    }
} status;

// 添加互补滤波
float complementaryFilter(float accAngle, float gyroRate, float dt, float alpha = 0.96)
{
    static float filterAngle = 0;
    filterAngle = alpha * (filterAngle + gyroRate * dt) + (1 - alpha) * accAngle;
    return filterAngle;
}

// 添加死区控制
float applyDeadzone(float value, float threshold = 2.0)
{
    if (abs(value) < threshold)
    {
        return 0;
    }
    return value > 0 ? value - threshold : value + threshold;
}

// 添加校准相关变量
float offsetX = 0;
float offsetY = 0;

void calibrateSensors()
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Calibrating...");
    display.println("Keep MPU6050 still");
    display.display();

    float sumX = 0, sumY = 0;
    const int samples = 100;

    for (int i = 0; i < samples; i++)
    {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        float accX = atan2(ay, az) * RAD_TO_DEG;
        float accY = atan2(-ax, az) * RAD_TO_DEG;
        sumX += accX;
        sumY += accY;
        delay(10);
    }

    offsetX = sumX / samples;
    offsetY = sumY / samples;

    display.println("Calibration done!");
    display.display();
    delay(1000);
}

// 添加平滑运动控制
float smoothMove(float target, float current, float smoothFactor = 0.1)
{
    return current + (target - current) * smoothFactor;
}

// 实现getModeMultiplier函数
float getModeMultiplier(ControlMode mode)
{
    return SENSITIVITY[mode];
}

// 添加全局变量
float filteredAngleX = 0;
float filteredAngleY = 0;
float servoAngle = 90;

// 添加按键定义
const int MODE_BUTTON_PIN = 0;      // 模式切换按键
const int CALIBRATE_BUTTON_PIN = 1; // 校准按键

void handleButtons()
{
    static unsigned long lastPress = 0;
    const int DEBOUNCE_TIME = 200; // 防抖时间

    // 模式切换
    if (digitalRead(MODE_BUTTON_PIN) == LOW)
    {
        if (millis() - lastPress > DEBOUNCE_TIME)
        {
            currentMode = (ControlMode)((currentMode + 1) % 3);
            // 显示模式切换提示
            display.clearDisplay();
            display.setTextSize(2);
            display.setCursor(0, 20);
            switch (currentMode)
            {
            case MODE_NORMAL:
                display.println("Normal");
                break;
            case MODE_SMOOTH:
                display.println("Smooth");
                break;
            case MODE_PRECISE:
                display.println("Precise");
                break;
            }
            display.display();
            delay(500);
            lastPress = millis();
        }
    }

    // 校准触发
    if (digitalRead(CALIBRATE_BUTTON_PIN) == LOW)
    {
        if (millis() - lastPress > DEBOUNCE_TIME)
        {
            calibrateSensors();
            lastPress = millis();
        }
    }
}

// 为不同模式设置不同PID参数
const struct PIDParams
{
    float Kp, Ki, Kd;
} MODE_PARAMS[] = {
    {2.0, 0.1, 0.5},  // 普通模式
    {1.0, 0.05, 0.8}, // 平滑模式
    {3.0, 0.15, 0.3}  // 精确模式
};

void updatePIDParams()
{
    pidX.Kp = MODE_PARAMS[currentMode].Kp;
    pidX.Ki = MODE_PARAMS[currentMode].Ki;
    pidX.Kd = MODE_PARAMS[currentMode].Kd;
}

// 添加动态平滑因子
float getSmoothing()
{
    switch (currentMode)
    {
    case MODE_SMOOTH:
        return 0.05; // 更平滑
    case MODE_PRECISE:
        return 0.2; // 更快响应
    default:
        return 0.1; // 默认
    }
}

struct Performance
{
    unsigned long lastUpdate;
    float fps;
    float maxAngleSpeed;
    float avgProcessTime;
    int sampleCount;

    void update(unsigned long processTime)
    {
        fps = 1000000.0f / processTime;
        sampleCount++;
        avgProcessTime = (avgProcessTime * (sampleCount - 1) + processTime) / sampleCount;
    }

    void reset()
    {
        sampleCount = 0;
        avgProcessTime = 0;
        maxAngleSpeed = 0;
    }
} perf;

// 添加显示相关常量
const int BAR_WIDTH = 64;
const int BAR_HEIGHT = 16;
const int CENTER_X = SCREEN_WIDTH / 2;
const int CENTER_Y = SCREEN_HEIGHT / 2;

void drawDisplay() {
    display.clearDisplay();
    
    // 1. 顶部状态栏
    display.drawFastHLine(0, 8, SCREEN_WIDTH, SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("FPS:");
    display.print(int(1000.0/dt/1000.0));
    
    // 2. 大号显示当前角度
    display.setTextSize(2);
    display.setCursor(35, 12);
    int displayAngle = abs(int(servoAngle - 90));  // 显示相对于中心的角度
    if(displayAngle < 10) display.print(" ");  // 对齐个位数
    display.print(displayAngle);
    display.setTextSize(1);
    display.print(char(247));  // 度数符号
    
    // 3. 动态水平仪
    drawLevelIndicator(filteredAngleX);
    
    // 4. 方向指示
    if(servoAngle > 92) {
        display.fillTriangle(120, 25, 120, 35, 127, 30, SSD1306_WHITE);  // 右箭头
    } else if(servoAngle < 88) {
        display.fillTriangle(0, 25, 0, 35, 7, 30, SSD1306_WHITE);  // 左箭头
    }
    
    // 5. 底部状态栏
    display.drawFastHLine(0, 56, SCREEN_WIDTH, SSD1306_WHITE);
    display.setCursor(0, 57);
    if(abs(servoAngle - 90) < 2) {
        display.print("BALANCED");
    } else {
        display.print(servoAngle > 90 ? "RIGHT" : "LEFT");
    }
    
    // 添加速度显示
    display.setCursor(0, 45);
    display.setTextSize(1);
    display.print("Speed:");
    display.print(map(servoAngle, 0, 180, 0, 100));
    display.print("%");
    
    display.display();
}

// 绘制水平仪
void drawLevelIndicator(float angle) {
    const int centerY = 40;
    const int width = 60;
    const int height = 12;
    
    // 外框
    display.drawRect(CENTER_X - width/2, centerY - height/2, width, height, SSD1306_WHITE);
    
    // 中心标记
    display.drawPixel(CENTER_X, centerY - height/2 - 1, SSD1306_WHITE);
    display.drawPixel(CENTER_X, centerY + height/2 + 1, SSD1306_WHITE);
    
    // 刻度marks
    for(int i = -2; i <= 2; i++) {
        int x = CENTER_X + (i * width/4);
        display.drawPixel(x, centerY - height/2 - 1, SSD1306_WHITE);
        display.drawPixel(x, centerY + height/2 + 1, SSD1306_WHITE);
    }
    
    // 动态球
    int ballPos = constrain(map(angle, -45, 45, -width/2 + 4, width/2 - 4), -width/2 + 4, width/2 - 4);
    display.fillCircle(CENTER_X + ballPos, centerY, 3, SSD1306_WHITE);
}

// 添加电调控制类
class LedcServo {
public:
    float freq = 50;
    int resolution = 8;
    float pwmBaseScale;
    float pwmMin;
    float pwmMax;
    int channel;
    int scale = 1;
    
    void setup(float freq, int resolution, int channel) {
        this->freq = freq;
        this->resolution = resolution;
        this->pwmBaseScale = this->freq * pow(2, this->resolution) / 1000;
        this->pwmMin = 1 * this->pwmBaseScale;
        this->pwmMax = 2 * this->pwmBaseScale;
        this->channel = channel;
        ledcSetup(this->channel, this->freq, this->resolution);
    }
    
    void setScale(float s) {
        if (s <= 0 || s > 1) return;
        this->scale = s;
        this->pwmMin = (1.5 - s * 0.5) * this->pwmBaseScale;
        this->pwmMax = (1.5 + s * 0.5) * this->pwmBaseScale;
    }
    
    void attachPin(int p) {
        ledcAttachPin(p, this->channel);
    }
    
    void write(float v, float min, float max) {
        float newV = constrain(v, min, max);
        ledcWrite(this->channel, map(newV, min, max, this->pwmMin, this->pwmMax));
    }
};

// 创建电调控制实例
LedcServo escControl;

// 在setup()中初始化电调
void setup() {
    Serial.begin(115200);
    Serial.println("System starting...");

    // 初始化I2C
    Wire.begin(19, 18); // SDA=19, SCL=18
    delay(200);         // 增加延时
    Serial.println("I2C initialized");

    // OLED初始化
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        Serial.println("OLED初始化失败!");
        return; // 不要死循环，继续执行
    }
    Serial.println("OLED初始化成功!");

    // 显示启动信息
    display.clearDisplay();
    display.setTextSize(1); // 使用小字体
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("System Starting...");
    display.display();

    // MPU6050初始化
    Serial.println("Initializing MPU6050...");
    mpu.initialize();
    if (!mpu.testConnection())
    {
        Serial.println("MPU6050连接失败!");
        display.println("MPU6050 Failed!");
        display.display();
    }
    else
    {
        Serial.println("MPU6050 OK!");
        display.println("MPU6050 OK");
        display.display();
    }

    // 初始化电调控制
    escControl.setup(50, 10, 1);  // 50Hz, 10位分辨率, 通道1
    escControl.attachPin(23);     // 连接到23号引脚
    escControl.setScale(1.0);     // 设置满量程
    
    // 电调初始化序列
    Serial.println("ESC Calibration...");
    display.println("ESC Init...");
    display.display();
    
    // 发送初始化序列
    escControl.write(0, -100, 100);  // 最小油门
    delay(3000);
    escControl.write(100, -100, 100); // 最大油门
    delay(2000);
    escControl.write(0, -100, 100);   // 回到最小油门
    delay(4000);
    
    Serial.println("ESC Ready!");
}

void checkErrors()
{
    static int errorCount = 0;
    const int ERROR_THRESHOLD = 10;

    // 检查传感器数据是否合理
    if (abs(filteredAngleX) > 60 || abs(filteredAngleY) > 60)
    {
        errorCount++;
        if (errorCount > ERROR_THRESHOLD)
        {
            // 重置系统
            setServoPWM(90);
            display.clearDisplay();
            display.println("System Reset");
            display.display();
            delay(1000);
            ESP.restart();
        }
    }
    else
    {
        errorCount = max(0, errorCount - 1);
    }
}

void loop()
{
    unsigned long currentTime = micros();
    dt = (currentTime - previousTime) / 1000000.0; // 转换为秒
    previousTime = currentTime;

    // 读取MPU6050数据
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // 计算倾角（使用加速度计）
    float accX = atan2(ay, az) * RAD_TO_DEG;
    float accY = atan2(-ax, az) * RAD_TO_DEG;

    // 转换陀螺仪数据为角速度（度/秒）
    float gyroX = gx / 131.0;
    float gyroY = gy / 131.0;

    // 使用卡尔曼滤波融合数据
    float angleX = kalmanX.getAngle(accX, gyroX, dt);
    float angleY = kalmanY.getAngle(accY, gyroY, dt);

    // 使用移动平均滤波进一步平滑数据
    filteredAngleX = getFilteredAngle(angleX, angleXSamples);
    filteredAngleY = getFilteredAngle(angleY, angleYSamples);

    // 计算目标速度
    float targetSpeed = calculateMotorSpeed(filteredAngleX);
    
    // 使用电调控制电机
    escControl.write(targetSpeed, -100, 100);
    
    // 调试输出
    Serial.print("Angle: ");
    Serial.print(filteredAngleX);
    Serial.print(" Speed: ");
    Serial.println(targetSpeed);
    
    // 更新显示
    display.clearDisplay();
    display.setTextSize(1);
    
    // 1. 顶部状态栏 - 显示FPS和运行时间
    display.setCursor(0, 0);
    display.print("FPS:");
    display.print(int(1000.0/dt/1000.0));
    display.print(" T:");
    display.print(millis()/1000);
    display.print("s");
    
    // 2. MPU6050数据
    display.setCursor(0, 10);
    display.print("X:");
    display.print(filteredAngleX, 1);
    display.print(" Y:");
    display.print(filteredAngleY, 1);
    
    // 3. 电机控制数据
    display.setCursor(0, 20);
    display.print("Speed:");
    display.print(int(targetSpeed));
    display.print("%");
    
    // 4. PWM信息
    display.setCursor(0, 30);
    int pwmValue = map(targetSpeed, 0, 100, escControl.pwmMin, escControl.pwmMax);
    display.print("PWM:");
    display.print(pwmValue);
    
    // 5. 状态指示器
    display.drawFastHLine(0, 40, 128, SSD1306_WHITE);
    display.setCursor(0, 45);
    if (targetSpeed == 0) {
        display.print("Status: IDLE");
    } else {
        display.print("Status: RUN ");
        // 添加动画指示器
        static int animFrame = 0;
        display.print("|/-\\"[animFrame++ % 4]);
    }
    
    // 6. 绘制速度条
    const int barWidth = 100;
    const int barHeight = 6;
    display.drawRect(14, 54, barWidth, barHeight, SSD1306_WHITE);
    int fillWidth = map(targetSpeed, 0, 100, 0, barWidth-2);
    if (fillWidth > 0) {
        display.fillRect(15, 55, fillWidth, barHeight-2, SSD1306_WHITE);
    }
    
    // 7. 错误指示（如果有）
    if (abs(filteredAngleX) > TILT_THRESHOLD) {
        display.setTextSize(1);
        display.setCursor(0, 56);
        display.print("! TILT !");
    }
    
    display.display();
    
    delay(20);  // 稍微增加延时
}

void drawEnhancedInterface()
{
    display.clearDisplay();

    // 顶部状态栏
    display.drawFastHLine(0, 8, 128, SSD1306_WHITE);

    // 模式图标
    display.drawRect(0, 0, 8, 8, SSD1306_WHITE);
    if (currentMode == MODE_PRECISE)
    {
        display.fillRect(0, 0, 8, 8, SSD1306_WHITE);
    }

    // 主要数据显示
    display.setTextSize(2);
    display.setCursor(32, 16);
    display.print(filteredAngleX, 1);

    // 绘制动态水平仪
    int centerY = 40;
    int centerX = 64;
    display.drawRect(centerX - 32, centerY - 8, 64, 16, SSD1306_WHITE);
    int barPos = constrain(map(filteredAngleX, -45, 45, -30, 30), -30, 30);
    display.fillRect(centerX + barPos - 2, centerY - 6, 4, 12, SSD1306_WHITE);

    // 性能指标
    display.setTextSize(1);
    display.setCursor(0, 56);
    display.print("FPS:");
    display.print(1000.0 / dt / 1000.0, 1);

    display.display();
}

void drawArrow(int x, int y, float angle, int length)
{
    float rad = angle * PI / 180.0;
    int endX = x + cos(rad) * length;
    int endY = y + sin(rad) * length;

    display.drawLine(x, y, endX, endY, SSD1306_WHITE);

    // 绘制箭头头部
    float headAngle1 = rad + PI * 0.875; // -45度
    float headAngle2 = rad - PI * 0.875; // +45度
    int headLength = 5;

    int head1X = endX + cos(headAngle1) * headLength;
    int head1Y = endY + sin(headAngle1) * headLength;
    int head2X = endX + cos(headAngle2) * headLength;
    int head2Y = endY + sin(headAngle2) * headLength;

    display.drawLine(endX, endY, head1X, head1Y, SSD1306_WHITE);
    display.drawLine(endX, endY, head2X, head2Y, SSD1306_WHITE);
}

// 添加一个函数用于显示错误信息
void showError(const char* error) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("ERROR");
    display.setTextSize(1);
    display.println();
    display.println(error);
    display.display();
    delay(2000);
}

// 添加一个函数用于显示当前配置
void showConfig() {
    display.clearDisplay();
    display.setTextSize(1);
    
    display.setCursor(0, 0);
    display.println("Configuration:");
    
    display.print("Min Throt: ");
    display.println(MIN_THROTTLE);
    
    display.print("Max Throt: ");
    display.println(MAX_THROTTLE);
    
    display.print("Deadzone: ");
    display.println("10.0");
    
    display.print("Tilt Limit: ");
    display.println(TILT_THRESHOLD);
    
    display.display();
    delay(2000);
}

// 修改calculateMotorSpeed函数
float calculateMotorSpeed(float tiltAngle) {
    // 安全检查
    if (abs(tiltAngle) > TILT_THRESHOLD) {
        return 0;
    }
    
    // 死区控制
    if (abs(tiltAngle) < DEADZONE) {
        return 0;
    }
    
    // 只在前倾时提供动力
    if (tiltAngle >= 0) { // 后倾
        return 0;
    }
    
    // 将倾角映射到速度（-100到100范围）
    float speed = map(constrain(tiltAngle, -30, -DEADZONE),
                     -30, -DEADZONE,
                     100, 0);  // 注意这里改为0-100范围
                     
    return speed;
}
