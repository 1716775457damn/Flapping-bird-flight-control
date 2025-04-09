#include "esp_camera.h"
#include <WiFi.h>
#include "AsyncUDP.h"
#include <vector>
#include "esp_task_wdt.h" // 添加看门狗定时器头文件

const char *ssid = "ovo";
const char *password = "twx20051";

#define maxcache 800

// 设置每次发送最大的数据量，如果选择一次发送会出现丢失数据，经测试，我这边每
// 次最大发送1436，选择一个稍微小点的数
AsyncUDP udp; // 异步udp既可以发送也可以接收

// 服务器IP和端口配置 - 使用变量方便修改
IPAddress serverIP(192, 168, 34, 254);  // 服务器IP地址
const uint16_t serverPort = 8080;       // 服务器端口

// 为ESP32-S3-CAM更新的摄像头引脚定义
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     40
#define SIOD_GPIO_NUM     17
#define SIOC_GPIO_NUM     18

#define Y9_GPIO_NUM       39
#define Y8_GPIO_NUM       41
#define Y7_GPIO_NUM       42
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       11
#define Y4_GPIO_NUM       10
#define Y3_GPIO_NUM       9
#define Y2_GPIO_NUM       8
#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM     13

// 添加状态LED引脚定义
#define STATUS_LED_PIN 4  // 系统状态指示灯
#define NETWORK_LED_PIN 2 // 网络状态指示灯

// 添加包序号相关变量
uint32_t packet_sequence = 0;
struct PacketHeader
{
    uint32_t sequence;
    uint32_t total_packets;
    uint32_t packet_size;
};

// 添加LED控制相关定义
#define CONTROL_LED_PIN 33 // 控制LED的引脚
bool led_state = false;    // LED当前状态

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_VGA,
    .jpeg_quality = 12,
    .fb_count = 1,
};

esp_err_t camera_init()
{
    // initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        Serial.println("Camera Init Failed!");
        return err;
    }
    sensor_t *s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV2640_PID)
    {
        //        s->set_vflip(s, 1);//flip it back
        //        s->set_brightness(s, 1);//up the blightness just a bit
        //        s->set_contrast(s, 1);
    }
    Serial.println("Camera Init OK!");
    return ESP_OK;
}

String aaaa = "";

void wifi_init(void)
{
    delay(10);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false); // 鍏抽棴STA妯″紡涓媤ifi浼戠湢锛屾彁楂樺搷搴旈�熷害
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.println("WiFi Connected OK!");
    Serial.print("IP Address:");
    Serial.println(WiFi.localIP());
    pinMode(33, OUTPUT);
}

void setup()
{
    // 初始化LED引脚
    pinMode(CONTROL_LED_PIN, OUTPUT);
    digitalWrite(CONTROL_LED_PIN, LOW); // 初始状态为关闭

    Serial.begin(115200);
    camera_init();
    wifi_init();
    Serial.println("Sys Is Running!");

    if (udp.listen(9090))
    {
        Serial.println("UDP Listening on port 9090");
    }

    if (udp.connect(serverIP, serverPort))
    {
        Serial.println("UDP Server Connected!");
        udp.onPacket([](AsyncUDPPacket packet)
                     {
            String command((char*)packet.data(), packet.length());
            if (command == "51") {
                digitalWrite(CONTROL_LED_PIN, HIGH);
                led_state = true;
                Serial.println("LED ON");
            }
            else if (command == "52") {
                digitalWrite(CONTROL_LED_PIN, LOW);
                led_state = false;
                Serial.println("LED OFF");
            } });
    }

    // 初始化状态LED
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(NETWORK_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH);

    // 初始化看门狗定时器
    esp_task_wdt_init(10, true);
    esp_task_wdt_add(NULL);
}

void loop()
{
    // 喂狗
    esp_task_wdt_reset();

    // 闪烁系统状态指示灯
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 1000)
    {
        digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
        lastBlink = millis();
    }

    // 网络状态指示
    digitalWrite(NETWORK_LED_PIN, WiFi.status() == WL_CONNECTED);

    // 尝试重新连接UDP服务器，如果连接失败，稍后再尝试
    if (!udp.connect(serverIP, serverPort))
    {
        Serial.println("Connected UDP Server Fail, After 10 Seconds Try Again!");
        delay(10000);
        return;
    }

    // 获取一帧图像
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
        Serial.println("Camera capture failed");
        delay(5000);
        return;
    }

    // 计算需要发送的数据包数量
    size_t total_len = fb->len;
    size_t max_packet_size = std::min((size_t)maxcache, (size_t)1400); // 减小包大小以适应头部
    size_t packets_count = (total_len + max_packet_size - 1) / max_packet_size;

    // 发送图像数据
    for (size_t i = 0; i < packets_count; ++i)
    {
        PacketHeader header;
        header.sequence = packet_sequence++;
        header.total_packets = packets_count;
        header.packet_size = std::min(total_len, max_packet_size);

        // 发送包头
        udp.write((uint8_t *)&header, sizeof(header));

        // 发送图像数据
        size_t send_len = header.packet_size;
        udp.write(fb->buf + i * max_packet_size, send_len);
        total_len -= send_len;

        // 短暂延时防止包堵塞
        delay(1);
    }

    // 通知对方帧发送完毕
    udp.println("Frame Over");

    // 返回帧缓冲区
    esp_camera_fb_return(fb);

    // 适当延时
    delay(20);
}