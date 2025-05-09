/**************************************************  
 * ESP32-S3 智能输液监测系统  
 * 功能：同时监测输液滴速、累计滴数和重量变化  
 * 特点：OLED显示信息,通过WebSocket发送数据  
 * 网络功能：WiFi + WebSocket服务器  
 * I2C连接：SCK->GPIO42, SDA->GPIO41  
 * 注意：请将文件保存为UTF-8编码格式  
 **************************************************/  

#include <Wire.h>  
#include <Adafruit_GFX.h>  
#include <Adafruit_SSD1306.h>  
#include "HX711.h"  
#include <U8g2lib.h>  // 添加U8g2库以支持中文显示  
#include <WiFi.h>     // 添加WiFi支持  
#include <WebSocketsServer.h> // 添加WebSocket支持  

// WiFi设置  
const char* ssid = "1503";     // 替换为您的WiFi名称  
const char* password = "18310007230"; // 替换为您的WiFi密码  
bool wifiConnected = false;  

// WebSocket服务器  
WebSocketsServer webSocket = WebSocketsServer(81); // 使用81端口  
bool clientConnected = false;  

// 传感器配置  
const int WATER_SENSOR_PIN = 11;    // 滴落传感器连接的GPIO引脚  
const int LED_PIN = 2;              // 内置LED引脚  

// I2C引脚定义  
const int SDA_PIN = 41;             // SDA连接到GPIO 41  
const int SCL_PIN = 42;             // SCL(SCK)连接到GPIO 42  

// HX711引脚定义  
const int DT_PIN = 17;  
const int SCK_PIN = 18;  
float calibration_factor = 1687.0;
HX711 scale;  
float Weight = 0;  

// OLED显示屏配置  
#define SCREEN_WIDTH 128            // OLED 显示宽度，像素  
#define SCREEN_HEIGHT 32            // OLED 显示高度，像素  
#define OLED_RESET    -1           // Reset pin  
#define SCREEN_ADDRESS 0x3C        // I2C地址  
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  // 使用U8g2替代Adafruit库  

// 消抖参数  
const unsigned long DEBOUNCE_TIME = 100;   // 消抖时间(毫秒)  

// 数据记录参数  
unsigned long previousMillis = 0;  
unsigned long previousUploadMillis = 0;  
const long displayInterval = 1000;        // 显示更新间隔(毫秒)  
const long uploadInterval = 1000;        // 数据上传间隔(毫秒) - 改为10秒
unsigned long recordCount = 0;  

// 全局变量  
volatile unsigned long dropCount = 0;           // 水滴计数  
volatile unsigned long lastDropTime = 0;        // 上次记录水滴的时间  
volatile unsigned long dropInterval = 0;        // 两滴之间的时间间隔
volatile bool newDrop = false;                 // 新水滴标志
unsigned long startTime = 0;                    // 程序开始时间  
unsigned long lastUploadDrops = 0;              // 上次上传时的滴数
unsigned long lastUploadTime = 0;               // 上次上传时间

// 添加滤波相关变量
const int WEIGHT_WINDOW_SIZE = 10;  // 权重移动平均窗口大小
float weightBuffer[WEIGHT_WINDOW_SIZE] = {0};  // 权重缓冲区
int weightIndex = 0;  // 当前权重存储位置
float filteredWeight = 0;  // 过滤后的重量
const float WEIGHT_CHANGE_THRESHOLD = 30.0;  // 重量突变阈值（克），超过这个值时重置滤波器

// 添加滴落时间记录
const int MAX_DROPS_PER_INTERVAL = 50;  // 每个间隔最多记录50滴
unsigned long dropTimestamps[MAX_DROPS_PER_INTERVAL];  // 存储滴落时间戳
int dropTimestampIndex = 0;  // 当前时间戳存储位置
bool hasNewDrops = false;    // 是否有新的滴落记录

// 滴速计算相关变量
const int DROP_RATE_WINDOW_SIZE = 10;  // 滴速滑动平均窗口大小 - 改为10点
float dropRateBuffer[DROP_RATE_WINDOW_SIZE] = {0};  // 滴速缓冲区
int dropRateIndex = 0;  // 当前滴速存储位置
float currentDropRate = 0;  // 当前滴速

// 中断服务函数  
void IRAM_ATTR waterDropIsr() {  
  unsigned long currentTime = millis();  
  
  // 消抖处理  
  if (currentTime - lastDropTime > DEBOUNCE_TIME) {  
    dropInterval = currentTime - lastDropTime;  // 记录时间间隔
    dropCount++;              
    lastDropTime = currentTime;  
    newDrop = true;  // 设置新水滴标志
  }  
}  

// 计算滴速
void updateDropRate() {
  if (newDrop) {
    // 计算滴速（滴/分钟）并更新滑动平均
    float rate = 60000.0 / dropInterval;
    dropRateBuffer[dropRateIndex] = rate;
    dropRateIndex = (dropRateIndex + 1) % DROP_RATE_WINDOW_SIZE;
    
    // 计算平均滴速
    float sum = 0;
    for (int i = 0; i < DROP_RATE_WINDOW_SIZE; i++) {
      sum += dropRateBuffer[i];
    }
    currentDropRate = sum / DROP_RATE_WINDOW_SIZE;
    newDrop = false;  // 清除新水滴标志
  } else if (millis() - lastDropTime > 3000) {  // 3秒无水滴，清零滴速
    for (int i = 0; i < DROP_RATE_WINDOW_SIZE; i++) {
      dropRateBuffer[i] = 0;
    }
    currentDropRate = 0;
  }
}

// WebSocket事件处理  
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {  
  switch(type) {  
    case WStype_DISCONNECTED:  
      Serial.printf("[%u] 断开连接!\n", num);  
      if (num == 0) clientConnected = false;  
      break;  
    case WStype_CONNECTED:  
      {  
        IPAddress ip = webSocket.remoteIP(num);  
        Serial.printf("[%u] 连接来自 %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);  
        clientConnected = true;  
        
        // 发送CSV表头  
        String header = "timestamp,drops,weight,dropRate";  
        webSocket.sendTXT(num, header);  
      }  
      break;  
    case WStype_TEXT:  
      Serial.printf("[%u] 收到: %s\n", num, payload);  
      break;  
  }  
}  

// 权重滤波函数
float filterWeight(float newWeight) {
  static float lastStableWeight = 0;
  
  // 检查是否是大幅度重量变化
  if (abs(newWeight - lastStableWeight) > WEIGHT_CHANGE_THRESHOLD) {
    // 重置滤波器
    for(int i = 0; i < WEIGHT_WINDOW_SIZE; i++) {
      weightBuffer[i] = newWeight;
    }
    lastStableWeight = newWeight;
    return newWeight;
  }
  
  // 正常滤波处理
  weightBuffer[weightIndex] = newWeight;
  weightIndex = (weightIndex + 1) % WEIGHT_WINDOW_SIZE;
  
  // 计算移动平均
  float sum = 0;
  for(int i = 0; i < WEIGHT_WINDOW_SIZE; i++) {
    sum += weightBuffer[i];
  }
  float filteredValue = sum / WEIGHT_WINDOW_SIZE;
  
  // 更新最后的稳定值
  if (abs(filteredValue - lastStableWeight) < 1.0) {  // 如果变化小于1克，认为是稳定的
    lastStableWeight = filteredValue;
  }
  
  return filteredValue;
}

// 更新OLED显示  
void updateDisplay() {  
  u8g2.clearBuffer();  
  
  // 绘制边框  
  u8g2.drawFrame(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);  
  
  // 设置中文字体  
  u8g2.setFont(u8g2_font_wqy12_t_gb2312);  
  
  // 显示水滴信息
  u8g2.setCursor(4, 12);  
  u8g2.print("滴数:");  
  u8g2.setCursor(40, 12);  
  u8g2.print(dropCount);  
  u8g2.print(" 滴速:");  // 添加滴速显示
  u8g2.print(currentDropRate, 1);
  u8g2.print("/分");
  
  // 显示重量信息  
  u8g2.setCursor(4, 24);  
  u8g2.print("重量:");  
  u8g2.setCursor(40, 24);  
  u8g2.print(Weight, 1);  
  u8g2.print("克");  
  
  u8g2.sendBuffer();  
}  

// 连接WiFi  
void connectToWiFi() {  
  Serial.println("正在连接到WiFi...");  
  
  // 显示连接信息  
  u8g2.clearBuffer();  
  u8g2.setFont(u8g2_font_wqy12_t_gb2312);  
  u8g2.setCursor(0, 12);  
  u8g2.print("正在连接WiFi...");  
  u8g2.setCursor(0, 24);  
  u8g2.print(ssid);  
  u8g2.sendBuffer();  
  
  WiFi.begin(ssid, password);  
  
  int attempts = 0;  
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {  
    delay(500);  
    Serial.print(".");  
    attempts++;  
  }  
  
  if(WiFi.status() == WL_CONNECTED) {  
    Serial.println("");  
    Serial.print("已连接WiFi, IP地址: ");  
    Serial.println(WiFi.localIP());  
    
    // 显示IP地址  
    u8g2.clearBuffer();  
    u8g2.setCursor(0, 12);  
    u8g2.print("WiFi已连接");  
    u8g2.setCursor(0, 24);  
    u8g2.print("IP:");  
    u8g2.print(WiFi.localIP().toString());  
    u8g2.sendBuffer();  
    
    wifiConnected = true;  
    delay(2000);  
  } else {  
    Serial.println("");  
    Serial.println("WiFi连接失败, 将继续使用串口传输数据");  
    
    // 显示连接失败  
    u8g2.clearBuffer();  
    u8g2.setCursor(0, 12);  
    u8g2.print("WiFi连接失败");  
    u8g2.setCursor(0, 24);  
    u8g2.print("使用串口传输");  
    u8g2.sendBuffer();  
    
    wifiConnected = false;  
    delay(2000);  
  }  
}  

void setup() {  
  // 记录开始时间  
  startTime = millis();  
  
  // 初始化串口通信  
  Serial.begin(115200);  
  while (!Serial && millis() < 3000);  // 等待串口准备好，但最多等3秒  
  
  // 发送CSV表头  
  Serial.println("timestamp,drops,weight,dropRate");  
  
  // 初始化I2C  
  Wire.begin(SDA_PIN, SCL_PIN);  
  
  // 初始化OLED  
  u8g2.begin();  
  u8g2.enableUTF8Print();  // 启用UTF8支持  
  
  // 初始化HX711  
  scale.begin(DT_PIN, SCK_PIN);  
  scale.set_scale(calibration_factor);  
  scale.set_gain(128);  // 设置增益为128
  
  // 多次读取以稳定初始值
  for(int i = 0; i < WEIGHT_WINDOW_SIZE; i++) {
    if(scale.is_ready()) {
      weightBuffer[i] = scale.get_units(10);  // 每个初始读数取10次平均
      delay(100);
    }
  }
  
  // 显示初始化信息  
  u8g2.clearBuffer();  
  u8g2.setFont(u8g2_font_wqy12_t_gb2312);  
  u8g2.setCursor(0, 12);  
  u8g2.print("智能输液监测系统");  
  u8g2.setCursor(0, 24);  
  u8g2.print("系统初始化中...");  
  u8g2.sendBuffer();  
  delay(2000);  
  
  // 初始化引脚  
  pinMode(WATER_SENSOR_PIN, INPUT_PULLDOWN);  
  pinMode(LED_PIN, OUTPUT);                   
  digitalWrite(LED_PIN, LOW);                 
  
  // 设置中断  
  attachInterrupt(digitalPinToInterrupt(WATER_SENSOR_PIN), waterDropIsr, RISING);  
  
  // 进行称重归零  
  u8g2.clearBuffer();  
  u8g2.setCursor(0, 12);  
  u8g2.print("正在校准重量...");  
  u8g2.sendBuffer();  
  scale.tare();  
  delay(1000);  
  
  // 连接WiFi  
  connectToWiFi();  
  
  // 如果WiFi连接成功，启动WebSocket服务器  
  if (wifiConnected) {  
    webSocket.begin();  
    webSocket.onEvent(webSocketEvent);  
    Serial.println("WebSocket服务器已启动,端口:81");  
  }  
  
  // 启动指示 - LED闪烁三次  
  for (int i = 0; i < 3; i++) {  
    digitalWrite(LED_PIN, HIGH);  
    delay(100);  
    digitalWrite(LED_PIN, LOW);  
    delay(100);  
  }  
  
  Serial.println("系统就绪,开始记录数据...");  
  digitalWrite(LED_PIN, HIGH);  
}  

void loop() {  
  // 处理WebSocket  
  if (wifiConnected) {  
    webSocket.loop();  
  }  
  
  unsigned long currentMillis = millis();  
  
  // 每秒更新一次所有数据
  if (currentMillis - previousMillis >= displayInterval) {  
    previousMillis = currentMillis;
    
    if (scale.is_ready()) {
      // 获取重量数据并应用滤波
      float rawWeight = scale.get_units(10);
      Weight = filterWeight(rawWeight);
    }
    
    // 更新滴速
    updateDropRate();
    
    // 更新显示
    updateDisplay();
  }

  // 每10秒上传一次数据
  if (currentMillis - previousUploadMillis >= uploadInterval) {
    previousUploadMillis = currentMillis;
    
    // 准备数据字符串
    String dataString = String(currentMillis) + "," +   
                       String(dropCount) + "," +   
                       String(Weight) + "," +
                       String(currentDropRate, 1);
    
    // 发送数据到串口  
    Serial.println(dataString);  
    
    // 如果有WebSocket客户端连接，则发送数据  
    if (wifiConnected && clientConnected) {  
      webSocket.broadcastTXT(dataString);  
    }
    
    // 闪烁LED表示数据已记录
    digitalWrite(LED_PIN, LOW);
    delay(20);
    digitalWrite(LED_PIN, HIGH);
  }
}  