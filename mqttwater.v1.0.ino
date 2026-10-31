/*
 * 水箱监测与抽排水控制系统 —— MQTT 物联网版
 * 硬件：ESP8266（NodeMCU / D1 mini，引脚充足）
 * 通信：MQTT over TLS (8883) + 遗嘱消息（LWT）
 * 网页：独立 HTML，通过 MQTT over WebSocket (8084) 订阅
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <EEPROM.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// ===================== 配置 =====================
const char* WIFI_SSID = "CMCC-eujw";
const char* WIFI_PASS = "gmw18479639832";

const char* MQTT_HOST = "i11884f9.ala.cn-hangzhou.emqxsl.cn";
const int   MQTT_PORT = 8883;
const char* MQTT_USER = "esp-waterdetection";
const char* MQTT_PASS = "water666888";
const char* CLIENT_ID = "esp01_water_01_test";

// EMQX Cloud Serverless 证书指纹（*.emqxsl.cn）
const char* MQTT_FINGERPRINT = "7E:52:D3:84:48:3C:5A:9F:A4:39:9A:8B:27:01:B1:F8:C6:AD:D4:47";

// MQTT 主题
const char* TOPIC_STATUS   = "water/esp01_water01/status";    // 设备发布状态
const char* TOPIC_CMD      = "water/esp01_water01/cmd";       // 设备订阅命令
const char* TOPIC_ONLINE   = "water/esp01_water01/online";    // 在线状态（LWT）

// ===================== 引脚定义 =====================
// 水位传感器（沿用原逻辑：上拉输入，浸水拉低）
const int PIN_WATER_HIGH = 5;   // 上水位探头
const int PIN_WATER_LOW  = 4;   // 下水位探头

// MOS 管水泵控制
const int PIN_PUMP_IN  = 14;     // 抽水泵 MOS 栅极
const int PIN_PUMP_OUT = 12;     // 排水泵 MOS 栅极

// 手动模式物理开关（上拉输入，按下接地）
const int PIN_SW_PUMP_IN  = 13;  // 手动抽水开关
const int PIN_SW_PUMP_OUT = 15;  // 手动排水开关

// 报警外设（网页可控）
const int PIN_LED_ALARM    = 2; // LED 报警指示灯
const int PIN_BUZZER       = 16; // 蜂鸣器

// ===================== 全局状态 =====================
String waterLevel = "中等";
String prevWaterLevel = "中等";

bool autoMode = false;           // 默认手动模式
bool ledAlarmEnabled = true;     // 网页开关，默认开
bool buzzerEnabled = true;       // 网页开关，默认开

// 自动模式状态机
enum AutoState {
  AUTO_IDLE,          // 空闲
  AUTO_FILLING,       // 自动抽水中
  AUTO_DRAINING       // 自动排水中
};
AutoState autoState = AUTO_IDLE;

// 报警阈值（对应水位状态）
const String ALARM_HIGH = "充足";  // 高于上限
const String ALARM_LOW  = "不足";  // 低于下限

// 传感器去抖
unsigned long lastCheck = 0;
const unsigned long CHECK_INTERVAL = 1000;
String lastSafeLevel = "中等";
unsigned long stableTimer = 0;
const unsigned long STABLE_MS = 200;

// MQTT
WiFiClientSecure espClient;
PubSubClient mqtt(espClient);
unsigned long lastMqttReconnect = 0;
const unsigned long MQTT_RECONNECT_COOLDOWN = 5000;

// NTP（用于系统时间显示）
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "cn.pool.ntp.org", 28800);
bool timeSynced = false;
bool ntpInited = false;
unsigned long ntpTimer = 0;

// EEPROM 抽水记录
#define EEPROM_ADDR_PUMPTIME 0
#define PUMPTIME_MAX_LEN 64
String lastPumpTime = "";

// ===================== 函数声明 =====================
void loadPumpTime();
void savePumpTime(const String& dt);
String getWaterLevelText(int high, int low);
void checkWater();
void updatePumps();
void controlAlarm();
void publishStatus();
void publishOnline(bool online);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void reconnectMqtt();
void syncNtp();
String buildStatusJson();
String getFormattedDateTime();


// ===================== setup =====================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== 水箱 MQTT 版启动 ===");

  // 引脚初始化
  pinMode(PIN_WATER_HIGH, INPUT_PULLUP);
  pinMode(PIN_WATER_LOW,  INPUT_PULLUP);
  pinMode(PIN_SW_PUMP_IN,  INPUT_PULLUP);
  pinMode(PIN_SW_PUMP_OUT, INPUT_PULLUP);

  pinMode(PIN_PUMP_IN,  OUTPUT);
  pinMode(PIN_PUMP_OUT, OUTPUT);
  digitalWrite(PIN_PUMP_IN, LOW);
  digitalWrite(PIN_PUMP_OUT, LOW);

  pinMode(PIN_LED_ALARM, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_LED_ALARM, LOW);
  digitalWrite(PIN_BUZZER, LOW);

  // EEPROM
  EEPROM.begin(128);
  loadPumpTime();
  Serial.println("上次抽水：" + (lastPumpTime.length() ? lastPumpTime : "无记录"));

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("连接 WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nIP: " + WiFi.localIP().toString());

  // MQTT TLS
  espClient.setInsecure();  // ✅ 关键：直接跳过证书校验（向成功代码学习）
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setKeepAlive(60);

  // 初始水位
  int h = digitalRead(PIN_WATER_HIGH);
  int l = digitalRead(PIN_WATER_LOW);
  waterLevel = getWaterLevelText(h, l);
  lastSafeLevel = waterLevel;
}


// ===================== loop =====================
void loop() {
  unsigned long now = millis();

  // MQTT 维持
  if (!mqtt.connected()) {
    if (now - lastMqttReconnect > MQTT_RECONNECT_COOLDOWN) {
      lastMqttReconnect = now;
      reconnectMqtt();
    }
  } else {
    mqtt.loop();
  }

  // 传感器采样（1s）
  if (now - lastCheck > CHECK_INTERVAL) {
    lastCheck = now;
    checkWater();
  }

  // 执行器更新
  updatePumps();
  controlAlarm();

  // NTP（WiFi 连通时）
  if (WiFi.status() == WL_CONNECTED) {
    syncNtp();
  }

  // 定时发布状态（3s）
  static unsigned long lastPub = 0;
  if (now - lastPub > 3000) {
    lastPub = now;
    publishStatus();
  }
}


// ===================== MQTT =====================
void reconnectMqtt() {
  if (WiFi.status() != WL_CONNECTED) return;

  Serial.print("连接 MQTT...");
  // 遗嘱消息：异常断开时 Broker 自动发布 "offline"
  if (mqtt.connect(CLIENT_ID, MQTT_USER, MQTT_PASS,
                   TOPIC_ONLINE, 1, true, "offline")) {
    Serial.println("成功");
    publishOnline(true);
    mqtt.subscribe(TOPIC_CMD);
  } else {
    Serial.printf("失败 rc=%d\n", mqtt.state());
  }
}

void publishOnline(bool online) {
  mqtt.publish(TOPIC_ONLINE, online ? "online" : "offline", true);
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  String msg;
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];

  Serial.printf("收到 [%s]: %s\n", topic, msg.c_str());

  if (msg == "MODE_AUTO") {
    autoMode = true;
    autoState = AUTO_IDLE;
    // 进入自动模式先关闭所有泵，由状态机接管
    digitalWrite(PIN_PUMP_IN, LOW);
    digitalWrite(PIN_PUMP_OUT, LOW);
  } else if (msg == "MODE_MANUAL") {
    autoMode = false;
    autoState = AUTO_IDLE;
    digitalWrite(PIN_PUMP_IN, LOW);
    digitalWrite(PIN_PUMP_OUT, LOW);
  } else if (msg == "LED_ON")    ledAlarmEnabled = true;
  else if (msg == "LED_OFF")     ledAlarmEnabled = false;
  else if (msg == "BUZZER_ON")   buzzerEnabled = true;
  else if (msg == "BUZZER_OFF")  buzzerEnabled = false;
}


// ===================== 水位检测（沿用原逻辑） =====================
String getWaterLevelText(int high, int low) {
  // 上拉输入：浸水导通 → LOW
  if (high == HIGH && low == LOW) return "充足";   // 上探头未浸、下探头浸没
  if (high == LOW  && low == HIGH) return "不足";  // 上探头浸没、下探头未浸
  return "中等";
}

void checkWater() {
  int h = digitalRead(PIN_WATER_HIGH);
  int l = digitalRead(PIN_WATER_LOW);
  String newLevel = getWaterLevelText(h, l);

  if (newLevel == lastSafeLevel) {
    if (millis() - stableTimer > STABLE_MS) {
      if (newLevel != waterLevel) {
        prevWaterLevel = waterLevel;
        waterLevel = newLevel;

        // 记录抽水完成时间
        if (waterLevel == "充足" && prevWaterLevel != "充足") {
          if (timeSynced) savePumpTime(getFormattedDateTime());
        }
      }
    }
  } else {
    lastSafeLevel = newLevel;
    stableTimer = millis();
  }
}


// ===================== 水泵控制 =====================
void updatePumps() {
  if (!autoMode) {
    // ---------- 手动模式 ----------
    autoState = AUTO_IDLE;

    // 抽水开关：按下（LOW）时开抽水泵，松开即停
    bool manualIn  = (digitalRead(PIN_SW_PUMP_IN) == LOW);
    bool manualOut = (digitalRead(PIN_SW_PUMP_OUT) == LOW);

    // 互锁：不允许同时抽水和排水
    if (manualIn && !manualOut) {
      digitalWrite(PIN_PUMP_IN, HIGH);
      digitalWrite(PIN_PUMP_OUT, LOW);
    } else if (manualOut && !manualIn) {
      digitalWrite(PIN_PUMP_IN, LOW);
      digitalWrite(PIN_PUMP_OUT, HIGH);
    } else {
      digitalWrite(PIN_PUMP_IN, LOW);
      digitalWrite(PIN_PUMP_OUT, LOW);
    }
    return;
  }

  // ---------- 自动模式 ----------
  switch (autoState) {
    case AUTO_IDLE:
      // 水位不足 → 开始抽水
      if (waterLevel == "不足") {
        autoState = AUTO_FILLING;
      }
      // 水位充足 → 开始排水（放尽）
      else if (waterLevel == "充足") {
        autoState = AUTO_DRAINING;
      }
      break;

    case AUTO_FILLING:
      digitalWrite(PIN_PUMP_IN, HIGH);
      digitalWrite(PIN_PUMP_OUT, LOW);
      // 到达上限（充足）→ 关闭抽水，转入排水
      if (waterLevel == "充足") {
        digitalWrite(PIN_PUMP_IN, LOW);
        autoState = AUTO_DRAINING;
      }
      break;

    case AUTO_DRAINING:
      digitalWrite(PIN_PUMP_IN, LOW);
      digitalWrite(PIN_PUMP_OUT, HIGH);
      // 水位放尽（不足）→ 关闭排水，回到空闲
      if (waterLevel == "不足") {
        digitalWrite(PIN_PUMP_OUT, LOW);
        autoState = AUTO_IDLE;
      }
      break;
  }
}


// ===================== 报警 =====================
void controlAlarm() {
  bool alarmActive = (waterLevel == ALARM_HIGH || waterLevel == ALARM_LOW);

  // LED 报警指示灯
  if (ledAlarmEnabled && alarmActive) {
    digitalWrite(PIN_LED_ALARM, HIGH);
  } else {
    digitalWrite(PIN_LED_ALARM, LOW);
  }

  // 蜂鸣器
  if (buzzerEnabled && alarmActive) {
    digitalWrite(PIN_BUZZER, HIGH);
  } else {
    digitalWrite(PIN_BUZZER, LOW);
  }
}


// ===================== 状态发布 =====================
String buildStatusJson() {
  String level = waterLevel;
  String mode  = autoMode ? "auto" : "manual";

  String autoStateStr = "idle";
  if (autoMode) {
    if (autoState == AUTO_FILLING)  autoStateStr = "filling";
    if (autoState == AUTO_DRAINING) autoStateStr = "draining";
  }

  String alarm = (waterLevel == ALARM_HIGH || waterLevel == ALARM_LOW) ? "true" : "false";

  String dt = timeSynced ? getFormattedDateTime() : "等待NTP同步";

  String pumpIn  = digitalRead(PIN_PUMP_IN)  ? "on" : "off";
  String pumpOut = digitalRead(PIN_PUMP_OUT) ? "on" : "off";

  String json = "{";
  json += "\"level\":\"" + level + "\",";
  json += "\"mode\":\"" + mode + "\",";
  json += "\"autoState\":\"" + autoStateStr + "\",";
  json += "\"alarm\":" + alarm + ",";
  json += "\"pumpIn\":\"" + pumpIn + "\",";
  json += "\"pumpOut\":\"" + pumpOut + "\",";
  json += "\"datetime\":\"" + dt + "\",";
  json += "\"lastPump\":\"" + (lastPumpTime.length() ? lastPumpTime : "无记录") + "\"";
  json += "}";
  return json;
}

void publishStatus() {
  if (!mqtt.connected()) return;
  String json = buildStatusJson();
  mqtt.publish(TOPIC_STATUS, json.c_str(), true);  // retain=true，网页一订阅就能拿到最新状态
}


// ===================== NTP =====================
void syncNtp() {
  if (!ntpInited) {
    timeClient.begin();
    ntpInited = true;
    ntpTimer = millis();
  }
  if (millis() - ntpTimer > 10000) {
    ntpTimer = millis();
    if (timeClient.update()) {
      if (!timeSynced) Serial.println("NTP 同步成功");
      timeSynced = true;
    }
  }
}

String getFormattedDateTime() {
  if (!timeSynced) return "";
  time_t t = timeClient.getEpochTime();
  struct tm* ptm = localtime(&t);
  char buf[24];
  sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d",
          ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday,
          ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
  return String(buf);
}


// ===================== EEPROM =====================
void loadPumpTime() {
  char buf[PUMPTIME_MAX_LEN] = {0};
  for (int i = 0; i < PUMPTIME_MAX_LEN - 1; i++) {
    uint8_t c = EEPROM.read(EEPROM_ADDR_PUMPTIME + i);
    if (c == 0xFF || c == 0) break;
    buf[i] = (char)c;
  }
  lastPumpTime = String(buf);
  if (lastPumpTime.length() < 8) lastPumpTime = "";
}

void savePumpTime(const String& dt) {
  for (int i = 0; i < PUMPTIME_MAX_LEN; i++) EEPROM.write(EEPROM_ADDR_PUMPTIME + i, 0);
  int len = dt.length();
  if (len >= PUMPTIME_MAX_LEN) len = PUMPTIME_MAX_LEN - 1;
  for (int i = 0; i < len; i++) EEPROM.write(EEPROM_ADDR_PUMPTIME + i, dt[i]);
  EEPROM.commit();
  lastPumpTime = dt;
}