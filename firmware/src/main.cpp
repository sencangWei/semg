/*
 * BNO08x IMU → WiFi/TCP 推送给 customer_collector 的 imu_client.py
 *
 * 硬件: FireBeetle 2 ESP32-S3 (4MB Flash) + GY-BNO08x
 *   SDA->GPIO1  SCL->GPIO2  RST->GPIO4
 *
 * 角色: 默认 STA(连家里路由器) + TCP 服务器; 编译加 -DUSE_WIFI_AP 切回 AP(离线 fallback)
 *   左手: STA imu_left  /  AP imu_left_ap   port=8766
 *   右手: STA imu_right /  AP imu_right_ap  port=8767
 *   WiFi 凭据: 本文件顶部 WIFI_SSID / WIFI_PASS (左右手共用)
 *
 * 推送格式 (JSON, 每帧一行, 200Hz), 与 imu_client.py 的 _normalize_json 对齐:
 *   {"t":1234567,"q":[w,x,y,z],"q6":[w,x,y,z],"qa":3,"q6a":3,
 *    "a":[ax,ay,az],"g":[gx,gy,gz],"la":[lx,ly,lz]}
 *   t = ESP32 micros() 单调微秒时钟 (姿态轨迹采样时间; 跨设备对齐仍用 PC wall_ts)
 *   q  = 9轴 Rotation Vector (磁北+重力参考, 向后兼容)
 *   q6 = 6轴 Game Rotation Vector (无磁力计, 短时精细姿态默认使用)
 *   qa/q6a = 精度状态: 0不可靠, 1低, 2中, 3高
 *   a  = 加速度 m/s^2 (含重力)
 *   la = 线性加速度 m/s^2 (硬件去重力+去偏置, 用于位置轨迹积分)
 *
 * 对齐原理: PC 端 imu_client 每收到一帧就用 time.time() 打 wall 时间戳,
 *           所有设备都打 PC 墙钟, 后处理 align.py 按真实时间对齐.
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_BNO08x.h>

// ---------- 左右手配置 (编译期切换) ----------
#if defined(POSITION_RIGHT)
  #define AP_SSID    "EMG_IMU_RIGHT"
  #define TCP_PORT   8767
  #define ROLE       "RIGHT"
#else
  #define AP_SSID    "EMG_IMU_LEFT"
  #define TCP_PORT   8766
  #define ROLE       "LEFT"
#endif
#define AP_CHANNEL   6

// ---------- WiFi STA 凭据 / 模式 ----------
// 默认走 STA(连家里路由器); 编译加 -DUSE_WIFI_AP=1 切回纯 AP(离线/无路由器 fallback, 单手已验证)
// (宏名故意用 USE_WIFI_AP 而非 WIFI_MODE_AP: 后者是 ESP-IDF wifi_mode_t 枚举的既有成员, 命令行 -D 会污染 SDK 头文件)
// ★★★ STA 模式 WiFi 凭据: 改成你家路由器的 SSID 和密码 (左右手共用这一份) ★★★
// (不放 platformio.ini: Windows 下命令行 -D 传字符串宏引号会被吃掉, 直接写源码最稳)
#define WIFI_SSID   "@Ruijie-s6145_iot"
#define WIFI_PASS   "1234567890"
#define WIFI_STA_TIMEOUT_MS   15000UL   // setup() 等 DHCP 拿 IP 的超时
#define WIFI_RECONNECT_MS      5000UL   // loop() 重连尝试最小间隔(避免狂连刷屏)

// ---------- STA 静态 IP (避免 DHCP 每次分不同 IP, config.json 填一次即可) ----------
// 你家路由器: 网关 192.168.113.1, 子网掩码 255.255.255.0. 换网段时改下面 STA_IP_PREFIX / STA_GW_TAIL.
#define STA_IP_PREFIX   192, 168, 113   // 网段前 3 段
#define STA_GW_TAIL     1               // 网关(路由器)尾号
#if defined(POSITION_RIGHT)
  #define STA_IP_TAIL   50              // 右手固定 .50
#else
  #define STA_IP_TAIL   49              // 左手固定 .49
#endif

// ---------- BNO08x 接线 ----------
#define BNO_SDA   1
#define BNO_SCL   2
#define BNO_RST   4

// ---------- 电池充电保护 ----------
// 需要外接分压电阻才能读取电池电压, 例如:
//   电池正极 --100kΩ-- ADC_PIN --100kΩ-- GND  (分压比 2.0)
// 未焊接分压电阻时保持 BATTERY_ADC_PIN = -1 禁用本功能.
#define BATTERY_ADC_PIN        -1       // A1/GPIO5 预留; -1 禁用
#define BATTERY_DIVIDER_RATIO  2.0f     // 分压比 = (R_upper + R_lower) / R_lower
#define BATTERY_MAX_V          4.25f    // 过充保护阈值 (锂电池满电 4.2V)
#define BATTERY_MIN_V          3.30f    // 欠压保护阈值
#define BATTERY_CHECK_MS       5000UL   // 检测间隔
#define BATTERY_FAULT_HYST     2        // 连续 N 次越限才判定故障

// ---------- 电源指示 ----------
// DFR1145 板载单色 LED: GPIO21, 高电平点亮. 固件运行期间常亮.
#define STATUS_LED_PIN 21

// ---------- 推送 ----------
#define SEND_HZ   200            // 200Hz (BNO08x 报告最高支持 400Hz)
#define REPORT_INTERVAL_US (1000000UL / SEND_HZ)

Adafruit_BNO08x bno(BNO_RST);
sh2_SensorValue_t val;
bool bnoOK = false;

// 最新传感器读数 (getSensorEvent 异步到达, 这里缓存最新值)
float qw = 1, qx = 0, qy = 0, qz = 0;
float q6w = 1, q6x = 0, q6y = 0, q6z = 0;
uint8_t qAccuracy = 0, q6Accuracy = 0;
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float lax = 0, lay = 0, laz = 0;   // 线性加速度 (硬件去重力+去偏置), 用于位置轨迹

WiFiServer server(TCP_PORT);
WiFiClient client;
uint32_t frames_sent = 0;

// ---------- 电池保护状态 ----------
bool battery_fault = false;            // true = 过压/欠压, 停止推送保护
float battery_voltage = 0.0f;
int battery_fault_count = 0;
int battery_ok_count = 0;

// 读取电池电压 (mV 参考默认 3.3V / 12bit = 4095)
float readBatteryVoltage() {
#if BATTERY_ADC_PIN >= 0
    int raw = analogRead(BATTERY_ADC_PIN);
    float v_adc = raw * (3.3f / 4095.0f);
    return v_adc * BATTERY_DIVIDER_RATIO;
#else
    return 0.0f;
#endif
}

// 检查电池电压, 越限时设置 battery_fault 并告警
void checkBatteryProtection() {
#if BATTERY_ADC_PIN >= 0
    static uint32_t last_check = 0;
    uint32_t now = millis();
    if (now - last_check < BATTERY_CHECK_MS) return;
    last_check = now;

    battery_voltage = readBatteryVoltage();

    bool overvoltage = battery_voltage > BATTERY_MAX_V;
    bool undervoltage = (battery_voltage > 0.1f) && (battery_voltage < BATTERY_MIN_V);

    if (overvoltage || undervoltage) {
        battery_ok_count = 0;
        if (++battery_fault_count >= BATTERY_FAULT_HYST) {
            battery_fault = true;
            Serial.printf("[BATT] 故障! V=%.2fV  %s  停止推送保护IMU\n",
                          battery_voltage,
                          overvoltage ? "过压" : "欠压");
        }
    } else {
        battery_fault_count = 0;
        if (battery_fault && ++battery_ok_count >= BATTERY_FAULT_HYST) {
            battery_fault = false;
            Serial.printf("[BATT] 恢复 V=%.2fV  恢复推送\n", battery_voltage);
        }
    }
#endif
}

void setReports() {
    bno.enableReport(SH2_ROTATION_VECTOR, REPORT_INTERVAL_US);       // 9轴: 磁北+重力参考
    bno.enableReport(SH2_GAME_ROTATION_VECTOR, REPORT_INTERVAL_US);  // 6轴: 陀螺仪+加速度计
    bno.enableReport(SH2_ACCELEROMETER, REPORT_INTERVAL_US);         // m/s^2, 含重力
    bno.enableReport(SH2_LINEAR_ACCELERATION, REPORT_INTERVAL_US);   // m/s^2, 去重力+去偏置, 用于轨迹
    bno.enableReport(SH2_GYROSCOPE_CALIBRATED, REPORT_INTERVAL_US);  // rad/s
}

void setup() {
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH);  // 一上电进入固件就常亮

    Serial.begin(115200);
    delay(200);
    Wire.begin(BNO_SDA, BNO_SCL);
    Wire.setClock(400000);  // 400kHz I2C, 为 200Hz 留出足够带宽

    Serial.println();
    Serial.println("============================================");
    Serial.printf("  IMU WiFi 推送  [%s]  AP=%s port=%d\n", ROLE, AP_SSID, TCP_PORT);
    Serial.println("============================================");

    // ---- WiFi (默认 STA 连家里路由器; -DWIFI_MODE_AP 切回 AP 离线 fallback) ----
#if defined(USE_WIFI_AP)
    WiFi.mode(WIFI_AP);
    bool ok = WiFi.softAP(AP_SSID, nullptr, AP_CHANNEL);   // 开放 AP, 无密码
    Serial.printf("[WiFi] (AP) softAP ok=%d  IP=%s\n", ok, WiFi.softAPIP().toString().c_str());
#else
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);            // 栈层自动重连(ESP32 上不总靠谱, loop 里有兜底)
    WiFi.setSleep(false);                   // 关 WiFi 省电 → 推送稳定 ~200Hz (代价: 耗电增加, 电池手环要权衡)
    // 静态 IP: 避免 DHCP 每次分不同 IP
    IPAddress staticIP(STA_IP_PREFIX, STA_IP_TAIL);
    IPAddress gateway(STA_IP_PREFIX, STA_GW_TAIL);
    IPAddress subnet(255, 255, 255, 0);
    IPAddress dns(STA_IP_PREFIX, STA_GW_TAIL);
    WiFi.config(staticIP, gateway, subnet, dns);
    Serial.printf("[WiFi] (STA) 连接 SSID=%s (固定IP 尾号 %d)...\n", WIFI_SSID, STA_IP_TAIL);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < WIFI_STA_TIMEOUT_MS) {
        delay(300);
        Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WiFi] (STA) 已连接  IP=%s  RSSI=%d dBm\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
    } else {
        Serial.printf("[WiFi] (STA) %lums 内未连上 (status=%d). loop 将持续重试.\n",
                      (unsigned long)WIFI_STA_TIMEOUT_MS, (int)WiFi.status());
        Serial.println("       若路由器未开/5GHz-only/WPA3 问题, 检查后重试或改用 -DUSE_WIFI_AP 编译.");
    }
#endif
    // ---- TCP 监听 (两种模式都要) ----
    server.begin();
    Serial.printf("[TCP]  监听 %d (等 PC 的 imu_client 连接)\n", TCP_PORT);

    // ---- 电池 ADC 初始化 (如启用) ----
#if BATTERY_ADC_PIN >= 0
    pinMode(BATTERY_ADC_PIN, INPUT);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    Serial.printf("[BATT] 电池保护已启用, 检测引脚 GPIO%d, 分压比 %.1f\n",
                  BATTERY_ADC_PIN, BATTERY_DIVIDER_RATIO);
#else
    Serial.println("[BATT] 电池保护已禁用 (未配置 BATTERY_ADC_PIN)");
#endif

    // ---- BNO08x init (自动扫 I2C 找 0x4A/0x4B) ----
    uint8_t found = 0;
    for (uint8_t a = 0x08; a <= 0x77 && !found; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0 && (a == 0x4A || a == 0x4B)) found = a;
    }
    if (found && bno.begin_I2C(found, &Wire)) {
        setReports();
        bnoOK = true;
        Serial.printf("[BNO08x] init OK @0x%02X  报告: rotation+game_rotation+accel+linear_accel+gyro\n", found);
    } else {
        Serial.println("[BNO08x] init 失败! 查 SDA/SCL/RST/VCC/GND");
    }
    Serial.println("---- 采集中, PC 端运行 imu_client.py 连接本设备 ----");
}

void loop() {
    // 0) STA 断线检测 + 重连 (AP 模式不需要, 它自己就是热点)
#if !defined(USE_WIFI_AP)
    {
        static uint32_t lastReconnect = 0;
        static bool wasConnected = (WiFi.status() == WL_CONNECTED);   // 初值对齐 setup 出口
        bool connected = (WiFi.status() == WL_CONNECTED);

        // 掉线且到重连间隔 → 主动重新关联
        if (!connected && (millis() - lastReconnect) > WIFI_RECONNECT_MS) {
            lastReconnect = millis();
            Serial.printf("[WiFi] 断线 (status=%d), 触发重连...\n", (int)WiFi.status());
            WiFi.disconnect();                  // 清旧关联(否则 begin 可能直接返回)
            WiFi.begin(WIFI_SSID, WIFI_PASS);   // 重新关联 + DHCP
        }
        // 边沿: 连上→断开 —— 停推, 丢弃死 socket
        if (wasConnected && !connected) {
            client.stop();
            Serial.println("[WiFi] WiFi 掉线 → 停推 + 重置 TCP client");
        }
        // 边沿: 断开→连上 —— 打印新 IP(DHCP 可能换了, 用户要重新填 config.json)
        if (!wasConnected && connected) {
            Serial.printf("[WiFi] (重)连上  IP=%s\n", WiFi.localIP().toString().c_str());
        }
        wasConnected = connected;
    }
#endif

    // 1) 维持 TCP 连接 (断开则重新 accept)
    // 先 stop() 清掉 stale socket, 避免 server.available() 反复返回已断开的旧 client
    if (!client.connected()) {
        client.stop();
        client = server.available();
    }

    // 2) BNO08x 自复位后报告配置会丢失, 必须重新启用
    if (bnoOK && bno.wasReset()) {
        setReports();
        Serial.println("[BNO08x] 检测到复位, 已重新启用报告");
    }

    // 3) 读 BNO08x 事件, 更新缓存
    if (bnoOK && bno.getSensorEvent(&val)) {
        switch (val.sensorId) {
            case SH2_ROTATION_VECTOR:
                qw = val.un.rotationVector.real;
                qx = val.un.rotationVector.i;
                qy = val.un.rotationVector.j;
                qz = val.un.rotationVector.k;
                qAccuracy = val.status & 0x03;
                break;
            case SH2_GAME_ROTATION_VECTOR:
                q6w = val.un.gameRotationVector.real;
                q6x = val.un.gameRotationVector.i;
                q6y = val.un.gameRotationVector.j;
                q6z = val.un.gameRotationVector.k;
                q6Accuracy = val.status & 0x03;
                break;
            case SH2_ACCELEROMETER:
                ax = val.un.accelerometer.x;
                ay = val.un.accelerometer.y;
                az = val.un.accelerometer.z;
                break;
            case SH2_LINEAR_ACCELERATION:
                lax = val.un.linearAcceleration.x;
                lay = val.un.linearAcceleration.y;
                laz = val.un.linearAcceleration.z;
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                gx = val.un.gyroscope.x;
                gy = val.un.gyroscope.y;
                gz = val.un.gyroscope.z;
                break;
        }
    }

    // 4) 电池保护检测
    checkBatteryProtection();

    // 5) 以固定频率向 PC 推 JSON (电池故障时停止推送, 保护 IMU)
    static uint32_t lastSend = 0;
    const uint32_t period = 1000 / SEND_HZ;
    uint32_t now = millis();
    if (!battery_fault && (int32_t)(now - lastSend) >= period) {
        lastSend = now;
        if (client.connected()) {
            char buf[380];
            int n = snprintf(buf, sizeof(buf),
                "{\"t\":%lu,\"q\":[%.4f,%.4f,%.4f,%.4f],"
                "\"q6\":[%.4f,%.4f,%.4f,%.4f],\"qa\":%u,\"q6a\":%u,"
                "\"a\":[%.3f,%.3f,%.3f],\"g\":[%.3f,%.3f,%.3f],"
                "\"la\":[%.3f,%.3f,%.3f]}\n",
                (unsigned long)micros(),
                qw, qx, qy, qz,
                q6w, q6x, q6y, q6z,
                (unsigned int)qAccuracy, (unsigned int)q6Accuracy,
                ax, ay, az,
                gx, gy, gz,
                lax, lay, laz);
            size_t sent = client.write((const uint8_t *)buf, (size_t)n);
            if (sent > 0) {
                frames_sent++;
            } else {
                // 写入失败(对端 RST 等), 主动关闭让下次循环重新 accept
                client.stop();
            }
        }
    }

    // 6) 状态指示灯: 正常常亮, 电池故障时快闪告警
    if (battery_fault) {
        digitalWrite(STATUS_LED_PIN, ((now / 100) % 2) ? HIGH : LOW);
    } else {
        digitalWrite(STATUS_LED_PIN, HIGH);
    }

    // 7) 状态打印 (USB, 每 3s)
    static uint32_t lastStatus = 0;
    if (now - lastStatus > 3000) {
        lastStatus = now;
        Serial.printf("[STATUS] client=%s  frames_sent=%lu  batt=%.2fV\n",
                      client.connected() ? "YES" : "no", frames_sent, battery_voltage);
    }
}
