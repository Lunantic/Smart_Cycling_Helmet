/**
 * ESP32_GPS_Bridge.ino — WiFi-UART 透明桥
 * =======================================
 *
 * 功能: 将 STM32 USART3 发来的遥测数据通过 WiFi TCP 透传到 PC 上位机，
 *       同时支持 PC → STM32 的反向命令转发。
 *
 * 硬件:
 *   ESP32 DevKit-C
 *   Serial2: RX=GPIO16, TX=GPIO17  →  STM32 USART3 (PB10=TX, PB11=RX)
 *   GPIO2: 状态 LED (快闪=等待连接, 常亮=已连接)
 *
 * 数据流:
 *   STM32 → Serial2 → ESP32 → WiFi TCP → PC (wifi_reader.py :8888)
 *   PC    → WiFi TCP → ESP32 → Serial2 → STM32
 *
 * STM32 数据格式 (键值对, 直接透传, ESP32 不做解析):
 *   LAT:31.2304,LON:121.4737,TIME:12:30:45,STATUS:0,SPEED:23.2\r\n
 *
 * 配置: 修改下方 WIFI_SSID / WIFI_PASS / SERVER_HOST / SERVER_PORT
 */

#include <WiFi.h>

/* ================================================================
 *  用户配置 — 部署时修改
 * ================================================================ */
const char* WIFI_SSID   = "最爱麻辣小龙虾";
const char* WIFI_PASS   = "123456789";
const char* SERVER_HOST = "10.247.79.53";
const int   SERVER_PORT = 8888;

/* ================================================================
 *  硬件引脚
 * ================================================================ */
#define LED_PIN     2      /* 内置 LED, LOW=亮 (大多数板子)   */
#define STM32_RX    16     /* ESP32 RX ← STM32 TX (PB10)     */
#define STM32_TX    17     /* ESP32 TX → STM32 RX (PB11)     */

/* ================================================================
 *  缓冲区大小
 * ================================================================ */
#define UART_BUF_SIZE   128   /* Serial2 接收缓冲区             */
#define TCP_BUF_SIZE    256   /* TCP 接收缓冲区                 */

/* ================================================================
 *  重连间隔 (毫秒)
 * ================================================================ */
#define WIFI_RETRY_MS    5000    /* WiFi 断线后重试间隔          */
#define TCP_RETRY_MS     3000    /* TCP 断线后重试间隔           */
#define HEARTBEAT_MS     200     /* LED 闪烁周期 (运行指示)      */

/* ================================================================
 *  全局状态
 * ================================================================ */
static WiFiClient  tcpClient;
static char        uart_buf[UART_BUF_SIZE];   /* Serial2 行缓冲       */
static int         uart_idx = 0;
static char        tcp_buf[TCP_BUF_SIZE];     /* TCP 接收缓冲         */
static int         tcp_idx  = 0;

static unsigned long last_wifi_retry  = 0;
static unsigned long last_tcp_retry   = 0;
static unsigned long last_led_blink   = 0;
static int           led_state        = 0;
static bool          wifi_was_up      = false;

/* ================================================================
 *  connectWiFi — 非阻塞式 WiFi 连接
 *  返回 true 表示已连接, false 表示正在连接或失败
 * ================================================================ */
bool connectWiFi()
{
    if (WiFi.status() == WL_CONNECTED)
        return true;

    /* 首次连接或重连 */
    if (WiFi.status() != WL_CONNECTED)
    {
        /* 只在首次调用时 begin, 后续 WiFi 库会自动重试 */
        static bool begun = false;
        if (!begun)
        {
            Serial.println("[WiFi] 开始连接...");
            WiFi.mode(WIFI_STA);
            WiFi.begin(WIFI_SSID, WIFI_PASS);
            begun = true;
        }

        /* 检查连接状态 */
        if (WiFi.status() == WL_CONNECTED)
        {
            Serial.println();
            Serial.print("[WiFi] 已连接! IP: ");
            Serial.println(WiFi.localIP());
            begun = false;  /* 下次断线可以重新 begin */
            return true;
        }
        else
        {
            /* 正在连接中, 打印点号 */
            static unsigned long last_dot = 0;
            if (millis() - last_dot > 500)
            {
                last_dot = millis();
                Serial.print(".");
            }
            return false;
        }
    }

    return (WiFi.status() == WL_CONNECTED);
}

/* ================================================================
 *  connectTCP — 连接 PC TCP 服务器
 *  返回 true 表示已连接
 * ================================================================ */
bool connectTCP()
{
    if (tcpClient.connected())
        return true;

    /* 限流重试 */
    if (millis() - last_tcp_retry < TCP_RETRY_MS)
        return false;

    last_tcp_retry = millis();
    Serial.printf("[TCP] 连接 %s:%d ...\n", SERVER_HOST, SERVER_PORT);

    if (tcpClient.connect(SERVER_HOST, SERVER_PORT))
    {
        Serial.println("[TCP] 已连接!");
        tcpClient.setTimeout(10);   /* 非阻塞读取, 10ms 超时 */
        return true;
    }
    else
    {
        Serial.println("[TCP] 连接失败, 稍后重试...");
        return false;
    }
}

/* ================================================================
 *  forwardUartToTCP — 将 Serial2 缓冲区的行透传到 TCP
 * ================================================================ */
void forwardUartToTCP()
{
    while (Serial2.available() > 0)
    {
        char c = Serial2.read();

        if (c == '\n')
        {
            /* 行结束 — 去掉尾部 \r (如果有) */
            if (uart_idx > 0 && uart_buf[uart_idx - 1] == '\r')
                uart_idx--;
            uart_buf[uart_idx] = '\0';

            if (uart_idx > 0 && tcpClient.connected())
            {
                /* 透传: 加上 \r\n 发送到 PC */
                tcpClient.print(uart_buf);
                tcpClient.print("\r\n");
            }
            uart_idx = 0;
        }
        else if (c == '\r')
        {
            /* 跳过 \r, \n 负责终止 */
        }
        else if (uart_idx < UART_BUF_SIZE - 1)
        {
            uart_buf[uart_idx++] = c;
        }
        else
        {
            /* 缓冲区溢出 — 丢弃并重同步 */
            uart_idx = 0;
        }
    }
}

/* ================================================================
 *  forwardTCPToUart — 将 TCP 收到的数据透传到 Serial2 (STM32)
 *
 *  PC 命令示例: RESET, FALL_EVENT, LED_ON, LED_OFF 等
 *  透传到 STM32, 由 STM32 决定如何处理。
 * ================================================================ */
void forwardTCPToUart()
{
    if (!tcpClient.connected())
        return;

    while (tcpClient.available() > 0)
    {
        char c = tcpClient.read();

        if (c == '\n')
        {
            if (tcp_idx > 0 && tcp_buf[tcp_idx - 1] == '\r')
                tcp_idx--;
            tcp_buf[tcp_idx] = '\0';

            if (tcp_idx > 0)
            {
                /* 透传到 STM32 */
                Serial2.print(tcp_buf);
                Serial2.print("\r\n");
                Serial.printf("[CMD→STM32] %s\n", tcp_buf);
            }
            tcp_idx = 0;
        }
        else if (c == '\r')
        {
            /* skip */
        }
        else if (tcp_idx < TCP_BUF_SIZE - 1)
        {
            tcp_buf[tcp_idx++] = c;
        }
        else
        {
            tcp_idx = 0;
        }
    }
}

/* ================================================================
 *  updateLED — 状态指示
 *   - 快速闪烁 (100ms): 等待 WiFi/TCP 连接
 *   - 常亮:           全部已连接, 正常运行
 *   - 灭:             严重错误
 * ================================================================ */
void updateLED()
{
    unsigned long now = millis();

    if (WiFi.status() == WL_CONNECTED && tcpClient.connected())
    {
        /* 全部正常 — LED 常亮 */
        digitalWrite(LED_PIN, LOW);   /* LOW = 亮 (大多数板子) */
    }
    else
    {
        /* 等待连接 — LED 快闪 */
        if (now - last_led_blink > HEARTBEAT_MS)
        {
            last_led_blink = now;
            led_state = !led_state;
            digitalWrite(LED_PIN, led_state ? LOW : HIGH);
        }
    }
}

/* ================================================================
 *  setup
 * ================================================================ */
void setup()
{
    Serial.begin(115200);               /* USB 调试输出 */
    Serial2.begin(115200,                /* STM32 USART3 */
                  SERIAL_8N1,
                  STM32_RX, STM32_TX);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);         /* LED 灭 */

    delay(500);

    Serial.println();
    Serial.println("========================================");
    Serial.println("  ESP32 WiFi-UART 透明桥");
    Serial.println("========================================");
    Serial.printf("  STM32 UART: RX=GPIO%d TX=GPIO%d @ 115200\n", STM32_RX, STM32_TX);
    Serial.printf("  PC Server:  %s:%d\n", SERVER_HOST, SERVER_PORT);
    Serial.println("========================================");

    /* 启动 WiFi 连接 (非阻塞) */
    connectWiFi();
}

/* ================================================================
 *  loop
 * ================================================================ */
void loop()
{
    bool wifi_ok  = (WiFi.status() == WL_CONNECTED);
    bool tcp_ok   = tcpClient.connected();

    /* ---- 1. WiFi 连接管理 ---- */
    if (!wifi_ok)
    {
        wifi_was_up = false;
        /* 断线后关闭 TCP */
        if (tcpClient.connected())
            tcpClient.stop();

        /* 非阻塞重连 */
        if (millis() - last_wifi_retry > WIFI_RETRY_MS)
        {
            last_wifi_retry = millis();
            connectWiFi();
        }
        delay(50);
        updateLED();
        return;  /* 没有 WiFi, 跳过后续操作 */
    }

    /* WiFi 恢复通知 */
    if (!wifi_was_up && wifi_ok)
    {
        wifi_was_up = true;
        Serial.println("[WiFi] 连接恢复");
    }

    /* ---- 2. TCP 连接管理 ---- */
    if (!tcp_ok)
    {
        connectTCP();
        if (!tcpClient.connected())
        {
            delay(50);
            updateLED();
            return;  /* 没有 TCP, 跳过数据转发 */
        }
    }

    /* ---- 3. 数据透传 ---- */
    forwardUartToTCP();    /* STM32 → PC   */
    forwardTCPToUart();    /* PC → STM32   */

    /* ---- 4. TCP 断线检测 ---- */
    if (!tcpClient.connected())
    {
        Serial.println("[TCP] 连接已断开");
    }

    /* ---- 5. LED 状态更新 ---- */
    updateLED();

    /* ---- 6. 让出 CPU ---- */
    delay(10);  /* ~100Hz 循环, 足够处理 GPS 1-10Hz 数据 */
}
