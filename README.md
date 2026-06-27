# 智能骑行头盔系统 / Smart Cycling Helmet System

> 🚴‍♂️ 基于 RT-Thread 的嵌入式智能骑行安全系统 —— 集成摔倒检测、GPS 定位、语音交互、LED 灯光信号与 PC 上位机实时监控。

> 🚴‍♂️ An RT-Thread based embedded smart cycling safety system with fall detection, GPS tracking, voice interaction, LED signaling, and a real-time PC monitoring station.

---

## 📖 项目简介 / Overview

**智能骑行头盔系统** 是一款面向骑行安全的嵌入式物联网项目。头盔端以 STM32F103C8T6 为主控、RT-Thread Nano 为实时操作系统，通过 MPU6050 六轴传感器实现摔倒检测与转向识别，结合 GPS 定位、超声波后方测距、ASRPRO 离线语音交互及 WS2812 全彩 LED 转向灯带，构成完整的车载安全感知与预警体系；所有传感器数据经 ESP32 WiFi 网桥实时上传至 PC 上位机，上位机基于 Python + PyQt5 构建，通过 Leaflet 交互地图实时追踪骑手位置与移动轨迹，同步显示速度曲线与骑行状态，并支持向头盔下发远程控制指令。

---

## ✨ 功能特性 / Features

### 头盔端 / Helmet Device

| 功能 | 描述 | 技术方案 |
|---|---|---|
| 🛡️ **摔倒检测** | 两阶段状态机（自由落体→撞击），误触发免疫 | MPU6050 加速度计，0.60g/1.50g 阈值 |
| 📡 **GPS 定位** | 位置、速度、海拔、UTC 时间采集 | NEO-6M，5Hz 更新率，双语句解析 |
| 🎤 **语音交互** | 语音播报转向/障碍物提醒，语音控制转向灯 | ASRPRO 离线语音模块，双向 UART 协议 |
| 💡 **LED 灯光信号** | 转向流水灯、摔倒警示闪烁、障碍物告警 | 20 颗 WS2812 全彩 LED，彩虹动画 |
| 🔊 **超声波测距** | 后方障碍物检测（2cm~450cm），近距告警 | HC-SR04 + TIM2 输入捕获 + 信号量同步 |
| 📺 **TFT 显示** | 骑行状态、GPS 坐标、加速度实时显示 | ST7735R 128×160 RGB565，中文渲染 |
| 📶 **WiFi 遥测** | 实时数据上传 PC，支持双向指令 | ESP32 TCP 网桥，mDNS 服务发现 |
| 💓 **心跳指示** | 系统运行状态指示 | GPIO LED，2s 闪烁周期 |（调试作用，实际作品中并未显示，但是可以后续添加）

### 上位机 / PC Station

| 功能 | 描述 | 技术方案 |
|---|---|---|
| 🗺️ **实时地图追踪** | Leaflet 地图显示骑手位置与移动轨迹 | Leaflet + PyQtWebEngine |
| 📈 **速度曲线** | 实时滚动速度波形图（60 采样窗口） | pyqtgraph |
| 🚨 **状态监控** | 正常骑行 / 摔倒警告，状态栏闪烁告警 | PyQt5 信号槽 |
| 🎮 **远程控制** | 向头盔发送指令（复位、模拟摔倒等） | TCP socket 双向通信 |
| 🧪 **模拟模式** | 无需硬件即可体验全部功能 | 内置模拟数据生成器 | （方便最开始去调试上位机界面）

---

## 🏗️ 系统架构 / System Architecture

```
┌──────────────────────────────────────────────────────┐
│                    智能骑行头盔系统                     │
└──────────────────────────────────────────────────────┘
                           │
    ┌──────────────────────┼──────────────────────┐
    ▼                      ▼                      ▼
┌─────────┐   ┌──────────────────┐   ┌─────────────────┐
│ GPS定位  │   │  STM32F103C8T6   │   │  PC 上位机       │
│ NEO-6M  │   │  RT-Thread Nano  │   │  Python/PyQt5   │
│ 5Hz     │   │  6线程 / 6Threads │   │  TCP Server     │
└────┬─────┘   └────────┬─────────┘   └────────┬────────┘
     │                  │                      │
     │  USART1 (9600)   │    USART3 (115200)   │ socket
     ├──────────────────┤◄═════════════════════►│ :8888
     │                  │     ESP32 WiFi 网桥   │
     │                  │                      │
     │    ┌─────────────┤                      │
     │    │  MPU6050    │                      │
     │    │  I2C1(400K) │                      │
     │    ├─────────────┤                      │
     │    │  HC-SR04    │                      │
     │    │  TIM2捕获    │                      │
     │    ├─────────────┤                      │
     │    │  ASRPRO     │                      │
     │    │  USART2(9600)│                     │
     │    ├─────────────┤                      │
     │    │  WS2812×20  │                      │
     │    │  GPIO 位带   │                      │
     │    ├─────────────┤                      │
     │    │  TFT LCD    │                      │
     │    │  ST7735 SPI │                      │
     └────┴─────────────┘                      │
                                               │
                         实时地图/轨迹/速度/状态◄┘
```

### RT-Thread 线程模型 / Thread Model

| 线程 | 优先级 | 栈 | 周期 | 职责 |
|---|---|---|---|---|
| **gps** | 18 (最高) | 2560B | 10ms 轮询 | NMEA 接收解析 ($GPRMC + $GPGGA) |
| **voice** | 19 | 1536B | 50ms 轮询 | ASRPRO 双向指令，语音播报控制 |
| **sensor** | 20 | 2560B | 100Hz 检测 | MPU6050 读取，摔倒/转向检测，TFT 刷新 |
| **turn_led** | 22 | 1024B | 状态机 | WS2812 灯带动画控制 |
| **ultrasonic** | 24 | 1536B | 2Hz | HC-SR04 距离测量 |
| **heartbeat** | 26 (最低) | 512B | 2s | 心跳 LED 闪烁 | 

---

## 🧰 硬件组成 / Hardware

| 模块 | 型号 | 接口 | 数量 | 说明 |
|---|---|---|---|---|
| 主控 MCU | STM32F103C8T6 | - | 1 | ARM Cortex-M3, 72MHz, 64KB Flash |
| 开发板 | 野火 STM32-F103 指南者 | - | 1 | BH-F103 |
| 六轴 IMU | MPU6050 | I2C1 (PB6/PB7) | 1 | 加速度计 + 陀螺仪 |
| GPS 模块 | NEO-6M | USART1 (PA9/PA10) | 1 | GPS/GLONASS |
| 超声波模块 | HC-SR04 | GPIO + TIM2 (PA0/PA1) | 1 | 后方测距 |
| 语音模块 | ASRPRO | USART2 (PA2/PA3) | 1 | 离线语音识别/合成 |
| WiFi 网桥 | ESP32 DevKit-C | USART3 (PB10/PB11) | 1 | TCP 透传 |
| LED 灯带 | WS2812 | PA8 (左) / PA11 (右) | 2×10 | 全彩可寻址 |
| TFT 屏 | ST7735R | 模拟 SPI (PA4/5/7, PB0/1) | 1 | 128×160 RGB565 |

---

## 📂 目录结构 / Directory Structure

```
Smart_Cycling_Helmet/
├── User/                       # 应用层源代码 / Application source
│   ├── main.c                  # 入口 & 6个RT-Thread线程实现
│   ├── mpu6050.c/h             # MPU6050驱动 & 摔倒/转向检测
│   ├── gps.c/h                 # GPS NMEA解析 & 5Hz配置
│   ├── hcsr04.c/h              # HC-SR04超声波驱动
│   ├── voice.c/h               # ASRPRO语音模块协议
│   ├── esp_bridge.c/h          # ESP32 WiFi网桥驱动
│   ├── Lcd_Driver.c/h          # ST7735 TFT驱动
│   ├── GUI.c/h                 # 图形库 & 中文GBK渲染
│   ├── ws2812.c/h              # WS2812驱动 & 动画控制
│   ├── led/                    # LED GPIO初始化
│   ├── stm32f10x_it.c/h        # 中断服务程序
│   ├── font.h / boot_logo.h    # 字库 & 开机logo位图
│   └── text_logo.h / status_icons.h
├── ESP32_GPS_Bridge/           # ESP32 WiFi网桥固件
│   └── ESP32_GPS_Bridge.ino    # Arduino WiFi TCP透传
├── Libraries/                  # STM32固件库
│   ├── CMSIS/                  # ARM Cortex-M3核心
│   └── FWlib/                  # ST标准外设库 v3.5.0
├── Project/RVMDK (uv5)/        # Keil MDK工程
│   ├── BH-F103.uvprojx         # Keil项目文件
│   └── RTE/RTOS/               # RT-Thread Nano配置
└── Doc/                        # 文档
    ├── readme.txt              # 原始说明
    └── analysis.txt            # 详细代码分析文档
```

---

## 🔧 构建指南 / Build Instructions

### 前置要求 / Prerequisites

- **IDE**: Keil MDK-ARM v5 或更高版本 (uVision5)
- **工具链**: ARM Compiler 5.06 update 7
- **器件包**: Keil.STM32F1xx_DFP.2.4.1
- **RTOS**: RT-Thread Nano v3.1.5 (已包含在工程中)

### 编译 / Build

1. 用 Keil MDK 打开 `Project/RVMDK (uv5)/BH-F103.uvprojx`
2. 确认 Target 为 `STM32F103C8`，输出文件名为 `定位`
3. 点击 **Build (F7)** 编译
4. 通过 ST-Link 或 DAP-Link 将固件烧录至开发板

### 模块开关 / Module Switches

在 `main.c` 中通过宏定义控制模块启停：

```c
#define MODULE_MPU6050_ENABLE    // MPU6050 摔倒检测模块
#define MODULE_HCSR04_ENABLE     // HC-SR04 超声波模块
#define MODULE_ASRPRO_ENABLE     // ASRPRO 语音模块
```

### ESP32 WiFi 网桥 / ESP32 Bridge

使用 Arduino IDE 打开 `ESP32_GPS_Bridge/ESP32_GPS_Bridge.ino`，编译烧录至 ESP32。ESP32 将作为 TCP Client 主动连接 PC 上位机的 TCP Server（端口 8888）。

---

## 🖥️ 上位机使用 / Upper Computer

上位机代码路径：`D:\all_practice\PyCharm\helmet_1`

### 安装依赖 / Install Dependencies

```bash
cd D:\all_practice\PyCharm\helmet_1
pip install -r requirements.txt
```

### 运行 / Run

```bash
python main.py
```

### 使用方式 / Usage

- **模拟模式**（默认开启）：无需硬件，自动生成模拟 GPS 数据进行演示
- **WiFi 模式**：PC 作为 TCP Server 监听 `0.0.0.0:8888`，ESP32 作为 Client 主动连接，点击"启动WiFi服务"
- **远程控制**：在命令框输入指令（如 `RESET`、`FALL_EVENT`、`LEFT_ON`），点击发送

---

## 📡 通信协议 / Communication Protocol

### 头盔 → 上位机 (Uplink)

**GPS 数据帧（键值对格式）：**
```
LAT:31.2345,LON:121.6789,TIME:12:30:45,STATUS:0,SPEED:15.2
```

**GPS 数据帧（NMEA 风格 $IH 语句）：**
```
$IH,31.2345,121.6789,12:30:45,0,15.2#
```

**传感器心跳帧：**
```
S:ACC:1.03,GYZ:2.5,STA:0  （改功能并未在上位机里面显示，后续会持续跟进）
```

### 上位机 → 头盔 (Downlink) （上位机的命令左右两个方向有误，需要调换一下）

| 指令 | 作用 |
|---|---|
| `RESET` | 系统复位 / System reset |
| `FALL_EVENT` | 触发模拟摔倒 / Trigger test fall |
| `LEFT_ON` | 左转向灯开 / Left turn on |
| `LEFT_OFF` | 左转向灯关 / Left turn off |
| `RIGHT_ON` | 右转向灯开 / Right turn on |
| `RIGHT_OFF` | 右转向灯关 / Right turn off |

---

## 🎥 演示效果

启动后：
1. 头盔 TFT 屏显示 "智能骑行系统" 开机画面 (~1.5s)
2. 进入状态界面：显示骑行状态、加速度值、GPS 经纬度
3. 骑行中：转向灯彩虹流水动画，语音播报转弯方向
4. 摔倒时：LED 红色闪烁 + 语音告警 + 上位机弹窗
5. 后方障碍物 < 2m：超声波告警 + "小心行驶" 语音提示
6. 上位机实时显示地图位置、轨迹、速度曲线

---

## 🔮 后续改进方向 / Roadmap

- [ ] 低功耗设计（电池供电优化）
- [ ] 本地数据存储（SD 卡 / Flash 日志记录）
- [ ] RT-Thread 邮箱/消息队列替代全局变量通信
- [ ] 自适应摔倒阈值校准
- [ ] ESP32 HTTP POST 云端对接（OneNET / 阿里云 IoT）
- [ ] 手机 App 端监控（蓝牙/WiFi）
- [ ] OTA 固件升级

---

## 📄 开源协议 / License

本项目采用 **MIT License** 开源。详见 [LICENSE](LICENSE) 文件。

This project is open sourced under the **MIT License**. See [LICENSE](LICENSE) for details.

---

## 👤 作者 / Author

- **作者**: Lunantic
- **GitHub**: [Your GitHub URL]
- **日期**: 2025

---

## 🙏 致谢 / Acknowledgments

- [RT-Thread](https://github.com/RT-Thread/rt-thread) — 国产开源实时操作系统
- [野火电子](https://www.firebbs.cn/) — STM32 开发板及教程
- [Leaflet](https://leafletjs.com/) — 开源地图库
- [OpenStreetMap](https://www.openstreetmap.org/) / [CartoDB](https://carto.com/) — 地图瓦片服务
- [PyQt5](https://www.riverbankcomputing.com/software/pyqt/) — Python GUI 框架
