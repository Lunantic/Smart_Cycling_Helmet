"""
智能头盔检测系统 - 主窗口模块
============================
功能：构建桌面应用程序的主界面，包含：
  1. 实时数据展示面板（经纬度、时间、状态、速度）
  2. 实时速度曲线图表（pyqtgraph 绘制，保留最近 60 个数据点）
  3. WiFi TCP 服务控制面板（监听 IP、端口、启动/停止）
  4. 远程控制面板（发送指令、清除轨迹、重置地图、模拟报警）
  5. 嵌入式 Leaflet 地图（显示骑手位置和移动轨迹）
  6. 模拟模式（无硬件时自动生成测试数据）

核心类：SmartHelmetSystem(QMainWindow) - 应用程序主窗口

数据流：
  模拟数据 / WiFi TCP 数据 → on_data_received() → 更新 UI 标签 + 速度曲线 + 地图标记
"""

import sys
import json
import random
import threading
from datetime import datetime
from typing import Optional, Dict, Any

from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtWebEngineWidgets import QWebEngineView  # Qt WebEngine：在桌面应用中嵌入 Chromium 浏览器
from PyQt5.QtCore import QTimer, QDateTime, QUrl, pyqtSignal, QObject
import pyqtgraph as pg   # 高性能实时绘图库（用于速度曲线图表）

from wifi_reader import WifiReaderThread  # WiFi TCP 读取线程
from map_html import MAP_HTML_TEMPLATE  # 嵌入地图的 HTML 模板


# ======================== 主窗口类 ========================
class SmartHelmetSystem(QtWidgets.QMainWindow):
    """智能头盔安全监控系统主窗口

    这是一个单窗口应用程序，使用 QSplitter 分为左右两栏：
    - 左侧面板（约 380px）：数据展示 + 图表 + WiFi 控制 + 远程控制
    - 右侧面板（约 820px）：Leaflet 交互式地图

    工作模式：
    1. 模拟模式（默认）：定时器每 1.5 秒生成随机位置数据
    2. WiFi 模式：通过 WifiReaderThread 接收 ESP32 遥测数据
    """

    def __init__(self):
        """初始化主窗口

        执行流程：
        1. 调用父类 QMainWindow 构造函数
        2. 初始化成员变量（WiFi 线程、模拟定时器、数据缓存等）
        3. 构建 UI 界面（init_ui）
        4. 连接信号和槽（init_signals）
        5. 初始化 WiFi 配置（init_wifi_settings）
        6. 配置速度曲线图表样式
        7. 默认启用模拟模式
        """
        super().__init__()  # 必须调用父类构造函数

        # ====== 成员变量初始化 ======
        # wifi_thread: WiFi TCP 读取线程对象（仅在 WiFi 服务模式时创建）
        self.wifi_thread: Optional[WifiReaderThread] = None
        # simulate_timer: 模拟模式定时器（每隔一段时间生成模拟数据）
        self.simulate_timer: Optional[QTimer] = None
        # is_simulating: 是否处于模拟模式
        self.is_simulating = False
        # current_data: 缓存最新一次收到的完整数据（dict），用于重置视角等操作
        self.current_data = {}
        # trail_data: Python 端轨迹缓存（目前主要使用 JS 端缓存）
        self.trail_data = []

        # ====== 构建界面 ======
        self.init_ui()        # 创建所有 UI 控件和布局
        self.init_signals()   # 连接按钮点击、状态变化等信号
        self.init_wifi_settings()  # 初始化 WiFi 配置默认值

        # ====== 初始化速度曲线图表 ======
        self.speed_curve_data = []  # 速度数据缓存（最多保留 60 个点）

        # 设置左轴标签：速度 (km/h)
        self.speed_plot.setLabel('left', '速度 (km/h)', color='black')
        # 设置底部轴标签：采样点序号
        self.speed_plot.setLabel('bottom', '采样点', color='black')
        # 设置图表标题
        self.speed_plot.setTitle('实时速度曲线', color='black', size='12pt')
        # 显示网格线（x 和 y 方向），半透明
        self.speed_plot.showGrid(x=True, y=True, alpha=0.3)

        # 创建速度曲线对象
        # pg.mkPen: 创建画笔，color='#FF5722' 橙色，width=2 线宽
        # symbol='o': 数据点用圆形标记
        # symbolSize=5: 数据点大小 5 像素
        # symbolBrush='#FF5722': 数据点填充色为橙色
        self.speed_curve = self.speed_plot.plot(
            pen=pg.mkPen(color='#FF5722', width=2),
            symbol='o',
            symbolSize=5,
            symbolBrush='#FF5722'
        )

        # 设置坐标轴颜色为黑色（适配白色背景）
        self.speed_plot.getAxis('left').setPen(pg.mkPen(color='black', width=2))
        self.speed_plot.getAxis('bottom').setPen(pg.mkPen(color='black', width=2))
        self.speed_plot.getAxis('left').setTextPen('black')
        self.speed_plot.getAxis('bottom').setTextPen('black')

        # ====== 默认启用模拟模式 ======
        # 这样用户在没有硬件的情况下也能看到程序运行效果
        self.checkbox_simulate.setChecked(True)
        self.toggle_simulate_mode(True)

    def init_ui(self):
        """
        构建 UI 界面
        """
        self.setWindowTitle("智能头盔检测系统 - 骑手安全监控平台")
        self.setMinimumSize(1200, 700)  # 最小窗口尺寸，防止布局被压扁

        # ====== 全局样式表（QSS - Qt Style Sheet，类似 CSS） ======
        # 使用增强对比度的配色方案，确保文字清晰可见
        self.setStyleSheet("""
            /* 主窗口背景为浅灰色 */
            QMainWindow { background-color: #F0F0F0; }

            /* QGroupBox（分组框）：白色背景，圆角边框 */
            QGroupBox {
                font-weight: bold;
                border: 1px solid #888888;
                border-radius: 8px;
                margin-top: 12px;          /* 顶部留空给标题 */
                padding-top: 12px;
                background-color: #FFFFFF;
                color: #000000;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 16px;                 /* 标题靠左 */
                padding: 0 8px 0 8px;       /* 标题左右内边距 */
                color: #000000;
            }

            /* 标签：14px 黑色字体 */
            QLabel { color: #000000; font-size: 14px; }

            /* 按钮：浅灰背景，圆角 */
            QPushButton {
                background-color: #E0E0E0;
                border: 1px solid #AAAAAA;
                border-radius: 6px;
                padding: 5px 12px;
                color: #000000;
            }
            QPushButton:hover { background-color: #D0D0D0; }    /* 鼠标悬停变暗 */
            QPushButton:pressed { background-color: #C0C0C0; }  /* 按下时更暗 */

            /* 警告按钮（objectName="warningBtn"）：14px 字体 */
            QPushButton#warningBtn {
                color: #000000;
                font-size: 14px;
                margin-bottom: 6px;
            }

            /* 输入框和下拉框：白色背景，灰色边框 */
            QLineEdit, QComboBox {
                border: 1px solid #AAAAAA;
                border-radius: 4px;
                padding: 4px;
                background-color: white;
                color: #000000;
            }

            /* 状态栏：浅灰背景 */
            QStatusBar {
                background-color: #E0E0E0;
                color: #000000;
            }
        """)

        # ====== 创建中心部件和主布局 ======
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)  # QMainWindow 需要设置中心部件
        main_layout = QtWidgets.QHBoxLayout(central_widget)  # 水平布局
        main_layout.setContentsMargins(10, 10, 10, 10)  # 四周留 10px 边距
        main_layout.setSpacing(15)  # 左右面板间隔 15px

        # ========== 左侧面板构建 ==========
        left_panel = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_panel)  # 垂直布局
        left_layout.setSpacing(15)  # 组件间距
        left_layout.setContentsMargins(0, 0, 0, 0)

        # -------- 1. 实时数据区域 --------
        self.data_group = QtWidgets.QGroupBox("📡 实时数据")
        data_layout = QtWidgets.QFormLayout(self.data_group)  # 表单布局（标签-值对）
        data_layout.setSpacing(10)
        data_layout.setLabelAlignment(QtCore.Qt.AlignRight)  # 标签右对齐

        # 创建各个数据显示标签（初始值显示占位符 "--"）
        self.lat_label = QtWidgets.QLabel("--")   # 纬度显示
        self.lat_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #000000;")

        self.lon_label = QtWidgets.QLabel("--")   # 经度显示
        self.lon_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #000000;")

        self.time_label = QtWidgets.QLabel("--")  # GPS 时间显示
        self.time_label.setStyleSheet("font-family: monospace; font-size: 14px; color: #000000;")

        self.status_label = QtWidgets.QLabel("⚪ 等待数据")  # 骑手状态显示
        self.status_label.setStyleSheet(
            "font-size: 15px; font-weight: bold; padding: 4px 8px; border-radius: 16px; background-color: #F0F0F0; color: #000000;")

        self.speed_label = QtWidgets.QLabel("-- km/h")  # 速度显示
        self.speed_label.setStyleSheet("font-size: 16px; font-weight: bold; color: #FF5722;")

        self.recv_time_label = QtWidgets.QLabel("--")  # 最后更新时间显示
        self.recv_time_label.setStyleSheet("color: #000000;")

        # 将标签添加到表单布局（自动排列为两列：标签名 | 值）
        data_layout.addRow("📍 纬度:", self.lat_label)
        data_layout.addRow("📍 经度:", self.lon_label)
        data_layout.addRow("⏱️ 时间:", self.time_label)
        data_layout.addRow("🚴 状态:", self.status_label)
        data_layout.addRow("⚡ 速度:", self.speed_label)
        data_layout.addRow("📅 最后更新:", self.recv_time_label)
        left_layout.addWidget(self.data_group)

        # -------- 2. 速度曲线图表区域 --------
        chart_group = QtWidgets.QGroupBox("📈 实时速度曲线")
        chart_layout = QtWidgets.QVBoxLayout(chart_group)
        # 创建 pyqtgraph PlotWidget（高性能实时图表）
        self.speed_plot = pg.PlotWidget()
        self.speed_plot.setBackground('white')       # 白色背景
        self.speed_plot.setMinimumHeight(200)         # 最小高度 200px
        chart_layout.addWidget(self.speed_plot)
        left_layout.addWidget(chart_group)

        # -------- 3. WiFi 设置区域 --------
        self.com_group = QtWidgets.QGroupBox("📶 WiFi 设置")
        com_layout = QtWidgets.QGridLayout(self.com_group)  # 网格布局

        # 监听 IP 输入框
        self.host_input = QtWidgets.QLineEdit("0.0.0.0")
        self.host_input.setPlaceholderText("监听 IP (默认 0.0.0.0 接受所有连接)")

        # 监听端口输入框
        self.port_input = QtWidgets.QLineEdit("8888")
        self.port_input.setPlaceholderText("监听端口 (默认 8888)")

        # 启动/停止服务按钮
        self.btn_connect = QtWidgets.QPushButton("启动服务")

        # 模拟模式复选框
        self.checkbox_simulate = QtWidgets.QCheckBox("模拟模式 (无硬件)")

        # 网格布局排列（行, 列, 行跨度, 列跨度）
        com_layout.addWidget(QtWidgets.QLabel("监听 IP:"), 0, 0)
        com_layout.addWidget(self.host_input, 0, 1)
        com_layout.addWidget(QtWidgets.QLabel("监听端口:"), 1, 0)
        com_layout.addWidget(self.port_input, 1, 1)
        com_layout.addWidget(self.btn_connect, 2, 0, 1, 2)       # 占 1 行 2 列（全宽）
        com_layout.addWidget(self.checkbox_simulate, 3, 0, 1, 2) # 占 1 行 2 列（全宽）
        left_layout.addWidget(self.com_group)

        # -------- 4. 远程控制区域 --------
        self.control_group = QtWidgets.QGroupBox("🎛️ 远程控制")
        control_layout = QtWidgets.QVBoxLayout(self.control_group)

        # 命令输入行（水平布局：输入框 + 发送按钮）
        cmd_layout = QtWidgets.QHBoxLayout()
        self.cmd_input = QtWidgets.QLineEdit()
        self.cmd_input.setPlaceholderText("输入命令 (如: RESET 或 自定义指令)")  # 灰色占位提示文字
        self.btn_send_cmd = QtWidgets.QPushButton("发送")
        self.btn_send_cmd.setEnabled(False)  # 初始禁用，启动服务或开启模拟后才可用
        cmd_layout.addWidget(self.cmd_input)
        cmd_layout.addWidget(self.btn_send_cmd)

        # 辅助按钮行（水平布局：三个快捷操作按钮）
        btn_row = QtWidgets.QHBoxLayout()

        self.btn_clear_trail = QtWidgets.QPushButton("🗑️ 清除轨迹")
        # 功能：清除地图上的移动轨迹线，保留当前位置标记

        self.btn_reset_map = QtWidgets.QPushButton("🗺️ 重置地图视角")
        # 功能：将地图中心移到骑手当前位置，缩放级别 15

        self.btn_sim_fall = QtWidgets.QPushButton("⚠️ 模拟摔倒报警")
        self.btn_sim_fall.setObjectName("warningBtn")  # 设置 objectName 用于 QSS 样式选择器
        # 功能：手动触发一次摔倒报警，用于测试报警功能

        btn_row.addWidget(self.btn_clear_trail)
        btn_row.addWidget(self.btn_reset_map)
        btn_row.addWidget(self.btn_sim_fall)

        control_layout.addLayout(cmd_layout)  # 先放命令输入行
        control_layout.addLayout(btn_row)     # 再放辅助按钮行
        left_layout.addWidget(self.control_group)

        # 添加弹性空间：将上方控件推向顶部，防止被拉伸
        left_layout.addStretch()

        # 设置左侧面板宽度限制
        # 注释掉的是固定宽度方式，改用最小/最大宽度让用户可以通过分割器调整
        ## left_panel.setFixedWidth(380)
        left_panel.setMinimumWidth(400)  # 最窄 400px（保证控件不溢出）
        left_panel.setMaximumWidth(800)  # 最宽 800px（防止地图被挤压过小）

        # ========== 右侧面板：地图 ==========
        right_panel = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)  # 地图不留边距

        # QWebEngineView：Qt 的 Chromium 浏览器组件，用于渲染 Leaflet 地图
        self.web_view = QWebEngineView()
        self.web_view.setHtml(MAP_HTML_TEMPLATE)  # 加载地图 HTML
        right_layout.addWidget(self.web_view)

        # ====== 使用 QSplitter 组合左右面板 ======
        # QSplitter 允许用户拖动分割线来调整左右面板大小
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([380, 820])  # 初始比例：左侧 380px，右侧 820px
        main_layout.addWidget(splitter)

        # ====== 状态栏 ======
        self.status_bar = self.statusBar()  # 获取 QMainWindow 内置状态栏
        self.status_bar.showMessage("就绪 | 模拟模式已启用")

    def init_signals(self):
        """连接信号和槽（Signal & Slot）

        Qt 的信号槽机制类似于观察者模式：
        - 信号（Signal）：当某个事件发生时发出
        - 槽（Slot）：响应信号的函数

        此处连接所有 UI 控件的交互信号到对应的处理函数。
        """
        # 启动/停止 WiFi 服务按钮 → toggle_wifi_service()
        self.btn_connect.clicked.connect(self.toggle_wifi_service)

        # 模拟模式复选框状态变化 → on_simulate_toggled()
        # stateChanged 参数：Qt.Checked(2) 或 Qt.Unchecked(0)
        self.checkbox_simulate.stateChanged.connect(self.on_simulate_toggled)

        # 发送命令按钮 → send_remote_command()
        self.btn_send_cmd.clicked.connect(self.send_remote_command)

        # 清除轨迹按钮 → clear_map_trail()
        self.btn_clear_trail.clicked.connect(self.clear_map_trail)

        # 重置地图视角按钮 → reset_map_view()
        self.btn_reset_map.clicked.connect(self.reset_map_view)

        # 模拟摔倒按钮 → simulate_fall_event()
        self.btn_sim_fall.clicked.connect(self.simulate_fall_event)

    def init_wifi_settings(self):
        """初始化 WiFi 配置默认值

        WiFi 模式使用 Python 标准库 socket，无需额外依赖，
        因此始终可用。

        默认配置：
          - 监听 IP: 0.0.0.0（接受来自所有网络接口的连接）
          - 监听端口: 8888
        """
        # WiFi 配置使用标准库，始终可用，无需特殊处理
        # 默认值已在 init_ui 中通过 QLineEdit 设置
        self.status_bar.showMessage("就绪 | 模拟模式已启用 | WiFi 服务端口: 8888")

    def toggle_wifi_service(self):
        """启动/停止 WiFi TCP 服务的切换函数

        启动流程：
          1. 检查是否处于模拟模式（模拟模式下不能启动服务）
          2. 获取用户配置的 IP 和端口
          3. 创建 WifiReaderThread 并连接其信号
          4. 启动线程开始监听 TCP 连接

        停止流程：
          1. 调用线程的 stop() 方法
          2. 清空线程引用
          3. 恢复 UI 状态
        """
        if self.wifi_thread and self.wifi_thread.isRunning():
            # ====== 停止 WiFi 服务 ======
            self.wifi_thread.stop()       # 停止线程（关闭 socket + 等待线程退出）
            self.wifi_thread = None       # 清空引用
            self.btn_connect.setText("启动服务")  # 恢复按钮文字
            self.status_bar.showMessage("WiFi 服务已停止")
            self.btn_send_cmd.setEnabled(False)   # 停止后禁用发送按钮
        else:
            # ====== 启动 WiFi 服务 ======
            # 模拟模式和 WiFi 服务模式互斥
            if self.is_simulating:
                QtWidgets.QMessageBox.information(self, "提示", "请先关闭模拟模式再启动 WiFi 服务")
                return

            # 获取用户配置
            host = self.host_input.text().strip()
            if not host:
                host = "0.0.0.0"  # 默认值
                self.host_input.setText(host)

            port_text = self.port_input.text().strip()
            if not port_text:
                port_text = "8888"  # 默认值
                self.port_input.setText(port_text)

            try:
                port = int(port_text)
                # 端口范围验证
                if port < 1 or port > 65535:
                    QtWidgets.QMessageBox.warning(self, "错误", "端口号必须在 1-65535 之间")
                    return

                # 创建 WiFi TCP 读取线程
                self.wifi_thread = WifiReaderThread(host, port)

                # 连接线程信号到主窗口的槽函数
                # 信号名与 serial_reader 版完全一致，处理函数无需修改
                self.wifi_thread.data_received.connect(self.on_data_received)
                self.wifi_thread.error_occurred.connect(self.on_wifi_error)
                self.wifi_thread.connection_status.connect(self.on_wifi_connection_status)

                # 启动线程（开始监听 TCP 连接）
                self.wifi_thread.start()

                # 更新 UI 状态
                self.btn_connect.setText("停止服务")
                self.btn_send_cmd.setEnabled(True)
                self.status_bar.showMessage(f"WiFi 服务已启动: {host}:{port} (等待 ESP32 连接...)")
            except ValueError:
                QtWidgets.QMessageBox.warning(self, "错误", "端口号必须是整数")
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "错误", f"无法启动 WiFi 服务: {str(e)}")

    def on_simulate_toggled(self, state):
        """模拟模式复选框状态变化处理

        Args:
            state: Qt.Checked(2) 表示勾选，Qt.Unchecked(0) 表示取消勾选
        """
        self.toggle_simulate_mode(state == QtCore.Qt.Checked)

    def toggle_simulate_mode(self, enable: bool):
        """启用/关闭模拟模式的切换函数

        模拟模式说明：
          - 程序启动时默认开启模拟模式
          - 模拟模式会生成上海附近的随机位置数据
          - 模拟模式与 WiFi 服务模式互斥：开启模拟会停止 WiFi 服务

        Args:
            enable: True=启用模拟模式, False=关闭模拟模式
        """
        # 状态未改变则无需操作（防止递归触发）
        if enable == self.is_simulating:
            return

        if enable:
            # ====== 启用模拟模式 ======
            # 先断开 WiFi 连接（如果有的话）
            if self.wifi_thread and self.wifi_thread.isRunning():
                self.wifi_thread.stop()
                self.wifi_thread = None
                self.btn_connect.setText("启动服务")

            # 创建定时器：每 1500 毫秒（1.5 秒）生成一次模拟数据
            self.simulate_timer = QTimer()
            self.simulate_timer.timeout.connect(self.generate_simulate_data)
            self.simulate_timer.start(1500)  # 启动定时器

            self.is_simulating = True

            # 模拟模式下禁用 WiFi 相关控件，防止误操作
            self.btn_connect.setEnabled(False)
            self.host_input.setEnabled(False)
            self.port_input.setEnabled(False)
            self.btn_send_cmd.setEnabled(True)  # 模拟模式支持发送模拟命令
            self.status_bar.showMessage("模拟模式运行中 (生成演示数据)")
        else:
            # ====== 关闭模拟模式 ======
            if self.simulate_timer:
                self.simulate_timer.stop()      # 停止定时器
                self.simulate_timer = None      # 清空引用

            self.is_simulating = False

            # 恢复 WiFi 控件的可用状态
            self.btn_connect.setEnabled(True)
            self.host_input.setEnabled(True)
            self.port_input.setEnabled(True)
            self.btn_send_cmd.setEnabled(False)
            self.status_bar.showMessage("模拟模式已关闭 | 可启动 WiFi 服务")

        # 同步复选框状态（使用 blockSignals 防止递归触发 stateChanged）
        self.checkbox_simulate.blockSignals(True)   # 暂时阻止信号发出
        self.checkbox_simulate.setChecked(enable)    # 设置复选框状态
        self.checkbox_simulate.blockSignals(False)  # 恢复信号

    def generate_simulate_data(self):
        """生成模拟遥测数据

        模拟骑手在上海地区骑行的场景：
        - 初始位置：上海市中心 (31.2304, 121.4737)
        - 每次调用随机偏移 ±0.002 度（约 ±200 米），模拟骑行移动
        - 随机速度：0 ~ 35 km/h
        - 5% 概率触发摔倒事件，用于测试报警功能

        生成的数据结构与真实 WiFi 数据完全一致，确保两种模式的数据处理流程统一。
        """
        # 首次调用时初始化模拟位置（上海市中心坐标）
        if not hasattr(self, 'sim_lat'):
            self.sim_lat = 31.2304   # 上海纬度
            self.sim_lon = 121.4737  # 上海经度

        # 模拟位置移动：在当前位置基础上随机偏移
        # random.uniform(-0.002, 0.002): 约 ±200 米的随机偏移
        self.sim_lat += random.uniform(-0.002, 0.002)
        self.sim_lon += random.uniform(-0.002, 0.002)

        # 模拟速度：0 到 35 km/h 之间的随机值（保留一位小数）
        speed = round(random.uniform(0, 35), 1)

        # 模拟摔倒检测：5% 概率
        status_rand = random.random()  # 生成 [0, 1) 之间的随机数
        if status_rand < 0.05:
            status = "摔倒"     # 骑手摔倒
            status_raw = 1      # 原始状态码：1
        else:
            status = "正常"     # 正常骑行
            status_raw = 0      # 原始状态码：0

        # 获取当前时间（HH:MM:SS 格式）
        current_time = datetime.now().strftime("%H:%M:%S")

        # 组装数据结构（与 TCP 解析的数据结构保持一致）
        data = {
            'lat': self.sim_lat,         # 纬度（度）
            'lon': self.sim_lon,         # 经度（度）
            'time': current_time,        # GPS 时间
            'status': status,            # 骑手状态文字（"正常" 或 "摔倒"）
            'status_raw': status_raw,    # 原始状态码（0 或 1）
            'speed': speed              # 速度（km/h）
        }

        # 传递给统一的数据处理函数（与真实数据使用同一入口）
        self.on_data_received(data)

    def on_data_received(self, data: dict):
        """处理接收到的遥测数据（核心数据处理函数）

        无论数据来自 ESP32 WiFi 还是模拟生成，都通过此函数统一处理。

        处理流程：
        1. 更新 UI 标签显示（经纬度、时间、状态、速度、最后更新）
        2. 根据状态改变状态标签样式（正常=绿色，摔倒=红色闪烁）
        3. 更新速度曲线图表（追加数据点，保留最近 60 个）
        4. 调用 JS 更新地图上的骑手位置标记和轨迹线
        5. 如果检测到摔倒，在地图标记上显示警告提示
        """
        # 缓存最新数据（用于重置地图视角等功能）
        self.current_data = data

        # ====== 1. 更新 UI 标签 ======
        # 经纬度：显示 6 位小数
        self.lat_label.setText(f"{data.get('lat', 0):.6f}°")
        self.lon_label.setText(f"{data.get('lon', 0):.6f}°")
        # GPS 时间
        self.time_label.setText(data.get('time', '--'))
        # 速度：显示 1 位小数
        speed_val = data.get('speed', 0)
        self.speed_label.setText(f"{speed_val:.1f} km/h")
        # 记录数据接收的本地时间
        self.recv_time_label.setText(QDateTime.currentDateTime().toString("hh:mm:ss"))

        # ====== 2. 处理骑手状态显示 ======
        status = data.get('status', '未知')
        if status == "摔倒":
            # 摔倒状态：红色背景 + 白色粗体文字
            self.status_label.setText("⚠️ 摔倒警告 ⚠️")
            self.status_label.setStyleSheet(
                "background-color: #CC0000; color: white; font-weight: bold; padding: 4px 8px; border-radius: 16px;")
            # 触发视觉警告（状态栏闪烁）
            self.flash_warning()
        else:
            # 正常状态：绿色背景 + 白色粗体文字
            self.status_label.setText("✅ 正常骑行")
            self.status_label.setStyleSheet(
                "background-color: #2E7D32; color: white; font-weight: bold; padding: 4px 8px; border-radius: 16px;")

        # ====== 3. 更新速度曲线 ======
        self.speed_curve_data.append(speed_val)  # 追加新数据点
        # 限制数据点数量：最多保留 60 个（滑动窗口）
        # 超过 60 个时移除最早的数据点
        if len(self.speed_curve_data) > 60:
            self.speed_curve_data.pop(0)
        # 更新图表显示
        self.speed_curve.setData(self.speed_curve_data)

        # ====== 4. 更新地图上的骑手位置 ======
        lat = data.get('lat')
        lon = data.get('lon')
        if lat and lon and self.web_view:
            # 通过 runJavaScript() 调用地图页面中的 updateLocation() 函数
            # addToTrail=true: 将新位置添加到轨迹线上
            self.web_view.page().runJavaScript(f"updateLocation({lat}, {lon}, true);")

        # ====== 5. 摔倒事件的地图警告 ======
        if status == "摔倒":
            # 在地图标记上显示 "⚠️ 摔倒事件！" 工具提示，3 秒后自动消失
            self.web_view.page().runJavaScript("""
                if (currentMarker) {
                    // bindTooltip: 绑定工具提示
                    // permanent: true → 始终显示（不依赖鼠标悬停）
                    // direction: 'top' → 显示在标记上方
                    currentMarker.bindTooltip("⚠️ 摔倒事件！", { permanent: true, direction: 'top' }).openTooltip();
                    // 3 秒后自动关闭提示
                    setTimeout(() => { currentMarker.closeTooltip(); }, 3000);
                }
            """)

    def flash_warning(self):
        """视觉警告：状态栏闪烁

        通过临时改变状态栏背景色为橙色，500 毫秒后恢复原样，
        以此吸引用户注意到报警状态。
        """
        original = self.status_bar.styleSheet()  # 保存原始样式
        # 临时设置橙色背景
        self.status_bar.setStyleSheet("background-color: #FF5722; color: white;")
        # 500 毫秒后恢复原始样式
        # QTimer.singleShot: 单次定时器，只触发一次
        QTimer.singleShot(500, lambda: self.status_bar.setStyleSheet(original))
        # 状态栏显示警告文字，3000 毫秒后自动消失
        self.status_bar.showMessage("⚠️ 摔倒报警！请立即确认骑手状态！", 3000)

    def on_wifi_error(self, error_msg):
        """WiFi 服务错误处理

        当 TCP 服务发生异常（如端口被占用、客户端断连）时调用。
        弹出警告对话框，停止服务，恢复 UI 状态。

        Args:
            error_msg: WifiReaderThread 通过 error_occurred 信号发送的错误描述
        """
        # 弹出警告对话框
        QtWidgets.QMessageBox.warning(self, "WiFi 服务错误", error_msg)

        # 清理 WiFi 线程（仅在客户端断开等可恢复错误时）
        # 端口被占用等严重错误需要用户手动干预
        if self.wifi_thread:
            # 对于客户端断开，不停止服务，等待重连
            # 只更新状态文字
            self.status_bar.showMessage(f"⚠️ {error_msg}")

    def on_wifi_connection_status(self, connected):
        """WiFi 客户端连接状态变化处理

        Args:
            connected: True=ESP32 客户端已连接, False=客户端断开或等待连接中
        """
        if connected:
            # 显示客户端信息
            client_info = self.wifi_thread.get_client_info() if self.wifi_thread else "未知"
            self.status_bar.showMessage(f"ESP32 已连接 ({client_info})，接收数据中...")
        else:
            self.status_bar.showMessage("WiFi 服务运行中，等待 ESP32 连接...")

    def send_remote_command(self):
        """发送远程控制命令到头盔设备

        支持两种模式：
        - 模拟模式：RESET 命令清除轨迹，其他命令弹出确认提示
        - WiFi 模式：通过 WifiReaderThread.send_command() 写入 TCP socket

        命令通过 \r\n 结尾，保持与原有协议兼容。
        """
        cmd = self.cmd_input.text().strip()  # 获取输入的命令并去除首尾空白
        if not cmd:  # 空命令不处理
            return

        if self.is_simulating:
            # ====== 模拟模式下的命令处理 ======
            if cmd.upper() == "RESET":
                # RESET: 清除地图轨迹（模拟复位操作）
                self.clear_map_trail()
                QtWidgets.QMessageBox.information(self, "模拟控制", "模拟复位指令已执行")
            else:
                # 其他命令：仅弹窗确认，不执行实际操作
                QtWidgets.QMessageBox.information(self, "模拟控制", f"[模拟] 命令已发送: {cmd}")
        else:
            # ====== WiFi 模式下的命令处理 ======
            if self.wifi_thread and self.wifi_thread.isRunning():
                # 通过 TCP 发送命令到 ESP32
                if self.wifi_thread.send_command(cmd):
                    self.status_bar.showMessage(f"已发送: {cmd}", 2000)  # 2 秒后消失
                else:
                    QtWidgets.QMessageBox.warning(self, "错误", "发送失败，请检查 ESP32 连接")
            else:
                QtWidgets.QMessageBox.warning(self, "错误", "WiFi 服务未启动")

        # 清空输入框（无论发送成功与否）
        self.cmd_input.clear()

    def clear_map_trail(self):
        """清除地图上的移动轨迹线

        调用 Leaflet 地图的 clearTrail() JS 函数，清空轨迹点数组和折线。
        注意：不会清除当前位置标记，只清除历史路径。
        """
        self.web_view.page().runJavaScript("clearTrail();")
        self.status_bar.showMessage("轨迹已清除", 1500)  # 1.5 秒后消失

    def reset_map_view(self):
        """重置地图视角到骑手当前位置

        如果存在当前数据，则以骑手位置为中心、缩放级别 15（街区级别）。
        如果没有数据，则回到默认的上海中心视图。
        """
        if self.current_data and 'lat' in self.current_data and 'lon' in self.current_data:
            lat = self.current_data['lat']
            lon = self.current_data['lon']
            # 调用 JS 的 resetView() 函数
            self.web_view.page().runJavaScript(f"resetView({lat}, {lon});")
        else:
            # 无数据时回退到默认中心（上海，缩放级别 13）
            self.web_view.page().runJavaScript("map.setView([31.23, 121.47], 13);")

    def simulate_fall_event(self):
        """手动触发一次模拟摔倒报警

        功能说明：
        - 构造一个假的摔倒数据包
        - 如果当前有位置数据，使用当前位置；否则使用默认的上海市中心坐标
        - 通过 on_data_received() 处理，与真实数据走同一流程
        - 在 WiFi 模式下还会通过 TCP 发送 "FALL_EVENT" 指令到 ESP32 设备

        用途：测试报警功能是否正常工作
        """
        # 构造摔倒事件数据
        # 优先使用当前缓存的位置数据，没有则使用默认坐标
        fake_fall = {
            'lat': self.current_data.get('lat', 31.2304) if self.current_data else 31.2304,
            'lon': self.current_data.get('lon', 121.4737) if self.current_data else 121.4737,
            'time': datetime.now().strftime("%H:%M:%S"),
            'status': "摔倒",         # 强制设置为摔倒状态
            'status_raw': 1,          # 原始状态码为 1
            'speed': self.current_data.get('speed', 12.0) if self.current_data else 12.0
        }

        # 通过标准数据通道处理（与收到真实数据完全一致的流程）
        self.on_data_received(fake_fall)

        # 如果当前是 WiFi 模式且有活跃连接，同时向硬件发送摔倒事件通知
        if not self.is_simulating and self.wifi_thread and self.wifi_thread.isRunning():
            self.wifi_thread.send_command("FALL_EVENT")

        self.status_bar.showMessage("模拟摔倒报警已触发", 2000)

    def closeEvent(self, event):
        """窗口关闭事件处理（重写 QMainWindow.closeEvent）

        程序退出前的清理工作：
        1. 停止 WiFi 线程（防止后台线程阻止程序退出）
        2. 停止模拟定时器
        3. 接受关闭事件（允许窗口正常关闭）

        Args:
            event: Qt 关闭事件对象
        """
        # 停止 WiFi 读取线程（如果有的话）
        if self.wifi_thread and self.wifi_thread.isRunning():
            self.wifi_thread.stop()

        # 停止模拟数据定时器（如果有的话）
        if self.simulate_timer:
            self.simulate_timer.stop()

        # 接受关闭事件，允许窗口正常关闭
        event.accept()
