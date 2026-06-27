"""
串口数据读取模块
===============
功能：在后台线程中通过串口（UART/RS-232）读取智能头盔硬件发送的遥测数据，
      解析后通过 Qt 信号发送给主界面线程进行显示。

硬件通信协议：
  头盔设备通过串口发送 ASCII 文本行，每行包含一组遥测数据。
  支持两种数据格式：
    格式一（键值对）：LAT:31.2345,LON:121.6789,TIME:12:30:45,STATUS:0,SPEED:15.2
    格式二（NMEA风格）：$IH,纬度,经度,时间,状态,速度#

依赖：pyserial 库用于串口通信
"""

import serial
import serial.tools.list_ports  # 用于枚举系统可用串口列表
from PyQt5.QtCore import QThread, pyqtSignal  # QThread: Qt 线程基类, pyqtSignal: Qt 信号机制
from typing import Optional, Dict, Any  # 类型注解，提高代码可读性

# 全局标记：pyserial 库是否成功导入（即是否支持串口通信）
# 如果 pyserial 安装失败，此值为 False，程序将只能使用模拟模式
SERIAL_AVAILABLE = True


class SerialReaderThread(QThread):
    """串口数据读取线程

    继承自 QThread，在独立线程中运行串口读取循环，避免阻塞主界面。

    Qt 信号说明：
    - data_received(dict): 收到并解析成功的数据，dict 包含 lat, lon, time, status, speed 等字段
    - error_occurred(str): 串口发生错误时发出，携带错误描述字符串
    - connection_status(bool): 串口连接状态变化时发出，True=已连接, False=已断开

    使用示例：
        thread = SerialReaderThread("COM3", 9600)
        thread.data_received.connect(handle_data)
        thread.start()          # 启动线程，开始读取
        thread.send_command("RESET")  # 发送命令到设备
        thread.stop()           # 停止线程
    """

    # ====== Qt 信号定义 ======
    # pyqtSignal 是 Qt 的跨线程通信机制，emit() 发出的数据会在主线程的槽函数中安全处理
    data_received = pyqtSignal(dict)       # 收到有效数据时发出
    error_occurred = pyqtSignal(str)       # 发生错误时发出
    connection_status = pyqtSignal(bool)   # 连接状态改变时发出

    def __init__(self, port: str, baudrate: int = 9600, parent=None):
        """初始化串口读取线程

        Args:
            port: 串口设备路径，Windows 上是 "COM3" 等形式，Linux 上是 "/dev/ttyUSB0"
            baudrate: 波特率（bps），默认 9600，常见值：9600, 115200, 4800
            parent: Qt 父对象，用于 Qt 对象树内存管理
        """
        super().__init__(parent)  # 调用 QThread 构造函数
        self.port = port          # 串口路径
        self.baudrate = baudrate  # 波特率
        self.running = False      # 线程运行标志，设置为 False 可停止线程
        self.serial_conn = None   # pyserial Serial 对象，线程启动后创建

    def run(self):
        """线程主函数（QThread 自动调用）

        执行流程：
        1. 打开串口（9600/115200 波特率，8数据位，无校验，1停止位）
        2. 进入循环：不断检查串口缓冲区是否有新数据
        3. 收到完整一行后调用 _parse_data() 解析
        4. 解析成功则通过 data_received 信号发送给主线程
        5. 循环间隔 50ms，避免 CPU 空转
        6. self.running 变为 False 时退出循环，关闭串口
        """
        try:
            # ==== 打开串口 ====
            # 参数说明：
            #   port: 串口设备路径
            #   baudrate: 波特率（每秒传输的符号数）
            #   timeout: 读取超时（秒），0.5 表示最多等待 0.5 秒
            #   bytesize: 数据位 = 8（每个字节 8 位）
            #   parity: 校验位 = N（无校验）
            #   stopbits: 停止位 = 1
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.5,              # 读取超时 0.5 秒
                bytesize=serial.EIGHTBITS, # 8 位数据位
                parity=serial.PARITY_NONE, # 无校验位
                stopbits=serial.STOPBITS_ONE  # 1 位停止位
            )

            # 通知主线程：串口连接成功
            self.connection_status.emit(True)
            self.running = True  # 设置运行标志

            # ==== 数据读取主循环 ====
            while self.running:
                # in_waiting 属性：返回接收缓冲区中的字节数
                # 只有缓冲区有数据时才读取，避免空读阻塞
                if self.serial_conn and self.serial_conn.in_waiting:
                    # readline(): 读取一行（以 \n 结尾），返回 bytes
                    # decode('utf-8', errors='ignore'): 将字节解码为字符串，忽略无法解码的字符
                    # strip(): 去除行首行尾的空白字符（\r, \n, 空格等）
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()

                    if line:  # 非空行才处理
                        # 调用解析函数，将原始文本转为结构化字典
                        data = self._parse_data(line)
                        if data:  # 解析成功
                            # 通过信号将数据安全地传递给主线程
                            self.data_received.emit(data)

                # 休眠 50 毫秒，降低 CPU 占用率
                # 使用 msleep 而非 time.sleep，这样 Qt 线程管理才能正常工作
                self.msleep(50)

        except Exception as e:
            # 发生任何异常（如串口被拔出、权限不足等），通过信号通知主线程
            self.error_occurred.emit(f"串口错误: {str(e)}")
            self.connection_status.emit(False)
        finally:
            # 无论正常退出还是异常退出，都要确保串口被关闭
            # 否则串口会一直被占用，其他程序无法使用
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()

    def _parse_data(self, line: str) -> Optional[Dict[str, Any]]:
        """解析串口接收到的原始数据行

        支持两种协议格式：

        格式一（键值对格式）：
            示例: "LAT:31.2345,LON:121.6789,TIME:12:30:45,STATUS:0,SPEED:15.2"
            字段说明:
                LAT   - 纬度（度，浮点数）
                LON   - 经度（度，浮点数）
                TIME  - GPS 时间（HH:MM:SS 格式）
                STATUS- 骑手状态（0=正常骑行, 1=摔倒）
                SPEED - 速度（km/h，浮点数）

        格式二（NMEA风格协议，类似 GPS 的 $GPGGA 语句）：
            示例: "$IH,31.2345,121.6789,12:30:45,0,15.2#"
            字段顺序: $IH,纬度,经度,时间,状态,速度#

        Args:
            line: 从串口读取的一行原始文本

        Returns:
            解析成功返回 dict，格式为：
            {'lat': 31.2345, 'lon': 121.6789, 'time': '12:30:45',
             'status': '正常', 'status_raw': 0, 'speed': 15.2}
            解析失败返回 None
        """

        # ===== 尝试格式一：键值对格式 =====
        # 特征判断：同时包含 ':'（键值分隔符）和 ','（字段分隔符）
        if ':' in line and ',' in line:
            parts = line.split(',')  # 按逗号分割得到各个键值对字段
            data = {}
            for part in parts:
                if ':' in part:
                    # split(':', 1): 只分割第一个冒号
                    # 因为值中可能包含冒号（虽然这里不会，但这是好习惯）
                    key, val = part.split(':', 1)
                    key = key.strip().upper()  # 键名去空格并转大写（统一处理）
                    val = val.strip()          # 值去空格

                    # 根据键名进行类型转换
                    if key in ('LAT', 'LON'):
                        data[key.lower()] = float(val)  # 经纬度转为浮点数
                    elif key == 'TIME':
                        data['time'] = val              # 时间保持字符串
                    elif key == 'STATUS':
                        # STATUS: 0=正常, 非0=摔倒
                        data['status'] = '正常' if val == '0' else '摔倒'
                        data['status_raw'] = int(val)
                    elif key == 'SPEED':
                        data['speed'] = float(val)      # 速度转为浮点数

            # 只有同时包含经纬度才认为是有效数据
            if 'lat' in data and 'lon' in data:
                return data

        # ===== 尝试格式二：NMEA 风格协议 $IH,...#  =====
        elif line.startswith('$IH') and '#' in line:
            # 提取 $ 和 # 之间的内容（去掉帧头帧尾）
            # line[1:line.index('#')]: 去掉开头的 '$'
            content = line[1:line.index('#')]
            parts = content.split(',')  # 按逗号分割各字段

            # 格式要求：至少 6 个字段，且第一个字段是 'IH'
            if len(parts) >= 6 and parts[0] == 'IH':
                try:
                    # 字段顺序：IH, 纬度, 经度, 时间, 状态, 速度
                    data = {
                        'lat': float(parts[1]),         # 纬度
                        'lon': float(parts[2]),         # 经度
                        'time': parts[3],               # GPS 时间
                        'status': '正常' if parts[4] == '0' else '摔倒',  # 骑手状态
                        'status_raw': int(parts[4]),    # 原始状态码
                        'speed': float(parts[5])        # 速度 km/h
                    }
                    return data
                except ValueError:
                    # 数值转换失败（如收到乱码导致的非数字字符）
                    pass  # 返回 None

        # 无法识别任何格式，返回 None
        return None

    def stop(self):
        """停止线程

        设置 running=False 让主循环退出，然后等待线程结束（最多 1 秒）。
        同时关闭串口连接，释放系统资源。
        """
        self.running = False  # 通知主循环退出

        # 关闭串口：确保硬件资源被释放
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()

        # wait(1000): 等待线程结束，最多 1000 毫秒
        # 给线程一个清理和退出的时间窗口
        self.wait(1000)

    def send_command(self, command: str) -> bool:
        """向串口设备发送控制命令

        用于远程控制头盔设备，如发送 RESET 复位指令等。
        命令以 \r\n（回车换行）结尾，符合常见串口设备通信习惯。

        Args:
            command: 要发送的命令字符串（不含换行符）

        Returns:
            True 表示发送成功，False 表示发送失败（串口未打开或写入出错）
        """
        # 只有在串口打开的情况下才能发送
        if self.serial_conn and self.serial_conn.is_open:
            try:
                # 写入命令 + 回车换行，编码为 UTF-8 字节
                # \r\n 是串口通信中常见的命令终止符
                self.serial_conn.write((command + '\r\n').encode('utf-8'))
                return True
            except Exception as e:
                # 写入失败（如串口突然断开），通知主线程
                self.error_occurred.emit(f"发送失败: {str(e)}")
                return False
        return False  # 串口未连接
