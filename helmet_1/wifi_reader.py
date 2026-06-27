"""
WiFi TCP 数据读取模块
====================
功能：在后台线程中通过 TCP Socket 接收 ESP32 DevKit-C 通过 WiFi 发送的遥测数据，
      解析后通过 Qt 信号发送给主界面线程进行显示。

与 serial_reader.py 的差异：
  - 通信介质：WiFi (TCP) 替代 串口 (UART)
  - PC 角色：TCP Server（监听端口，等待 ESP32 连接）
  - ESP32 角色：TCP Client（主动连接 PC 的 TCP Server）

硬件通信协议（与串口版完全一致）：
  头盔设备通过 TCP 发送 ASCII 文本行，每行包含一组遥测数据。
  支持两种数据格式：
    格式一（键值对）：LAT:31.2345,LON:121.6789,TIME:12:30:45,STATUS:0,SPEED:15.2
    格式二（NMEA风格）：$IH,纬度,经度,时间,状态,速度#

依赖：仅使用 Python 标准库 socket，无需额外安装
"""

import socket
import select
from PyQt5.QtCore import QThread, pyqtSignal
from typing import Optional, Dict, Any


class WifiReaderThread(QThread):
    """WiFi TCP 数据读取线程

    继承自 QThread，在独立线程中运行 TCP Server 读取循环，避免阻塞主界面。

    通信模式：
      PC (本程序)            ESP32 DevKit-C
      ─────────────          ──────────────
      TCP Server :8888  ←──  TCP Client
      监听连接               主动连接并发送数据

    Qt 信号说明（与 SerialReaderThread 完全一致）：
    - data_received(dict): 收到并解析成功的数据
    - error_occurred(str): 发生错误时发出
    - connection_status(bool): 客户端连接状态变化

    使用示例：
        thread = WifiReaderThread("0.0.0.0", 8888)
        thread.data_received.connect(handle_data)
        thread.start()              # 启动 TCP Server
        thread.send_command("RESET") # 发送命令到 ESP32
        thread.stop()               # 停止服务
    """

    # ====== Qt 信号定义（与 SerialReaderThread 完全一致） ======
    data_received = pyqtSignal(dict)       # 收到有效数据
    error_occurred = pyqtSignal(str)       # 发生错误
    connection_status = pyqtSignal(bool)   # 客户端连接状态

    def __init__(self, host: str = "0.0.0.0", port: int = 8888, parent=None):
        """初始化 WiFi TCP 读取线程

        Args:
            host: 监听的 IP 地址，默认 "0.0.0.0" 表示接受来自所有网络接口的连接
            port: 监听的 TCP 端口号，默认 8888
            parent: Qt 父对象
        """
        super().__init__(parent)
        self.host = host              # 监听 IP
        self.port = port              # 监听端口
        self.running = False          # 线程运行标志
        self.server_socket = None     # TCP Server socket
        self.client_socket = None     # 已连接的 ESP32 客户端 socket
        self.client_address = None    # 客户端地址（用于日志）

    def run(self):
        """线程主函数（QThread 自动调用）

        执行流程：
        1. 创建 TCP Server socket，绑定端口并开始监听
        2. 进入主循环：等待客户端连接 → 接收数据行 → 解析 → 发射信号
        3. 支持断线自动重连（客户端断开后回到 accept 状态）
        4. 循环间隔 50ms，通过 select 实现非阻塞检查
        5. self.running 变为 False 时退出循环，关闭所有 socket
        """
        try:
            # ==== 创建 TCP Server Socket ====
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # SO_REUSEADDR: 允许重用 TIME_WAIT 状态的端口，方便快速重启服务
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # 设置超时：accept() 最多等待 0.5 秒，以便循环检查 self.running 标志
            self.server_socket.settimeout(0.5)

            # bind: 绑定到指定的 IP 和端口
            self.server_socket.bind((self.host, self.port))
            # listen(1): 开始监听，backlog=1 表示最多 1 个等待连接
            # （我们只连接一个 ESP32，所以 1 足够）
            self.server_socket.listen(1)

            # 通知主线程：服务已启动
            self.connection_status.emit(False)  # False = 服务运行中但暂无客户端连接
            self.running = True

            # ==== 主循环 ====
            while self.running:
                # ----- 阶段一：等待 ESP32 客户端连接 -----
                if self.client_socket is None:
                    try:
                        # accept(): 阻塞等待客户端连接（最多 0.5 秒超时）
                        client, addr = self.server_socket.accept()
                        self.client_socket = client
                        self.client_address = addr
                        # 客户端 socket 也设置超时，使 recv 可中断
                        self.client_socket.settimeout(0.5)
                        # 通知主线程：客户端已连接
                        self.connection_status.emit(True)
                    except socket.timeout:
                        # 超时是正常的，继续循环检查 self.running
                        continue
                    except Exception as e:
                        # 其他异常（如 socket 被关闭）
                        if self.running:
                            self.error_occurred.emit(f"接受连接失败: {str(e)}")
                        continue

                # ----- 阶段二：读取客户端数据 -----
                try:
                    # 使用 select 检查是否有数据可读（非阻塞）
                    # select 超时 0.1 秒，保证循环响应及时
                    readable, _, errored = select.select(
                        [self.client_socket], [], [self.client_socket], 0.1
                    )

                    if errored:
                        # 客户端异常断开
                        raise ConnectionError("客户端连接异常")

                    if readable:
                        # recv(): 接收数据，最多 4096 字节
                        data = self.client_socket.recv(4096)
                        if not data:
                            # recv 返回空 bytes 表示客户端正常断开
                            raise ConnectionError("客户端已断开")

                        # 将收到的字节解码为字符串
                        # 可能一次收到多行（如 ESP32 缓冲发送），按 \n 分割
                        text = data.decode('utf-8', errors='ignore')
                        lines = text.split('\n')

                        for line in lines:
                            line = line.strip()
                            if line:
                                # 调用解析函数（与串口版完全相同的协议）
                                parsed = self._parse_data(line)
                                if parsed:
                                    self.data_received.emit(parsed)

                except socket.timeout:
                    # recv 超时，正常情况，继续循环
                    pass
                except (ConnectionError, OSError) as e:
                    # 客户端断开连接
                    if self.running:
                        self.error_occurred.emit(f"客户端断开: {str(e)}")
                        self.connection_status.emit(False)
                    # 清理客户端 socket，回到 accept 状态等待重连
                    self._close_client()

                # 短暂休眠，降低 CPU 占用
                self.msleep(50)

        except OSError as e:
            # Socket 错误（如端口被占用）
            if self.running:
                self.error_occurred.emit(f"Socket 错误: {str(e)}")
        except Exception as e:
            # 其他未预料的异常
            if self.running:
                self.error_occurred.emit(f"WiFi 服务错误: {str(e)}")
        finally:
            # 清理所有 socket 资源
            self._close_client()
            if self.server_socket:
                try:
                    self.server_socket.close()
                except Exception:
                    pass
                self.server_socket = None

    def _close_client(self):
        """关闭客户端连接，清理资源"""
        if self.client_socket:
            try:
                self.client_socket.close()
            except Exception:
                pass
            self.client_socket = None
            self.client_address = None

    def _parse_data(self, line: str) -> Optional[Dict[str, Any]]:
        """解析接收到的原始数据行

        支持两种协议格式（与串口版完全一致）：

        格式一（键值对格式）：
            示例: "LAT:31.2345,LON:121.6789,TIME:12:30:45,STATUS:0,SPEED:15.2"
            字段: LAT(纬度), LON(经度), TIME(时间), STATUS(状态), SPEED(速度)

        格式二（NMEA风格协议）：
            示例: "$IH,31.2345,121.6789,12:30:45,0,15.2#"
            字段顺序: $IH,纬度,经度,时间,状态,速度#

        Args:
            line: 从 TCP 读取的一行原始文本

        Returns:
            解析成功返回 dict，失败返回 None
        """
        # ===== 尝试格式一：键值对格式 =====
        if ':' in line and ',' in line:
            parts = line.split(',')
            data = {}
            for part in parts:
                if ':' in part:
                    key, val = part.split(':', 1)
                    key = key.strip().upper()
                    val = val.strip()

                    if key in ('LAT', 'LON'):
                        try:
                            data[key.lower()] = float(val)
                        except ValueError:
                            pass
                    elif key == 'TIME':
                        data['time'] = val
                    elif key == 'STATUS':
                        data['status'] = '正常' if val == '0' else '摔倒'
                        try:
                            data['status_raw'] = int(val)
                        except ValueError:
                            data['status_raw'] = -1
                    elif key == 'SPEED':
                        try:
                            data['speed'] = float(val)
                        except ValueError:
                            pass

            if 'lat' in data and 'lon' in data:
                return data

        # ===== 尝试格式二：NMEA 风格协议 =====
        elif line.startswith('$IH') and '#' in line:
            content = line[1:line.index('#')]
            parts = content.split(',')

            if len(parts) >= 6 and parts[0] == 'IH':
                try:
                    data = {
                        'lat': float(parts[1]),
                        'lon': float(parts[2]),
                        'time': parts[3],
                        'status': '正常' if parts[4] == '0' else '摔倒',
                        'status_raw': int(parts[4]),
                        'speed': float(parts[5])
                    }
                    return data
                except ValueError:
                    pass

        return None

    def stop(self):
        """停止 TCP 服务和线程

        设置 running=False，关闭所有 socket 连接，等待线程退出。
        """
        self.running = False

        # 关闭客户端连接
        self._close_client()

        # 关闭服务端 socket（这会导致 accept() 抛出异常，从而退出主循环）
        if self.server_socket:
            try:
                self.server_socket.close()
            except Exception:
                pass
            self.server_socket = None

        # 等待线程结束，最多 1000 毫秒
        self.wait(1000)

    def send_command(self, command: str) -> bool:
        """向已连接的 ESP32 客户端发送控制命令

        用于远程控制头盔设备（如发送 RESET 复位指令）。
        命令以 \\r\\n 结尾，与串口版协议一致。

        Args:
            command: 要发送的命令字符串

        Returns:
            True 表示发送成功，False 表示无客户端连接或发送失败
        """
        if self.client_socket:
            try:
                self.client_socket.sendall((command + '\r\n').encode('utf-8'))
                return True
            except Exception as e:
                self.error_occurred.emit(f"发送失败: {str(e)}")
                return False
        return False

    def get_client_info(self) -> Optional[str]:
        """获取当前连接的客户端信息

        Returns:
            客户端地址字符串（如 "192.168.1.100:54321"），无连接时返回 None
        """
        if self.client_address:
            return f"{self.client_address[0]}:{self.client_address[1]}"
        return None
