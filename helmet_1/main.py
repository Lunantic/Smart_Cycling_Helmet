"""
智能头盔检测系统 - 程序入口模块
==============================
功能：创建 Qt 应用程序实例，设置全局样式，启动主窗口。
这是整个桌面程序的启动文件，直接运行此文件即可启动应用。

技术栈：PyQt5 (GUI框架) + pyserial (串口通信) + pyqtgraph (图表绘制)
"""

import sys
from PyQt5 import QtWidgets, QtGui
from main_window import SmartHelmetSystem  # 主窗口类，包含所有界面和业务逻辑


def main():
    """应用程序主入口函数

    执行流程：
    1. 创建 QApplication 实例（Qt 应用核心）
    2. 设置应用元信息（名称、组织名）
    3. 设置全局默认字体为 Microsoft YaHei（微软雅黑），字号9
    4. 实例化主窗口并显示
    5. 进入 Qt 事件循环（app.exec_()），直到窗口关闭
    """
    # 创建 Qt 应用程序对象，sys.argv 支持命令行参数传递
    app = QtWidgets.QApplication(sys.argv)

    # 设置应用程序名称（显示在任务管理器等位置）
    app.setApplicationName("智能头盔检测系统")
    # 设置组织名称（用于 QSettings 等配置存储）
    app.setOrganizationName("HelmetSafety")

    # 创建字体对象：微软雅黑，9号字
    # 微软雅黑是 Windows 上对中文支持很好的字体，确保界面文字清晰
    font = QtGui.QFont("Microsoft YaHei", 9)
    # 将字体应用到整个应用程序，所有控件默认继承此字体
    app.setFont(font)

    # 创建主窗口实例
    window = SmartHelmetSystem()
    # 显示窗口
    window.show()

    # 进入 Qt 事件循环，等待用户操作
    # sys.exit() 确保程序退出时返回正确的状态码
    sys.exit(app.exec_())


# Python 标准写法：只有直接运行此文件时才执行 main()
# 如果被其他模块 import，则不会自动执行
if __name__ == "__main__":
    main()
