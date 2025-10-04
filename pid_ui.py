import sys
import serial
import serial.tools.list_ports
import threading
import struct
import time
import numpy as np
import csv
from datetime import datetime

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QComboBox, QPushButton, QCheckBox, QLineEdit, 
                             QGroupBox, QGridLayout, QTextEdit, QFileDialog, QMessageBox)
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg

# PID参数默认值
KP_P = 1.0
KI_P = 0.0
KD_P = 0.0
MAX_I_OUT_P = 10.0
MAX_OUT_P = 50.0

KP_V = 1.0
KI_V = 0.0
KD_V = 0.0
MAX_I_OUT_V = 10.0
MAX_OUT_V = 50.0

class SerialMonitor(QMainWindow):
    def __init__(self):
        super().__init__()
        self.trans = None
        self.setWindowTitle("PID参数调试工具")
        self.setGeometry(100, 100, 1400, 900)
        
        # 创建中央部件和主布局
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # 左侧控制面板
        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        control_layout.setAlignment(Qt.AlignTop)
        control_panel.setMaximumWidth(400)
        
        # 串口设置组
        serial_group = QGroupBox("串口设置")
        serial_layout = QGridLayout(serial_group)
        
        serial_layout.addWidget(QLabel("串口:"), 0, 0)
        self.port_combo = QComboBox()
        # self.refresh_ports()
        serial_layout.addWidget(self.port_combo, 0, 1)
        
        self.refresh_btn = QPushButton("刷新")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        serial_layout.addWidget(self.refresh_btn, 0, 2)
        
        serial_layout.addWidget(QLabel("波特率:"), 1, 0)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(['9600', '19200', '38400', '57600', '115200'])
        self.baud_combo.setCurrentText('115200')
        serial_layout.addWidget(self.baud_combo, 1, 1)
        
        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self.toggle_connection)
        serial_layout.addWidget(self.connect_btn, 1, 2)
        
        control_layout.addWidget(serial_group)
        
        # PID模式选择组
        pid_mode_group = QGroupBox("PID模式选择")
        pid_mode_layout = QVBoxLayout(pid_mode_group)
        
        self.pid_position_cb = QCheckBox("位置PID")
        self.pid_position_cb.setChecked(False)
        pid_mode_layout.addWidget(self.pid_position_cb)
        
        self.pid_velocity_cb = QCheckBox("速度PID")
        self.pid_velocity_cb.setChecked(True)
        pid_mode_layout.addWidget(self.pid_velocity_cb)
        
        control_layout.addWidget(pid_mode_group)
        
        # PID参数组
        pid_params_group = QGroupBox("PID参数设置")
        pid_params_layout = QGridLayout(pid_params_group)
        
        # 位置PID参数
        pid_params_layout.addWidget(QLabel("位置PID参数"), 0, 0, 1, 2)
        pid_params_layout.addWidget(QLabel("KP_P:"), 1, 0)
        self.kp_p_edit = QLineEdit(str(KP_P))
        pid_params_layout.addWidget(self.kp_p_edit, 1, 1)
        
        pid_params_layout.addWidget(QLabel("KI_P:"), 2, 0)
        self.ki_p_edit = QLineEdit(str(KI_P))
        pid_params_layout.addWidget(self.ki_p_edit, 2, 1)
        
        pid_params_layout.addWidget(QLabel("KD_P:"), 3, 0)
        self.kd_p_edit = QLineEdit(str(KD_P))
        pid_params_layout.addWidget(self.kd_p_edit, 3, 1)
        
        pid_params_layout.addWidget(QLabel("MAX_I_OUT_P:"), 4, 0)
        self.max_i_out_p_edit = QLineEdit(str(MAX_I_OUT_P))
        pid_params_layout.addWidget(self.max_i_out_p_edit, 4, 1)
        
        pid_params_layout.addWidget(QLabel("MAX_OUT_P:"), 5, 0)
        self.max_out_p_edit = QLineEdit(str(MAX_OUT_P))
        pid_params_layout.addWidget(self.max_out_p_edit, 5, 1)
        
        # 速度PID参数
        pid_params_layout.addWidget(QLabel("速度PID参数"), 6, 0, 1, 2)
        pid_params_layout.addWidget(QLabel("KP_V:"), 7, 0)
        self.kp_v_edit = QLineEdit(str(KP_V))
        pid_params_layout.addWidget(self.kp_v_edit, 7, 1)
        
        pid_params_layout.addWidget(QLabel("KI_V:"), 8, 0)
        self.ki_v_edit = QLineEdit(str(KI_V))
        pid_params_layout.addWidget(self.ki_v_edit, 8, 1)
        
        pid_params_layout.addWidget(QLabel("KD_V:"), 9, 0)
        self.kd_v_edit = QLineEdit(str(KD_V))
        pid_params_layout.addWidget(self.kd_v_edit, 9, 1)
        
        pid_params_layout.addWidget(QLabel("MAX_I_OUT_V:"), 10, 0)
        self.max_i_out_v_edit = QLineEdit(str(MAX_I_OUT_V))
        pid_params_layout.addWidget(self.max_i_out_v_edit, 10, 1)
        
        pid_params_layout.addWidget(QLabel("MAX_OUT_V:"), 11, 0)
        self.max_out_v_edit = QLineEdit(str(MAX_OUT_V))
        pid_params_layout.addWidget(self.max_out_v_edit, 11, 1)
        
        control_layout.addWidget(pid_params_group)
        
        # 数据保存组
        save_group = QGroupBox("数据保存")
        save_layout = QVBoxLayout(save_group)
        
        self.save_btn = QPushButton("保存PID参数到CSV")
        self.save_btn.clicked.connect(self.save_pid_params)
        save_layout.addWidget(self.save_btn)
        
        control_layout.addWidget(save_group)
        
        # 状态显示
        self.status_text = QTextEdit()
        self.status_text.setMaximumHeight(100)
        self.status_text.setReadOnly(True)
        control_layout.addWidget(serial_group)
        control_layout.addWidget(pid_mode_group)
        control_layout.addWidget(pid_params_group)
        control_layout.addWidget(save_group)
        control_layout.addWidget(QLabel("状态信息:")) # 添加标签
        control_layout.addWidget(self.status_text)    # 添加文本框
        
        # 添加到主布局
        main_layout.addWidget(control_panel)
        
        # 右侧绘图区域
        plot_widget = QWidget()
        plot_layout = QVBoxLayout(plot_widget)
        
        # 使用pyqtgraph创建绘图区域[1,5](@ref)
        self.plot_widget = pg.GraphicsLayoutWidget()
        plot_layout.addWidget(self.plot_widget)
        
        # 创建四个子图
        self.motor_plot = self.plot_widget.addPlot(title="电机速度")
        self.motor_plot.addLegend()
        self.motor_plot.setLabel('left', '速度', 'cm/s')
        self.motor_plot.setLabel('bottom', '时间', 's')
        self.plot_widget.nextRow()
        
        self.velocity_plot = self.plot_widget.addPlot(title="线性速度")
        self.velocity_plot.addLegend()
        self.velocity_plot.setLabel('left', '速度', 'cm/s')
        self.velocity_plot.setLabel('bottom', '时间', 's')
        self.plot_widget.nextRow()
        
        self.position_plot = self.plot_widget.addPlot(title="位置")
        self.position_plot.addLegend()
        self.position_plot.setLabel('left', '位置', 'cm')
        self.position_plot.setLabel('bottom', '时间', 's')
        self.plot_widget.nextRow()
        
        self.attitude_plot = self.plot_widget.addPlot(title="姿态")
        self.attitude_plot.addLegend()
        self.attitude_plot.setLabel('left', '角度', 'mrad')
        self.attitude_plot.setLabel('bottom', '时间', 's')
        
        # 初始化曲线
        colors = ['r', 'g', 'b', 'y']
        self.motor_curves = []
        for i in range(4):
            curve = self.motor_plot.plot(pen=colors[i], name=f'电机{i+1}')
            self.motor_curves.append(curve)
        
        self.velocity_curves = []
        for i, label in enumerate(['X', 'Y', 'Z']):
            curve = self.velocity_plot.plot(pen=colors[i], name=f'速度{label}')
            self.velocity_curves.append(curve)
        
        self.position_curves = []
        for i, label in enumerate(['X', 'Y', 'Z']):
            curve = self.position_plot.plot(pen=colors[i], name=f'位置{label}')
            self.position_curves.append(curve)
        
        self.attitude_curves = []
        for i, label in enumerate(['Roll', 'Pitch', 'Yaw']):
            curve = self.attitude_plot.plot(pen=colors[i], name=label)
            self.attitude_curves.append(curve)
        
        main_layout.addWidget(plot_widget, 1)
        
        # 设置定时器用于更新图表
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(100)  # 每100ms更新一次图表

        self.refresh_ports()
        
        self.log_message("界面初始化完成")
    
    def refresh_ports(self):
        """刷新可用串口列表[1,6](@ref)"""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)
        self.log_message(f"找到 {len(ports)} 个串口")
    
    def toggle_connection(self):
        """切换串口连接状态[1](@ref)"""
        if self.trans and self.trans.ser and self.trans.ser.is_open:
            self.disconnect_serial()
        else:
            self.connect_serial()
    
    def connect_serial(self):
        """连接串口"""
        port = self.port_combo.currentText()
        if not port:
            QMessageBox.warning(self, "警告", "请选择串口")
            return
        
        baudrate = int(self.baud_combo.currentText())
        
        try:
            # 创建Trans实例
            self.trans = Trans(port, baudrate)
            self.trans.pid_position = self.pid_position_cb.isChecked()
            self.trans.pid_velocity = self.pid_velocity_cb.isChecked()
            
            # 更新PID参数
            self.update_pid_params()

            # 连接串口
            if self.trans.connect():
                self.connect_btn.setText("断开")
                self.log_message(f"已连接到 {port}")
                # 发送初始数据
                self.trans.send_data()
            else:
            # 如果连接失败，将self.trans置为None，避免后续操作出错
                self.log_message("连接失败: Trans.connect() 返回了 False")
                self.trans = None
                QMessageBox.critical(self, "错误", "连接失败，请检查串口线和端口号。")
                
        except Exception as e:
            self.log_message(f"连接发生异常: {str(e)}")
            self.trans = None  # 发生异常也置为None
            QMessageBox.critical(self, "错误", f"连接错误: {str(e)}")
        
    def disconnect_serial(self):
        """断开串口连接"""
        if self.trans:
            self.trans.disconnect()
            self.trans = None
        self.connect_btn.setText("连接")
        self.log_message("已断开串口连接")
    
    def update_pid_params(self):
        """从界面更新PID参数"""
        try:
            if self.pid_position_cb.isChecked():
                self.trans.kp_p = float(self.kp_p_edit.text())
                self.trans.ki_p = float(self.ki_p_edit.text())
                self.trans.kd_p = float(self.kd_p_edit.text())
                self.trans.max_i_out_p = float(self.max_i_out_p_edit.text())
                self.trans.max_out_p = float(self.max_out_p_edit.text())
            
            if self.pid_velocity_cb.isChecked():
                self.trans.kp_v = float(self.kp_v_edit.text())
                self.trans.ki_v = float(self.ki_v_edit.text())
                self.trans.kd_v = float(self.kd_v_edit.text())
                self.trans.max_i_out_v = float(self.max_i_out_v_edit.text())
                self.trans.max_out_v = float(self.max_out_v_edit.text())
                
        except ValueError:
            QMessageBox.warning(self, "警告", "请输入有效的数字")
    
    def save_pid_params(self):
        """保存PID参数到CSV文件[3](@ref)"""
        try:
            # 获取保存路径
            file_path, _ = QFileDialog.getSaveFileName(
                self, "保存PID参数", "", "CSV文件 (*.csv)")
            
            if not file_path:
                return
            
            # 准备数据
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            data = {
                "保存时间": timestamp,
                "位置PID模式": self.pid_position_cb.isChecked(),
                "速度PID模式": self.pid_velocity_cb.isChecked(),
                "KP_P": float(self.kp_p_edit.text()),
                "KI_P": float(self.ki_p_edit.text()),
                "KD_P": float(self.kd_p_edit.text()),
                "MAX_I_OUT_P": float(self.max_i_out_p_edit.text()),
                "MAX_OUT_P": float(self.max_out_p_edit.text()),
                "KP_V": float(self.kp_v_edit.text()),
                "KI_V": float(self.ki_v_edit.text()),
                "KD_V": float(self.kd_v_edit.text()),
                "MAX_I_OUT_V": float(self.max_i_out_v_edit.text()),
                "MAX_OUT_V": float(self.max_out_v_edit.text())
            }
            
            # 写入CSV文件
            with open(file_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # 写入标题行
                writer.writerow(['参数', '值', '说明'])
                # 写入数据行
                writer.writerow(['保存时间', data['保存时间'], '数据保存时间'])
                writer.writerow(['位置PID模式', data['位置PID模式'], '是否启用位置PID'])
                writer.writerow(['速度PID模式', data['速度PID模式'], '是否启用速度PID'])
                writer.writerow(['KP_P', data['KP_P'], '位置PID比例系数'])
                writer.writerow(['KI_P', data['KI_P'], '位置PID积分系数'])
                writer.writerow(['KD_P', data['KD_P'], '位置PID微分系数'])
                writer.writerow(['MAX_I_OUT_P', data['MAX_I_OUT_P'], '位置PID积分限幅'])
                writer.writerow(['MAX_OUT_P', data['MAX_OUT_P'], '位置PID输出限幅'])
                writer.writerow(['KP_V', data['KP_V'], '速度PID比例系数'])
                writer.writerow(['KI_V', data['KI_V'], '速度PID积分系数'])
                writer.writerow(['KD_V', data['KD_V'], '速度PID微分系数'])
                writer.writerow(['MAX_I_OUT_V', data['MAX_I_OUT_V'], '速度PID积分限幅'])
                writer.writerow(['MAX_OUT_V', data['MAX_OUT_V'], '速度PID输出限幅'])
            
            self.log_message(f"PID参数已保存到: {file_path}")
            QMessageBox.information(self, "成功", "PID参数已成功保存")
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存失败: {str(e)}")
            self.log_message(f"保存失败: {str(e)}")
    
    def update_plots(self):
        """更新图表显示"""
        if self.trans and hasattr(self.trans, 'history_time') and self.trans.history_time:
            # 更新电机速度曲线
            for i in range(4):
                self.motor_curves[i].setData(self.trans.history_time, self.trans.history_motor_v[i])
            
            # 更新线性速度曲线
            for i in range(3):
                self.velocity_curves[i].setData(self.trans.history_time, self.trans.history_v[i])
            
            # 更新位置曲线
            for i in range(3):
                self.position_curves[i].setData(self.trans.history_time, self.trans.history_pos[i])
            
            # 更新姿态曲线
            for i in range(3):
                self.attitude_curves[i].setData(self.trans.history_time, self.trans.history_att[i])
    
    def log_message(self, message):
        """记录状态消息"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.status_text.append(f"[{timestamp}] {message}")
    
    def closeEvent(self, event):
        """关闭窗口时确保串口关闭[6](@ref)"""
        if self.trans and self.trans.ser and self.trans.ser.is_open:
            self.trans.disconnect()
        event.accept()

# 保留你原有的Trans类，但稍作修改以适配PyQt
class Trans:
    def __init__(self, port, baudrate=115200):
        self.ser = None
        self.port = port
        self.baudrate = baudrate

        self.running = False
        self.send_over = False

        self.pid_position = False
        self.pid_velocity = True

        if self.pid_position:
            self.kp_p = KP_P
            self.ki_p = KI_P
            self.kd_p = KD_P
            self.max_i_out_p = MAX_I_OUT_P
            self.max_out_p = MAX_OUT_P

        if self.pid_velocity:
            self.kp_v = KP_V
            self.ki_v = KI_V
            self.kd_v = KD_V
            self.max_i_out_v = MAX_I_OUT_V
            self.max_out_v = MAX_OUT_V

        self.motor_v_1 = 0.0  # cm/s
        self.motor_v_2 = 0.0  # cm/s
        self.motor_v_3 = 0.0  # cm/s
        self.motor_v_4 = 0.0  # cm/s

        self.current_vx = 0.0  # cm/s
        self.current_vy = 0.0  # cm/s
        self.current_vz = 0.0  # cm/s

        self.current_x = 0.0  # cm
        self.current_y = 0.0  # cm
        self.current_z = 0.0  # cm
        self.current_roll = 0.0  # mrad
        self.current_pitch = 0.0  # mrad
        self.current_yaw = 0.0  # mrad

        # 初始化历史数据列表
        self.history_motor_v = [[], [], [], []]
        self.history_v = [[], [], []]
        self.history_pos = [[], [], []]
        self.history_att = [[], [], []]
        self.history_time = []

    def connect(self):
        """连接串口设备"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=0.1
            )
            # 启动接收线程
            self.recv_thread = threading.Thread(target=self._receive_data)
            self.recv_thread.daemon = True
            self.recv_thread.start()
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def disconnect(self):
        """断开连接并清理资源"""
        if hasattr(self, 'recv_thread') and self.recv_thread.is_alive():
            self.recv_thread.join(timeout=0.5)

        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None

    # 保留你原有的send_data、_receive_data、_parse_status_frame等方法
    def send_data(self):
        """发送数据"""
        if not self.ser or not self.ser.is_open:
            print("串口未连接，无法发送数据")
            return False

        data = bytearray(47)
        data[0] = 0xAA  # 帧头
        data[1] = 0x55  # 帧头
        data[2] = 0x2F  # 字节数
        data[3] = 0x00  # 保留位
        data[4] = 0x00  # 保留位
        
        if self.pid_position:
            data[5:9] = struct.pack('<f', self.kp_p)
            data[9:13] = struct.pack('<f', self.ki_p)
            data[13:17] = struct.pack('<f', self.kd_p)
            data[17:21] = struct.pack('<f', self.max_i_out_p)
            data[21:25] = struct.pack('<f', self.max_out_p)
        else:
            for i in range(5, 25, 4):
                data[i:i+4] = struct.pack('<f', 0.0)
                
        if self.pid_velocity:
            data[25:29] = struct.pack('<f', self.kp_v)  
            data[29:33] = struct.pack('<f', self.ki_v) 
            data[33:37] = struct.pack('<f', self.kd_v)  
            data[37:41] = struct.pack('<f', self.max_i_out_v) 
            data[41:45] = struct.pack('<f', self.max_out_v) 
        else:
            for i in range(25, 45, 4):
                data[i:i+4] = struct.pack('<f', 0.0)

        checksum = 0x00
        for i in range(45):
            checksum ^= data[i]
        data[45] = checksum
        data[46] = 0x5D  # 帧尾
        
        try:
            self.ser.write(data)
            self.send_over = True
            print("发送数据成功")
            return True
        except Exception as e:
            print(f"发送数据失败: {e}")
            return False

    def _receive_data(self):
        """接收线程函数，持续处理串口数据"""
        buffer = bytearray()
        while self.ser and self.ser.is_open:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    buffer.extend(data)
                    
                    while len(buffer) >= 31:
                        # 查找帧头0xAA 0x55
                        start_idx = buffer.find(b'\xAA\x55')
                        if start_idx == -1:
                            buffer.clear()
                            break
                            
                        # 检查是否包含完整帧 (31字节)
                        if len(buffer) < start_idx + 31:
                            break
                            
                        # 提取帧数据
                        frame = buffer[start_idx:start_idx+31]
                        del buffer[:start_idx+31]  # 移除已处理数据

                        # 验证帧尾 (索引30应为0x5D)
                        if frame[30] != 0x5D:
                            print("警告: 无效帧尾")
                            continue
                            
                        # 解析数据帧
                        self._parse_status_frame(frame)
            except Exception as e:
                print(f"接收错误: {e}")
                time.sleep(0.2)

    def _parse_status_frame(self, frame):
        """
        解析状态数据帧 (31字节)
        """
        if not self.send_over:
            return
            
        try:
            # 解析数据
            checksum = 0   
            for i in range(28):
                checksum ^= frame[i]
            if checksum != frame[29]:
                print("警告: 校验和错误")
                return
                
            self.running = bool(frame[2])
            if self.running:
                self.motor_v_1 = struct.unpack('>h', frame[3:5])[0]
                self.motor_v_2 = struct.unpack('>h', frame[5:7])[0]
                self.motor_v_3 = struct.unpack('>h', frame[7:9])[0]
                self.motor_v_4 = struct.unpack('>h', frame[9:11])[0]
                self.current_vx = struct.unpack('>h', frame[11:13])[0]
                self.current_vy = struct.unpack('>h', frame[13:15])[0]
                self.current_vz = struct.unpack('>h', frame[15:17])[0]
                self.current_x = struct.unpack('>h', frame[17:19])[0]
                self.current_y = struct.unpack('>h', frame[19:21])[0]
                self.current_z = struct.unpack('>h', frame[21:23])[0]
                self.current_roll = struct.unpack('>h', frame[23:25])[0]
                self.current_pitch = struct.unpack('>h', frame[25:27])[0]
                self.current_yaw = struct.unpack('>h', frame[27:29])[0]

                # 更新历史数据
                current_time = time.time()
                self.history_motor_v[0].append(self.motor_v_1)
                self.history_motor_v[1].append(self.motor_v_2)
                self.history_motor_v[2].append(self.motor_v_3)
                self.history_motor_v[3].append(self.motor_v_4)
                self.history_v[0].append(self.current_vx)
                self.history_v[1].append(self.current_vy)
                self.history_v[2].append(self.current_vz)
                self.history_pos[0].append(self.current_x)
                self.history_pos[1].append(self.current_y)
                self.history_pos[2].append(self.current_z)
                self.history_att[0].append(self.current_roll)
                self.history_att[1].append(self.current_pitch)
                self.history_att[2].append(self.current_yaw)
                self.history_time.append(current_time)
                    
                # 限制历史数据长度，防止内存占用过大
                max_history_length = 1000
                if len(self.history_time) > max_history_length:
                    self.history_motor_v[0] = self.history_motor_v[0][-max_history_length:]
                    self.history_motor_v[1] = self.history_motor_v[1][-max_history_length:]
                    self.history_motor_v[2] = self.history_motor_v[2][-max_history_length:]
                    self.history_motor_v[3] = self.history_motor_v[3][-max_history_length:]
                    self.history_v[0] = self.history_v[0][-max_history_length:]
                    self.history_v[1] = self.history_v[1][-max_history_length:]
                    self.history_v[2] = self.history_v[2][-max_history_length:]
                    self.history_pos[0] = self.history_pos[0][-max_history_length:]
                    self.history_pos[1] = self.history_pos[1][-max_history_length:]
                    self.history_pos[2] = self.history_pos[2][-max_history_length:]
                    self.history_att[0] = self.history_att[0][-max_history_length:]
                    self.history_att[1] = self.history_att[1][-max_history_length:]
                    self.history_att[2] = self.history_att[2][-max_history_length:]
                    self.history_time = self.history_time[-max_history_length:]
                    
        except Exception as e:
            print(f"解析数据帧错误: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SerialMonitor()
    window.show()
    sys.exit(app.exec_())