import sys
import serial
import serial.tools.list_ports
import threading
import struct
import time
import numpy as np
import csv
import json
from datetime import datetime

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QComboBox, QPushButton, QCheckBox, QLineEdit, 
                             QGroupBox, QGridLayout, QTextEdit, QFileDialog, QMessageBox,
                             QTableWidget, QTableWidgetItem, QHeaderView, QScrollArea, QTabWidget)
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
        self.setGeometry(100, 100, 1600, 900)
        
        # 创建中央部件和主布局
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # 左侧控制面板 - 使用选项卡组织
        left_tabs = QTabWidget()
        left_tabs.setMaximumWidth(500)
        
        # 串口和PID设置选项卡
        basic_tab = QWidget()
        basic_layout = QVBoxLayout(basic_tab)
        
        # 串口设置组
        serial_group = QGroupBox("串口设置")
        serial_layout = QGridLayout(serial_group)
        
        serial_layout.addWidget(QLabel("串口:"), 0, 0)
        self.port_combo = QComboBox()
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
        
        basic_layout.addWidget(serial_group)
        
        # 数据发送组
        send_group = QGroupBox("数据发送")
        send_layout = QHBoxLayout(send_group)
        self.send_btn = QPushButton("发送数据")
        self.send_btn.clicked.connect(self.send_message)
        send_layout.addWidget(self.send_btn)
        basic_layout.addWidget(send_group)

        # 数据保存组
        save_group = QGroupBox("配置管理")
        save_layout = QHBoxLayout(save_group)
        
        self.import_btn = QPushButton("导入配置")
        self.import_btn.clicked.connect(self.import_config)
        save_layout.addWidget(self.import_btn)
        
        self.export_btn = QPushButton("导出配置")
        self.export_btn.clicked.connect(self.export_config)
        save_layout.addWidget(self.export_btn)
        
        basic_layout.addWidget(save_group)
        
        # 状态显示
        self.status_text = QTextEdit()
        self.status_text.setMaximumHeight(600)
        self.status_text.setReadOnly(True)
        basic_layout.addWidget(QLabel("状态信息:"))
        basic_layout.addWidget(self.status_text)
        
        left_tabs.addTab(basic_tab, "基本设置")

        # 传出数据配置选项卡
        tx_tab = QWidget()
        tx_layout = QVBoxLayout(tx_tab)
        
        tx_header = QWidget()
        tx_header_layout = QHBoxLayout(tx_header)
        tx_header_layout.addWidget(QLabel("传出数据配置 (在串口关闭时修改)"))
        self.add_tx_btn = QPushButton("+")
        self.add_tx_btn.clicked.connect(self.add_tx_field)
        tx_header_layout.addWidget(self.add_tx_btn)
        self.remove_tx_btn = QPushButton("-")
        self.remove_tx_btn.clicked.connect(self.remove_tx_field)
        tx_header_layout.addWidget(self.remove_tx_btn)
        tx_header_layout.addStretch()
        tx_layout.addWidget(tx_header)
        
        # 传出数据表格
        self.tx_table = QTableWidget()
        self.tx_table.setColumnCount(4)
        self.tx_table.setHorizontalHeaderLabels(["启用", "名称", "类型", "值"])
        self.tx_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        tx_layout.addWidget(self.tx_table)
        
        # 添加一些默认传出字段
        self.add_tx_field("目标速度", "float", 0.0)
        self.add_tx_field("控制模式", "uint8", 1)
        
        left_tabs.addTab(tx_tab, "传出数据")

        # 传入数据保存组
        receive_group = QGroupBox("数据保存")
        receive_layout = QHBoxLayout(receive_group)
        self.receive_btn = QPushButton("保存数据")
        self.receive_btn.clicked.connect(self.receive_message)
        receive_layout.addWidget(self.receive_btn)
        
        # 传入数据配置选项卡
        rx_tab = QWidget()
        rx_layout = QVBoxLayout(rx_tab)
        
        rx_header = QWidget()
        rx_header_layout = QHBoxLayout(rx_header)
        rx_header_layout.addWidget(QLabel("传入数据配置 (在串口关闭时修改)"))
        self.add_rx_btn = QPushButton("+")
        self.add_rx_btn.clicked.connect(self.add_rx_field)
        rx_header_layout.addWidget(self.add_rx_btn)
        self.remove_rx_btn = QPushButton("-")
        self.remove_rx_btn.clicked.connect(self.remove_rx_field)
        rx_header_layout.addWidget(self.remove_rx_btn)
        rx_header_layout.addStretch()
        rx_layout.addWidget(receive_group)
        rx_layout.addWidget(rx_header)
        
        # 传入数据表格
        self.rx_table = QTableWidget()
        self.rx_table.setColumnCount(2)
        self.rx_table.setHorizontalHeaderLabels(["名称", "类型"])
        self.rx_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        rx_layout.addWidget(self.rx_table)
        
        # 添加一些默认传入字段
        self.add_rx_field("电机速度1", "int16")
        self.add_rx_field("电机速度2", "int16")
        self.add_rx_field("X速度", "int16")
        
        left_tabs.addTab(rx_tab, "传入数据")
        
        # 图表刷新组
        chart_refresh = QGroupBox("刷新图表")
        refresh_layout = QHBoxLayout(chart_refresh)
        self.refresh_btn = QPushButton("刷新")
        self.refresh_btn.clicked.connect(self.refresh_chart)
        refresh_layout.addWidget(self.refresh_btn)
        
        # 图表配置选项卡
        chart_tab = QWidget()
        chart_layout = QVBoxLayout(chart_tab)
        
        chart_header = QWidget()
        chart_header_layout = QHBoxLayout(chart_header)
        chart_header_layout.addWidget(QLabel("图表配置"))
        self.add_chart_btn = QPushButton("+图表")
        self.add_chart_btn.clicked.connect(self.add_chart)
        chart_header_layout.addWidget(self.add_chart_btn)
        self.remove_chart_btn = QPushButton("-图表")
        self.remove_chart_btn.clicked.connect(self.remove_chart)
        chart_header_layout.addWidget(self.remove_chart_btn)
        chart_header_layout.addStretch()
        chart_layout.addWidget(chart_refresh)
        chart_layout.addWidget(chart_header)

        # 图表配置表格
        self.chart_table = QTableWidget()
        self.chart_table.setColumnCount(2)
        self.chart_table.setHorizontalHeaderLabels(["图表名称", "数据源"])
        self.chart_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        chart_layout.addWidget(self.chart_table)
        
        left_tabs.addTab(chart_tab, "图表配置")
        
        main_layout.addWidget(left_tabs)
        
        # 右侧绘图区域
        self.plot_widget = QWidget()
        plot_layout = QVBoxLayout(self.plot_widget)
        
        # 图表容器（动态添加图表）
        self.charts_container = QVBoxLayout()
        self.charts = []  # 存储图表对象
        
        # 初始化默认图表
        self.add_chart("电机速度", ["电机速度1", "电机速度2"])
        self.add_chart("移动速度", ["X速度"])
        
        plot_layout.addLayout(self.charts_container)
        main_layout.addWidget(self.plot_widget, 1)
        
        # 设置定时器用于更新图表
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(100)  # 每100ms更新一次图表

        self.refresh_ports()
        
        self.log_message("界面初始化完成")
    
    def refresh_ports(self):
        """刷新可用串口列表[6](@ref)"""
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
        
            # 连接串口
            if self.trans.connect():
                self.connect_btn.setText("断开")
                self.log_message(f"已连接到 {port}")

            else:
                self.log_message("连接失败: Trans.connect() 返回了 False")
                self.trans = None
                QMessageBox.critical(self, "错误", "连接失败，请检查串口线和端口号。")
                
        except Exception as e:
            self.log_message(f"连接发生异常: {str(e)}")
            self.trans = None
            QMessageBox.critical(self, "错误", f"连接错误: {str(e)}")

    def send_message(self):
        """发送数据到设备"""
        try:
            if not self.trans:
                QMessageBox.warning(self, "警告", "未连接串口")
                return
            
            # 配置传出数据
            self.setup_tx_data()
            # 配置传入数据
            self.setup_rx_data()
            # 发送数据
            if self.trans:
                if self.trans.send_data() != None:
                    self.log_message(f"数据已发送: {self.trans.send_data()}")
            else:
                self.log_message("发送失败: 未连接串口")

        except Exception as e:
            self.log_message(f"发送数据异常: {str(e)}")
            QMessageBox.critical(self, "错误", f"发送数据错误: {str(e)}")

    def setup_tx_data(self):
        """配置传出数据"""
        if not self.trans:
            return
            
        self.trans.custom_tx_fields = []
        for row in range(self.tx_table.rowCount()):
            enabled = self.tx_table.cellWidget(row, 0).isChecked()
            name = self.tx_table.item(row, 1).text()
            data_type = self.tx_table.cellWidget(row, 2).currentText()
            value_item = self.tx_table.item(row, 3)
            value = value_item.text() if value_item else ""
            
            if enabled and value:
                try:
                    # 根据类型转换值
                    if data_type == "float":
                        value = float(value)
                    elif data_type == "int16" or data_type == "int32":
                        value = int(value)
                    elif data_type == "uint8" or data_type == "uint16" or data_type == "uint32":
                        value = int(value)
                    elif data_type == "bool":
                        value = bool(int(value))
                        
                    self.trans.custom_tx_fields.append({
                        'name': name,
                        'type': data_type,
                        'value': value,
                    })
                except ValueError:
                    self.log_message(f"传出数据 '{name}' 值格式错误")
    
    def setup_rx_data(self):
        """配置传入数据"""
        if not self.trans:
            return
            
        self.trans.custom_rx_fields = []
        for row in range(self.rx_table.rowCount()):
            name = self.rx_table.item(row, 0).text()
            data_type = self.rx_table.cellWidget(row, 1).currentText()
            # offset_item = self.rx_table.item(row, 2)
            # offset = int(offset_item.text()) if offset_item and offset_item.text().isdigit() else 0
            
            self.trans.custom_rx_fields.append({
                'name': name,
                'type': data_type,
                # 'offset': offset
            })
            
            # 初始化历史数据存储
            if name not in self.trans.history_data:
                self.trans.history_data[name] = []
    
    def disconnect_serial(self):
        """断开串口连接"""
        if self.trans:
            self.trans.disconnect()
            self.trans = None
        self.connect_btn.setText("连接")
        self.log_message("已断开串口连接")
    
    def import_config(self):
        """从JSON文件导入配置"""
        try:
            file_path, _ = QFileDialog.getOpenFileName(
                self, "导入配置", "", "JSON文件 (*.json)")
            
            if not file_path:
                return
                
            with open(file_path, 'r') as f:
                config = json.load(f)
                
            # 导入传出数据配置
            if 'tx_data' in config:
                self.tx_table.setRowCount(0)
                for item in config['tx_data']:
                    self.add_tx_field(item['name'], item['type'], item['value'], item.get('enabled', True))
            
            # 导入传入数据配置
            if 'rx_data' in config:
                self.rx_table.setRowCount(0)
                for item in config['rx_data']:
                    self.add_rx_field(item['name'], item['type'])
            
            # 导入图表配置
            if 'charts' in config:
                # 清除现有图表
                for i in reversed(range(self.charts_container.count())):
                    widget = self.charts_container.itemAt(i).widget()
                    if widget:
                        widget.deleteLater()
                    self.chart_table.removeRow(i)

                self.charts = []
                
                # 添加新图表
                for chart_config in config['charts']:
                    self.add_chart(
                        chart_config['name'], 
                        chart_config['data_sources'],
                    )
            
            self.log_message(f"配置已从 {file_path} 导入")
            QMessageBox.information(self, "成功", "配置导入成功")
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"导入失败: {str(e)}")
            self.log_message(f"导入失败: {str(e)}")
    
    def export_config(self):
        """导出配置到JSON文件"""
        try:
            file_path, _ = QFileDialog.getSaveFileName(
                self, "导出配置", "config", "JSON文件 (*.json)")
            
            if not file_path:
                return
                
            config = {
                'tx_data': [],
                'rx_data': [],
                'charts': []
            }
            
            # 导出传出数据配置
            for row in range(self.tx_table.rowCount()):
                enabled = self.tx_table.cellWidget(row, 0).isChecked()
                name = self.tx_table.item(row, 1).text()
                data_type = self.tx_table.cellWidget(row, 2).currentText()
                value_item = self.tx_table.item(row, 3)
                value = value_item.text() if value_item else ""
                
                config['tx_data'].append({
                    'name': name,
                    'type': data_type,
                    'value': value,
                    'enabled': enabled
                })
            
            # 导出传入数据配置
            for row in range(self.rx_table.rowCount()):
                name = self.rx_table.item(row, 0).text()
                data_type = self.rx_table.cellWidget(row, 1).currentText()
                
                config['rx_data'].append({
                    'name': name,
                    'type': data_type,
                })
            
            # 导出图表配置
            for chart in self.charts:
                config['charts'].append({
                    'name': chart['name'],
                    'data_sources': chart['data_sources'],
                })
                
            with open(file_path, 'w') as f:
                json.dump(config, f, indent=2)
            
            self.log_message(f"配置已导出到 {file_path}")
            QMessageBox.information(self, "成功", "配置导出成功")
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"导出失败: {str(e)}")
            self.log_message(f"导出失败: {str(e)}")
    
    def add_tx_field(self, name="新参数", data_type="float", value="", enabled=True):
        """添加传出数据字段"""
        row = self.tx_table.rowCount()
        self.tx_table.insertRow(row)
        
        # 启用复选框
        enabled_cb = QCheckBox()
        enabled_cb.setChecked(enabled)
        self.tx_table.setCellWidget(row, 0, enabled_cb)
        
        # 名称
        self.tx_table.setItem(row, 1, QTableWidgetItem(name))
        
        # 类型下拉框
        type_combo = QComboBox()
        type_combo.addItems(["int16", "uint16", "int32", "uint32", "float", "uint8"])
        type_combo.setCurrentText(data_type)
        self.tx_table.setCellWidget(row, 2, type_combo)
        
        # 值
        self.tx_table.setItem(row, 3, QTableWidgetItem(str(value)))
    
    def remove_tx_field(self):
        """移除传出数据字段"""
        current_row = self.tx_table.currentRow()
        if current_row >= 0:
            self.tx_table.removeRow(current_row)
    
    def add_rx_field(self, name="新数据", data_type="int16", offset=0):
        """添加入传数据字段"""
        row = self.rx_table.rowCount()
        self.rx_table.insertRow(row)
        
        # 名称
        self.rx_table.setItem(row, 0, QTableWidgetItem(name))
        
        # 类型下拉框
        type_combo = QComboBox()
        type_combo.addItems(["int16", "uint16", "int32", "uint32", "float", "uint8"])
        type_combo.setCurrentText(data_type)
        self.rx_table.setCellWidget(row, 1, type_combo)
        
        # 字节偏移
        self.rx_table.setItem(row, 2, QTableWidgetItem(str(offset)))
    
    def remove_rx_field(self):
        """移除传入数据字段"""
        current_row = self.rx_table.currentRow()
        if current_row >= 0:
            self.rx_table.removeRow(current_row)
    
    def add_chart(self, name="新图表", data_sources=None, color='r'):
        """添加新图表"""
        if data_sources is None:
            data_sources = []
            
        # 创建图表部件
        chart_widget = pg.GraphicsLayoutWidget()
        plot = chart_widget.addPlot(title=name)
        plot.addLegend()
        plot.setLabel('left', '值', '')
        plot.setLabel('bottom', '时间', 's')
        
        # 创建曲线
        curves = []
        for i, source in enumerate(data_sources):
            curve_color = color if len(data_sources) == 1 else ['r', 'g', 'b', 'y'][i % 4]
            curve = plot.plot(pen=curve_color, name=source)
            curves.append(curve)
        
        # 添加到容器
        self.charts_container.addWidget(chart_widget)
        
        # 存储图表信息
        self.charts.append({
            'name': name,
            'widget': chart_widget,
            'plot': plot,
            'curves': curves,
            'data_sources': data_sources,
            'color': color
        })
        
        # 添加到图表配置表格
        row = self.chart_table.rowCount()
        self.chart_table.insertRow(row)
        self.chart_table.setItem(row, 0, QTableWidgetItem(name))
        self.chart_table.setItem(row, 1, QTableWidgetItem(", ".join(data_sources)))
        # self.chart_table.setItem(row, 2, QTableWidgetItem(color))
    
    def refresh_chart(self):
        """刷新图表显示"""
        try:
            self.setup_rx_data()
            for i in range(self.chart_table.rowCount()):
                name_item = self.chart_table.item(i, 0)
                sources_item = self.chart_table.item(i, 1)

                if name_item and sources_item:
                    name = name_item.text()
                    sources = [s.strip() for s in sources_item.text().split(",")]
                    color = 'b'
                    
                    if i < len(self.charts):
                        chart = self.charts[i]
                        chart['name'] = name
                        chart['data_sources'] = sources
                        chart['color'] = color
                        
                        # 更新图表标题
                        chart['plot'].setTitle(name)
                        
                        # 更新曲线
                        chart['plot'].clear()
                        chart['curves'] = []
                        for j, source in enumerate(sources):
                            curve_color = color if len(sources) == 1 else ['r', 'g', 'b', 'y'][j % 4]
                            curve = chart['plot'].plot(pen=curve_color, name=source)
                            chart['curves'].append(curve)
            self.log_message("图表已刷新")
        except Exception as e:
            self.log_message(f"刷新图表异常: {str(e)}")

    def remove_chart(self):
        """移除图表"""
        current_row = self.chart_table.currentRow()
        if current_row >= 0 and current_row < len(self.charts):
            # 从界面移除
            chart = self.charts[current_row]
            self.charts_container.removeWidget(chart['widget'])
            chart['widget'].deleteLater()
            
            # 从列表中移除
            self.charts.pop(current_row)
            self.chart_table.removeRow(current_row)
    
    def update_plots(self):
        """更新图表显示"""
        if self.trans and hasattr(self.trans, 'history_time') and self.trans.history_time:
            # 更新每个图表
            for chart in self.charts:
                for i, source in enumerate(chart['data_sources']):
                    if source in self.trans.history_data and self.trans.history_data[source]:
                        chart['curves'][i].setData(
                            self.trans.history_time, 
                            self.trans.history_data[source]
                        )
                        
    def receive_message(self):
        """保存接收的数据到CSV文件"""
        try:
            if self.trans and self.trans.history_data:
                file_path, _ = QFileDialog.getSaveFileName(
                    self, "导出配置", "received_data", "CSV文件 (*.csv)")
                with open(file_path, "w", newline="") as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(["时间"] + list(self.trans.history_data.keys()))
                    for i in range(len(self.trans.history_time)):
                        row = [self.trans.history_time[i]]
                        for key in self.trans.history_data.keys():
                            row.append(self.trans.history_data[key][i] if i < len(self.trans.history_data[key]) else "")
                        writer.writerow(row)
                self.log_message("接收的数据已保存到 received_data.csv")
        except Exception as e:
            self.log_message(f"保存接收数据异常: {str(e)}")

    def log_message(self, message):
        """记录状态消息"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.status_text.append(f"[{timestamp}] {message}")
    
    def closeEvent(self, event):
        """关闭窗口时确保串口关闭[6](@ref)"""
        if self.trans and self.trans.ser and self.trans.ser.is_open:
            self.trans.disconnect()
        event.accept()

class Trans:
    def __init__(self, port, baudrate=115200):
        self.ser = None
        self.port = port
        self.baudrate = baudrate

        self.running = False
        self.send_over = False

        # 自定义数据字段
        self.custom_tx_fields = []  # 传出数据字段
        self.custom_rx_fields = []  # 传入数据字段
        
        # 历史数据存储
        self.history_time = []
        self.history_data = {}  # 按数据源名称存储历史数据

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

    def send_data(self):
        """发送数据（包含自定义字段）"""
        if not self.ser or not self.ser.is_open:
            print("串口未连接，无法发送数据")
            return False

        # 计算总数据长度
        total_size = 5  # 帧头(2) + 数据数(1) + 校验位(1) + 帧尾(1) 
        for field in self.custom_tx_fields:
            if field['type'] in ['int16', 'uint16']:
                total_size += 3  # 1字节类型标识 + 2字节数据
            elif field['type'] in ['int32', 'uint32', 'float']:
                total_size += 5  # 1字节类型标识 + 4字节数据
            elif field['type'] == 'uint8':
                total_size += 2  # 1字节类型标识 + 1字节数据

        data = bytearray(total_size)  # 预留足够空间
        data[0] = 0xAA  # 帧头
        data[1] = 0x55  # 帧头
        data[2] = len(self.custom_tx_fields)  # 数据数

        offset = 3  # 数据起始偏移
        # 添加自定义字段数据
        for i, field in enumerate(self.custom_tx_fields):
            if field['type'] == 'int16':
                data[offset] = 0x00  # 类型标识 int16
                data[offset+1:offset+3] = struct.pack('<h', field['value'])
                offset += 3
            elif field['type'] == 'uint16':
                data[offset] = 0x01  # 类型标识 uint16
                data[offset+1:offset+3] = struct.pack('<H', field['value'])
                offset += 3
            elif field['type'] == 'int32':
                data[offset] = 0x02  # 类型标识 int32
                data[offset+1:offset+5] = struct.pack('<i', field['value'])
                offset += 5
            elif field['type'] == 'uint32':
                data[offset] = 0x03  # 类型标识 uint32
                data[offset+1:offset+5] = struct.pack('<I', field['value'])
                offset += 5
            elif field['type'] == 'float':
                data[offset] = 0x04  # 类型标识 float
                data[offset+1:offset+5] = struct.pack('<f', field['value'])
                offset += 5
            elif field['type'] == 'uint8':
                data[offset] = 0x05  # 类型标识 uint8
                data[offset+1] = field['value'] & 0xFF
                offset += 2

        total_size = offset + 2  

        data = data[:total_size]  # 截断到实际大小
        # 计算校验和（除校验位和帧尾外的所有字节）
        checksum = 0x00
        for i in range(total_size - 2):  # 不包括校验位和帧尾
            checksum ^= data[i]
        
        data[total_size - 2] = checksum  # 校验位
        data[total_size - 1] = 0x5D  # 帧尾
        
        try:
            self.ser.write(data)
            self.send_over = True
            print(data)
            print("发送数据成功")
            return data
        except Exception as e:
            print(f"发送数据失败: {e}")
            return None

    def _receive_data(self):
        """接收线程函数，持续处理串口数据"""
        buffer = bytearray()
        while self.ser and self.ser.is_open:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    buffer.extend(data)
                    
                    while len(buffer) >= 5:  # 最小帧长度
                        # 查找帧头0xAA 0x55
                        start_idx = buffer.find(b'\xAA\x55')
                        if start_idx == -1:
                            buffer.clear()
                            break
                            
                        # 提取帧数据
                        data_length = buffer[start_idx + 2]
                        frame = buffer[start_idx:start_idx + data_length]
                        del buffer[:start_idx + data_length]  # 移除已处理数据

                        # 验证帧尾
                        if frame[-1] != 0x5D:
                            print("警告: 无效帧尾")
                            continue
                            
                        # 解析数据帧
                        self._parse_status_frame(frame)
            except Exception as e:
                print(f"接收错误: {e}")
                time.sleep(0.2)

    def _parse_status_frame(self, frame):
        """解析状态数据帧"""
        if not self.send_over:
            return
            
        try:
            # 校验和验证
            checksum = 0   
            for i in range(len(frame) - 2):  # 不包括校验位和帧尾
                checksum ^= frame[i]
                
            if checksum != frame[-2]:
                print("警告: 校验和错误")
                return
                
            # 解析基础数据
            self.running = bool(frame[3])
            if self.running:
                # 解析自定义字段数据
                current_time = time.time()
                offset = 4  # 数据起始偏移
                for field in self.custom_rx_fields:
                    try:
                        if offset + 4 > len(frame):
                            continue
                            
                        if field['type'] == 'int16' and offset + 2 <= len(frame):
                            value = struct.unpack('<h', frame[offset:offset+2])[0]
                            offset += 2
                        elif field['type'] == 'uint16' and offset + 2 <= len(frame):
                            value = struct.unpack('<H', frame[offset:offset+2])[0]
                            offset += 2
                        elif field['type'] == 'int32' and offset + 4 <= len(frame):
                            value = struct.unpack('<i', frame[offset:offset+4])[0]
                            offset += 4
                        elif field['type'] == 'uint32' and offset + 4 <= len(frame):
                            value = struct.unpack('<I', frame[offset:offset+4])[0]
                            offset += 4
                        elif field['type'] == 'float' and offset + 4 <= len(frame):
                            value = struct.unpack('<f', frame[offset:offset+4])[0]
                            offset += 4
                        elif field['type'] == 'uint8' and offset + 1 <= len(frame):
                            value = frame[offset]
                            offset += 1
                        else:
                            continue

                        # 存储历史数据
                        if field['name'] not in self.history_data:
                            self.history_data[field['name']] = []
                            
                        self.history_data[field['name']].append(value)
                        
                        # 限制历史数据长度
                        if len(self.history_data[field['name']]) > 1000:
                            self.history_data[field['name']] = self.history_data[field['name']][-1000:]
                            
                    except (struct.error, IndexError) as e:
                        print(f"解析字段 {field['name']} 错误: {e}")
                        continue
                
                # 更新时间历史
                self.history_time.append(current_time)
                if len(self.history_time) > 1000:
                    self.history_time = self.history_time[-1000:]
                    
        except Exception as e:
            print(f"解析数据帧错误: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SerialMonitor()
    window.show()
    sys.exit(app.exec_())