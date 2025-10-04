import serial
import threading
import struct
import time
import numpy as np
import matplotlib.pyplot as plt 

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

class Trans:
    def __init__(self, port, baudrate = 115200):
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

        self.motor_v_1 = 0.0 # cm/s
        self.motor_v_2 = 0.0 # cm/s
        self.motor_v_3 = 0.0 # cm/s
        self.motor_v_4 = 0.0 # cm/s

        self.current_vx = 0.0 # cm/s
        self.current_vy = 0.0 # cm/s
        self.current_vz = 0.0 # cm/s

        self.current_x = 0.0 # cm
        self.current_y = 0.0 # cm
        self.current_z = 0.0 # cm
        self.current_roll = 0.0 # mrad
        self.current_pitch = 0.0 # mrad
        self.current_yaw = 0.0 # mrad

        if not self.port:
            self._auto_detect_port()
        # 连接串口
        if self.connect():
            print("Connected to", self.port)
        else:
            print("Failed to connect")

        self.history_motor_v = [[], [], [], []]
        self.history_v = [[], [], []]
        self.history_pos = [[], [], []]
        self.history_att = [[], [], []]
        self.history_time = []

        plt.ion()  # 启用交互式实时绘图
        self.fig, self.ax = plt.subplots(2, 2, figsize=(12, 8))
        
        # 设置各子图的标题和标签
        self.ax[0, 0].set_xlabel('Time')
        self.ax[0, 0].set_ylabel('Velocity (cm/s)')
        self.ax[0, 0].set_title('Motor Velocities')
        self.ax[0, 0].grid(True)
        
        self.ax[0, 1].set_xlabel('Time')
        self.ax[0, 1].set_ylabel('Velocity (cm/s)')
        self.ax[0, 1].set_title('Linear Velocities')
        self.ax[0, 1].grid(True)
        
        self.ax[1, 0].set_xlabel('Time')
        self.ax[1, 0].set_ylabel('Position (cm)')
        self.ax[1, 0].set_title('Positions')
        self.ax[1, 0].grid(True)
        
        self.ax[1, 1].set_xlabel('Time')
        self.ax[1, 1].set_ylabel('Angle (mrad)')
        self.ax[1, 1].set_title('Attitudes')
        self.ax[1, 1].grid(True)
        
        # 初始化各曲线
        self.motor_line = [None] * 4
        self.v_line = [None] * 3
        self.pos_line = [None] * 3
        self.att_line = [None] * 3
        
        colors = ['r', 'g', 'b', 'y']
        for i in range(4):
            self.motor_line[i], = self.ax[0, 0].plot([], [], color=colors[i], label=f'motor_v_{i+1}')
        for i in range(3):
            self.v_line[i], = self.ax[0, 1].plot([], [], label=f'v_{["x","y","z"][i]}')
        for i in range(3):
            self.pos_line[i], = self.ax[1, 0].plot([], [], label=f'pos_{["x","y","z"][i]}')
        for i in range(3):
            self.att_line[i], = self.ax[1, 1].plot([], [], label=f'{["roll","pitch","yaw"][i]}')
        for i in range(2):
            for j in range(2):
                self.ax[i, j].legend()
        
        # 调整布局[6](@ref)
        plt.tight_layout()

    def _auto_detect_port(self):
        """自动检测可能的USB串口设备"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if 'USB' in port.description or 'Serial' in port.description:
                self.port = port.device
                print(f"自动选择串口: {self.port}")
                return
        raise Exception("未找到可用串口设备")

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
            print(f"已连接到 {self.port}")
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def disconnect(self):
        """断开连接并清理资源"""
        print("正在断开串口连接...")

        if hasattr(self, 'recv_thread') and self.recv_thread.is_alive():
            try:
                self.recv_thread.join(timeout=0.5)  # 等待接收线程结束
                print("接收线程已停止")
            except Exception as e:
                print(f"等待接收线程时发生错误: {e}")

        # 关闭串口
        if hasattr(self, 'ser') and self.ser:
            try:
                if self.ser.is_open:
                    print("串口连接已关闭")
            except Exception as e:
                print(f"关闭串口时发生错误: {e}")
            finally:
                self.ser = None

    def send_data(self):
        """发送数据"""
        if not self.ser or not self.ser.is_open:
            print("串口未连接，无法发送数据")

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
        else :
            for i in range(5, 25, 4):
                data[i:i+4] = struct.pack('<f', 0.0)
        if self.pid_velocity:
            data[25:29] = struct.pack('<f', self.kp_v)  
            data[29:33] = struct.pack('<f', self.ki_v) 
            data[33:37] = struct.pack('<f', self.kd_v)  
            data[37:41] = struct.pack('<f', self.max_i_out_v) 
            data[41:45] = struct.pack('<f', self.max_out_v) 
        else :
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
        except Exception as e:
            print(f"发送数据失败: {e}")

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
        帧结构: [0xAA, 0x55, running, motor_v_1_high, motor_v_1_low, motor_v_2_high, motor_v_2_low, 
                motor_v_3_high, motor_v_3_low, motor_v_4_high, motor_v_4_low,
                vx_high, vx_low, vy_high, vy_low, vz_high, vz_low, 
                x_high, x_low, y_high, y_low, z_high, z_low,
                roll_high, roll_low, pitch_high, pitch_low, yaw_high, yaw_low, checksum, 0x5D]
        """
        if self.send_over == False:
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
                print(f"运行状态: {self.running}")
                print(f"电机速度: {self.motor_v_1} cm/s, {self.motor_v_2} cm/s, {self.motor_v_3} cm/s, {self.motor_v_4} cm/s")
                print(f"当前速度: {self.current_vx} cm/s, {self.current_vy} cm/s, {self.current_vz} cm/s")
                print(f"当前位置: {self.current_x} cm, {self.current_y} cm, {self.current_z} cm")
                print(f"当前姿态: {self.current_roll} mrad, {self.current_pitch} mrad, {self.current_yaw} mrad")

                # 更新历史数据
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
                self.history_time.append(time.time())
                    
        except Exception as e:
            print(f"解析数据帧错误: {e}")
    
    def plt_show(self):
        # 更新电机速度子图
        for i in range(4):
            self.motor_line[i].set_data(self.history_time, self.history_motor_v[i])
        self.ax[0, 0].relim()
        self.ax[0, 0].autoscale_view()
        
        # 更新线性速度子图
        for i in range(3):
            self.v_line[i].set_data(self.history_time, self.history_v[i])
        self.ax[0, 1].relim()
        self.ax[0, 1].autoscale_view()
        
        # 更新位置子图
        for i in range(3):
            self.pos_line[i].set_data(self.history_time, self.history_pos[i])
        self.ax[1, 0].relim()
        self.ax[1, 0].autoscale_view()
        
        # 更新姿态子图
        for i in range(3):
            self.att_line[i].set_data(self.history_time, self.history_att[i])
        self.ax[1, 1].relim()
        self.ax[1, 1].autoscale_view()
        
        # 重绘整个图框
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

if __name__ == "__main__":
    trans = Trans(port="COM5")  # 替换为实际串口
    while True:
        time.sleep(0.2)
        trans.plt_show()
        if not trans.running:
            time.sleep(0.8)
            trans.send_data()
