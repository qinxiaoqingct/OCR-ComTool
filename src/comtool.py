import sys
import time
from PyQt5.QtWidgets import QApplication, QMainWindow, QGroupBox, QComboBox, QPushButton, QVBoxLayout, QHBoxLayout, \
    QLabel, QTextEdit, QFileDialog, QWidget,QTextEdit
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtCore import QIODevice, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QTextCursor
import serial
import serial.tools.list_ports
import os

# 串口参数
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # 初始化串口
        # self.serial = serial.Serial(port=port, baudrate=baud_rate, bytesize=data_bits,
        #             parity=parity, stopbits=stop_bits, timeout=1)

        # 设置窗口标题和大小
        self.setWindowTitle("OCR文本解析工具")
        self.resize(800, 600)

        # 初始化界面
        self.init_ui()
        self.init_protocol()

        # 初始化串口
        self.init_serial()

    def init_ui(self):
        # 初始化串口设置框
        self.serial_settings_box = QGroupBox("串口设置")
        self.port_label = QLabel("串口号:")
        self.port_combobox = QComboBox()
        self.refresh_button = QPushButton("刷新")
        self.baudrate_label = QLabel("波特率:")
        self.baudrate_combobox = QComboBox()
        self.databits_label = QLabel("数据位:")
        self.databits_combobox = QComboBox()
        self.stopbits_label = QLabel("停止位:")
        self.stopbits_combobox = QComboBox()
        self.parity_label = QLabel("校验位:")
        self.parity_combobox = QComboBox()
        self.open_button = QPushButton("打开串口")
        self.close_button = QPushButton("关闭串口")
        self.open_button.clicked.connect(self.open_serial)
        self.close_button.clicked.connect(self.close_serial)
        self.refresh_button.clicked.connect(self.refresh_serial_ports)
        hbox = QHBoxLayout()
        hbox.addWidget(self.port_label)
        hbox.addWidget(self.port_combobox)
        hbox.addWidget(self.refresh_button)
        hbox.addWidget(self.baudrate_label)
        hbox.addWidget(self.baudrate_combobox)
        hbox.addWidget(self.databits_label)
        hbox.addWidget(self.databits_combobox)
        hbox.addWidget(self.stopbits_label)
        hbox.addWidget(self.stopbits_combobox)
        hbox.addWidget(self.parity_label)
        hbox.addWidget(self.parity_combobox)
        hbox.addWidget(self.open_button)
        hbox.addWidget(self.close_button)
        self.serial_settings_box.setLayout(hbox)
        
        self.ocr_box = QGroupBox("OCR结果")
        # 创建文本框
        self.receive_text_text_browser = QTextEdit(self)
        self.receive_text_text_browser.setReadOnly(True)
        self.clear_receive_text_button = QPushButton("清除")
        self.clear_receive_text_button.clicked.connect(self.clear_receive_data)
        hbox = QHBoxLayout()
        hbox.addWidget(self.receive_text_text_browser)
        hbox.addWidget(self.clear_receive_text_button)
        self.ocr_box.setLayout(hbox)

        # 初始化主界面布局
        vbox = QVBoxLayout()
        vbox.addWidget(self.serial_settings_box)
        vbox.addWidget(self.ocr_box)
        widget = QWidget()
        widget.setLayout(vbox)
        self.setCentralWidget(widget)

        # 初始化状态栏
        self.statusBar().showMessage("就绪")

    def init_serial(self):
        # 初始化串口信息
        self.serial = QSerialPort(self)
        # self.serial.setBaudRate(self.baud_rate)
        self.serial.setReadBufferSize(1024)
        # self.serial.readyRead.connect(self.receive_data)
        self.serial.errorOccurred.connect(self.handle_serial_error)

        # 获取可用串口信息
        self.refresh_serial_ports()

        # 设置波特率列表
        baudrate_list = ['9600', '19200', '38400', '57600', "74880", '115200', '921600', '1500000','3000000']
        for baudrate in baudrate_list:
            self.baudrate_combobox.addItem(baudrate)
        self.baudrate_combobox.setCurrentIndex(7)

        # 设置数据位列表
        databits_list = ['5', '6', '7', '8']
        for databits in databits_list:
            self.databits_combobox.addItem(databits)
        self.databits_combobox.setCurrentIndex(3)

        # 设置停止位列表
        stopbits_list = ['1', '2']
        for stopbits in stopbits_list:
            self.stopbits_combobox.addItem(stopbits)
        self.stopbits_combobox.setCurrentIndex(0)

        # 设置校验位列表
        parity_list = ['None', 'Odd', 'Even', 'Mark', 'Space']
        for parity in parity_list:
            self.parity_combobox.addItem(parity)
        self.parity_combobox.setCurrentIndex(0)

    def open_serial(self):
        # 打开串口
        # self.serial.setPortName(self.port_combobox.currentText())
        # self.serial.setBaudRate(int(self.baudrate_combobox.currentText()))
        # self.serial.setDataBits(int(self.databits_combobox.currentText()))
        # self.serial.setStopBits(int(self.stopbits_combobox.currentText()))
        # parity_dict = {'None': QSerialPort.NoParity, 'Odd': QSerialPort.OddParity,
        #             'Even': QSerialPort.EvenParity, 'Mark': QSerialPort.MarkParity,
        #             'Space': QSerialPort.SpaceParity}
        # self.serial.setParity(parity_dict[self.parity_combobox.currentText()])
        # if self.serial.open(QIODevice.ReadWrite):
        #     self.statusBar().showMessage(f"已打开串口 {self.serial.portName()}")
        #     self.open_button.setEnabled(False)
        #     self.close_button.setEnabled

        # print("port:",self.port_combobox.currentText(),"\n")
        parity_dict = {'None': serial.PARITY_NONE, 'Odd': serial.PARITY_ODD,
                    'Even': serial.PARITY_EVEN, 'Mark': serial.PARITY_MARK,
                    'Space': serial.PARITY_SPACE}
        # print("baudrate:",int(self.baudrate_combobox.currentText()),"\n")
        # print("databits:",int(self.databits_combobox.currentText()),"\n")
        # print("stopbits:",int(self.stopbits_combobox.currentText()),"\n")
        # print("parity_dict:",parity_dict[self.parity_combobox.currentText()],"\n")


        # self.serial = serial.Serial(port=port, baudrate=1500000, bytesize=serial.EIGHTBITS,
        #     parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1)

        self.serial = serial.Serial(port=self.port_combobox.currentText(), baudrate=int(self.baudrate_combobox.currentText()), bytesize=int(self.databits_combobox.currentText()),
            parity=parity_dict[self.parity_combobox.currentText()], stopbits=int(self.stopbits_combobox.currentText()), timeout=1)

        # 重新打开串口连接
        if self.serial.is_open:
            self.statusBar().showMessage(f"已打开串口 {self.serial.port}")
            self.open_button.setEnabled(False)
            self.close_button.setEnabled(True)

        self.start_read_thread()

    def close_serial(self):
        if self.serial.is_open:
            # 关闭串口
            self.serial.close()
            self.statusBar().showMessage(f"已关闭串口 {self.serial.portstr}")
            self.open_button.setEnabled(True)
            self.close_button.setEnabled(False)

    def get_platform(self):
        import sys
        return sys.platform


    def refresh_serial_ports(self):
            # 检测所有存在的串口，将信息存储在字典中
            com_dict = {}
            port_list = list(serial.tools.list_ports.comports())
            port_list = port_list[::-1]
            for port in port_list:
                if self.get_platform() == 'darwin':
                    if '/dev/cu.usbserial' not in port[0]:
                        continue
                com_dict["%s" % port[0]] = "%s" % port[1]
            if len(com_dict) == 0:
                self.state_label.setText(" 无串口")

            self.com_dict = com_dict

            self.port_combobox.clear()
            for item in self.com_dict.keys():
                self.port_combobox.addItem(item)

            # 清空串口下拉列表
            # self.port_combobox.clear()
            # 获取当前可用的串口列表
            # available_ports = serial.tools.list_ports.comports()

            # 遍历可用串口列表，添加到串口下拉列表中
            # port_list = QSerialPortInfo.availablePorts()
            # for port in port_list:
            #     self.port_combobox.addItem(port.portName())

    def handle_serial_error(self, error):
        # 处理串口错误
        error_dict = {
            QSerialPort.NoError: 'No error occurred.',
            QSerialPort.DeviceNotFoundError: 'An error occurred while attempting to open an non-existent device.',
            QSerialPort.PermissionError: 'An error occurred while attempting to open an already opened device.',
            QSerialPort.OpenError: 'An error occurred while attempting to open an already opened device.',
            QSerialPort.NotOpenError: 'This error occurs when an operation is executed that can only be successfully performed if the device is open.',
            QSerialPort.ParityError: 'This error occurs when the parity checking is enabled and a parity error is detected.',
            QSerialPort.FramingError: 'This error occurs when the framing checking is enabled and a framing error is detected.',
            QSerialPort.BreakConditionError: 'This error occurs when the device is waiting for a break condition to finish and a break is detected.',
            QSerialPort.WriteError: 'An error occurred while attempting to write data to the device.',
            QSerialPort.ReadError: 'An error occurred while attempting to read data from the device.',
            QSerialPort.TimeoutError: 'A timeout error occurred.'
        }
        error_message = error_dict[error]
        self.statusBar().showMessage(f"串口错误: {error_message}")
    def clear_receive_data(self):
        self.receive_text_text_browser.clear()

    def init_protocol(self):
        self.uartFrameHeaderTag = '5846'
        self.uartFrameHeaderLen = 6
        self.uartFrameDataLen = 5
        self.uartFrameLenByte = 11
        self.uartRecvList = list()
        self.image_counts = 0
        self.uartRecvHexStr = ''

        self.uart_frame_command_type_dict = {"设置CSK通讯串口波特率": 0x02, "查询软件版本": 0x19, "设置工作模式": 0x50, 
                                             "设置参数": 0x51, "下发合成文本": 0x53, "下发合成认证码": 0x54, 
                                             "下发手动校准信息":0x55, "下发校准绝对坐标": 0x56, "下发翻译文本": 0x57,
                                             "下发查词文本": 0x58,
                                             "下发播报内容": 0x59,
                                             "下发播报音频id": 0x5a,
                                             "下发流式播音控制": 0x5b,
                                             "设置录音启停": 0x5c,
                                             "停止播音": 0x5d,
                                             "下发内置音频控制": 0x5e,
                                             "下发文件传输控制": 0x5f,
                                             "下发通用指令控制": 0x60,
                                             "响应帧反馈": 0x61

        }

    def hex_string_to_list(self, x):
        return list(bytearray.fromhex(x))

    def int_to_list(self, data, bytes = 2, byteorder = 'little', signed = True):
        hex_str = data.to_bytes(bytes, byteorder=byteorder, signed=signed).hex()
        return self.hex_string_to_list(hex_str)
    
    def get_frame_lrc_check_result(self, frame_list, pos, len):
        # lrc = self.get_frame_lrc(frame_list)
        lrc_calc_result = 0
        for i in range(pos, pos + len):
            lrc_calc_result += frame_list[i]
            lrc_calc_result = lrc_calc_result % 256

        if (lrc_calc_result % 256) == 0:
            return True
        else:
            # logger.LOGE('lrc_calc_result: {}, frame_list:{}'.format(lrc_calc_result, frame_list[pos: pos + len]))
            frame_hex = ' '.join( list(map(lambda s: '{:02x}'.format(s), frame_list[pos: pos + len])))
            print('lrc_calc_result: {}, frame_list:\n{}'.format(lrc_calc_result, frame_hex) )

            return False
    def get_frame_type_check_result(self, frame_list, frame_type='响应帧', pos = 6):
        frame_type_dict = {'命令帧':0xf0, '响应帧': 0xff}
        # 是对应帧返回True
        if frame_list[pos] == frame_type_dict.get(frame_type):
            return True
        # 否则返回False 
        return False
    
    def get_frame_len(self, frame_list):
        pos = 2
        len = 2
        return self.get_hex_from_list(frame_list[pos:pos+len])
    
    def get_frame_lrc(self, frame_list, pos, len):
        lrc_calc_result = 0
        for i in range(pos, pos + len):
            lrc_calc_result += frame_list[i]
        # print(f'get_frame_lrc:{lrc_calc_result}')
        lrc_calc_result = (256 - (lrc_calc_result % 256) ) % 256
        return lrc_calc_result
    
    def list_to_hex_string_new(self, x):
        data  = ['{:02x}'.format(i) for i in x ]
        data = ' '.join(data)
        return data
    
    def list_to_hex_string(self, x):
        def ToBytes(data):
            if type(data) == type('12'):
                if len(data)%2 != 0:
                    data += '0'
                    print("add '0' at end,amended: ",end="")
                    print(data)
                return bytes().fromhex(data)
            elif type(data) == type([1,]):
                return bytes(data)
            else:
                print("only 'str' or 'list' is valid!")
                return None
        def ToHexStr(data):
            if type(data) == type([1,]):
                bytes_data = ToBytes(data)
                return bytes_data.hex()
            elif type(data) == type(b'\x00'):
                return data.hex()    
            else:
                print("only 'list' or 'bytes' is valid!")
                return None
        return ToHexStr(x)
    
    def get_hex_from_list(self, data, signed = False):
        lens = len(data)
        out_data = 0
        for i in range(lens):
            out_data += data[i] << (8 * i)

        if signed == True:
            out_data = self.int_form_signed(out_data, lens, signed)

        return out_data
    
    def int_form_signed(self, value, size, signed = True):
        return int.from_bytes(value.to_bytes(size, byteorder='little'), 'little', signed=True)

    def start_read_thread(self):
        # 启动读取串口数据的线程
        self.read_thread_running = True
        self.read_thread = ReadThread(self.serial)
        self.read_thread.data_received.connect(self.on_data_received)
        self.read_thread.start()

    
    def get_frame_res_work_status(self, frame_list):
        res_work_dict = {0x00: {'name': '待机状态', 'value': 0x00 }, \
                         0x01: {'name':'扫描状态', 'value': 0x01 } , \
                         0x02: {'name':'等待状态', 'value': 0x02 }, \
                         0x03: {'name':'拒绝状态', 'value': 0x03  }, \
                         0x04: {'name':'标定状态', 'value': 0x04 } , \
                         0x05: {'name':'系统初始化完成', 'value': 0x05 }, \
                         0x06: {'name':'摄像头初始化异常', 'value': 0x06  }, \
                         0x07: {'name':'摄像头解析异常', 'value': 0x07 } , \
                         0x08: {'name':'算法初始化异常', 'value': 0x08 }, \
                         0x09: {'name':'OCR文本输出中', 'value': 0x09  }, \
                         0x0A: {'name':'OCR文本输出结束', 'value': 0x0A  }, \
                         0x0B: {'name':'TTS合成输出中', 'value': 0x0B } , \
                         0x0C: {'name':'TTS合成输出结束', 'value': 0x0C }, \
                         0x0D: {'name':'翻译输出中', 'value': 0x0D }, \
                         0x0E: {'name':'翻译结束', 'value': 0x0E }, \
                         0x0F: {'name':'查词输出中', 'value': 0x0F }, \
                         0x10: {'name':'查词结束', 'value': 0x10 }, \
                         0x11: {'name':'音频播放中', 'value': 0x11 }, \
                         0x12: {'name':'音频播放结束', 'value': 0x12 }, \
                         0x13: {'name':'录音中', 'value': 0x13 }, \
                         0x14: {'name':'录音结束', 'value': 0x14 }, \
                         0x15: {'name':'内置音频上传开始', 'value': 0x15 }, \
                         0x16: {'name':'内置音频上传结束', 'value': 0x16 }, \
                         0x17: {'name':'分词结果输出开始', 'value': 0x17 }, \
                         0x18: {'name':'分词结果输出结束', 'value': 0x18 }, \
                         0xE0: {'name':'测试输出开始', 'value': 0xE0 }, \
                         0xE1: {'name':'测试输出结束', 'value': 0xE1 }, \
                         0xE2: {'name':'调试输出开始', 'value': 0xE2 }, \
                         0xE3: {'name':'调试输出结束', 'value': 0xE3 }, \
                         
                         0xF0: {'name':'拒绝状态', 'value': 0xF0 }, \
                         0xF1: {'name':'错误数据帧参数', 'value': 0xF1 }, \
                         0xF2: {'name':'错误模式状态', 'value': 0xF2 }
     
        }
        res_work_status = frame_list[self.uartFrameHeaderLen + 3 ]

        if res_work_status in res_work_dict.keys():
            return res_work_dict.get(res_work_status)
        else:
            print(f"res_work_status {res_work_status} is not found")
            return None

    def uart_data_hexstr_parse(self, dataInfo):
        import re

        uartFrameHeaderTag = self.uartFrameHeaderTag

        dataInfo = dataInfo.strip().replace(' ', '')
        dataInfo = self.uartRecvHexStr + dataInfo
        self.uartRecvHexStr = dataInfo

        print('dataInfo:', dataInfo)

        search_pos = 0

        result = None

        # 1.find Uart Header firstly
        while re.search(uartFrameHeaderTag, dataInfo[search_pos:]):
            
            (startPos, endPos) = re.search(uartFrameHeaderTag, dataInfo[search_pos:]).span()


            effectiveLen = int ( (len(dataInfo[startPos:])  ) / 2 )

            hex_list = self.hex_string_to_list(dataInfo[startPos: (startPos + 2 * effectiveLen )])


            # 帧长度最小值检查，不满足帧头长度直接缓冲
            if effectiveLen < self.uartFrameHeaderLen:
                self.uartRecvHexStr = dataInfo[startPos: (startPos + 2 * effectiveLen )]
                self.uartRecvList = self.hex_string_to_list(self.uartRecvHexStr)

                print('不满足帧头长度:', self.uartFrameHeaderLen, '帧:', self.uartRecvHexStr)
                return 

            # 帧头检查
            if not self.get_frame_lrc_check_result(hex_list, 0, self.uartFrameHeaderLen):
                search_pos += endPos 
                print('帧头校验失败')
                print( (startPos, endPos)  )

                print(dataInfo[search_pos:])
                
                self.uartRecvHexStr = ""
                self.uartRecvList = []
                continue
            else:
                # print( '帧头检测: ', (startPos, endPos) )
                pass
                # print( (startPos, endPos)  )

            search_pos += startPos + 2 * self.uartFrameHeaderLen

            # print('hex_list:', hex_list)

            # 获取帧长度
            uartFrameLenByte = self.get_frame_len(hex_list)
            # print(f'effectiveLen:[{effectiveLen}]uartFrameLenByte:[{uartFrameLenByte}]')

            self.uartFrameLenByte = uartFrameLenByte


            # 检查帧类型
            if len(hex_list) > self.uartFrameHeaderLen:
                # 不是响应帧，直接返回
                if not self.get_frame_type_check_result(hex_list):
                    self.uartRecvHexStr = ''
                    self.uartRecvList = list()
                    return
            
            # 1.luck mode
            if effectiveLen == self.uartFrameLenByte:
                self.uartRecvList = list()
                self.uartRecvHexStr = ''

                # 帧数据检查
                if not self.get_frame_lrc_check_result(hex_list, self.uartFrameHeaderLen, self.uartFrameLenByte - self.uartFrameHeaderLen):
                    print(f'{effectiveLen} == {self.uartFrameLenByte}, 帧数据校验失败')
                    return

                # print('完整帧:', dataInfo[startPos:])
                # print('完整帧')
                frame_list = self.hex_string_to_list(dataInfo[startPos: (startPos + 2 * effectiveLen)])
                result = self.print_frame_info(frame_list)

                return result
            
            # 2.pack mode TO-DO: pack Uart Frame
            elif effectiveLen < self.uartFrameLenByte:
                self.uartRecvList = list()
                
                self.uartRecvHexStr = dataInfo[startPos: (startPos + 2 * effectiveLen )]
                self.uartRecvList = self.hex_string_to_list(self.uartRecvHexStr)
                

            # 3.sticky mode TO-DO: unstick Uart Frame
            elif effectiveLen > self.uartFrameLenByte:
                # 帧数据检查
                if not self.get_frame_lrc_check_result(hex_list, self.uartFrameHeaderLen, self.uartFrameLenByte - self.uartFrameHeaderLen):
                    print(f'{effectiveLen} > {self.uartFrameLenByte}, 帧数据校验失败')
                    self.uartRecvHexStr = ''
                    self.uartRecvList = list()

                    uart_len = effectiveLen - self.uartFrameLenByte

                    # 保留剩余数据
                    self.uartRecvHexStr = dataInfo[ (2 * self.uartFrameLenByte) : (2* self.uartFrameLenByte + 2 * uart_len )]  
                    self.uartRecvList = self.hex_string_to_list(self.uartRecvHexStr)
                    # 有额外的串口数据，递归解析
                    if uart_len > self.uartFrameHeaderLen:
                        print(f'[√]递归查询, 剩余数据{uart_len}')
                        return self.uart_data_hexstr_parse('')
                    else:
                        print(f'[x]递归查询, 剩余数据{uart_len} <= {self.uartFrameHeaderLen}')
                        return result


                frame_list = self.hex_string_to_list(dataInfo[startPos: (startPos + 2 * effectiveLen)])
                result = self.print_frame_info(frame_list)
                
                self.uartRecvList = list()
                uart_len = effectiveLen - self.uartFrameLenByte
                self.uartRecvHexStr = dataInfo[ (2 * self.uartFrameLenByte) : (2* self.uartFrameLenByte + 2 * uart_len )]  
                self.uartRecvList = self.hex_string_to_list(self.uartRecvHexStr)

                print(self.uartRecvHexStr)

                # 有额外的串口数据，递归解析
                if uart_len > self.uartFrameHeaderLen:
                    print(f'[√]递归查询, 剩余数据{uart_len}')
                    return self.uart_data_hexstr_parse('')
                else:
                    print(f'[x]递归查询, 剩余数据{uart_len} <= {self.uartFrameHeaderLen}')
                    return result
                # return result

            return 
            
        # 2.pack Uart Frame
        print('该帧找不到header:', self.uartFrameHeaderTag)
        # self.uartRecvList = list()
        # self.uartRecvHexStr = ''
        # print(dataInfo)
        # dataInfo[search_pos:]
        return 
    
    def uart_response_feedback_send(self, command_code, data_type):
        if not self.serial.isOpen():
            print('串口未打开')
            return
        
        status_code = {'正确的命令': 0x00, '数据帧格式错误': 0x01, '数据帧参数错误': 0x02 }

        cmd_hex_str = ''
        cmd = [0x58, 0x46, 0x12, 0x00, 0x04, 0x53, 0xf0, 0x00, self.uart_frame_command_type_dict.get('响应帧反馈') ]
        
        frame_id = 0
        data_len = 2
        cmd.extend( self.int_to_list(command_code, bytes=1) )
        cmd.extend( self.int_to_list(frame_id, bytes=4) )
        
        cmd.extend( self.int_to_list(data_len, bytes=2) )
        cmd.extend( self.int_to_list(data_type, bytes=1) )
        cmd.extend( self.int_to_list(status_code.get('正确的命令'), bytes=1) )
        cmd.append(0)
        
        cmd[-1] = self.get_frame_lrc(cmd, 6, len(cmd) - 6)
        cmd[2], cmd[3] = self.int_to_list(len(cmd))
        cmd[5] = self.get_frame_lrc(cmd, 0, 5)

        cmd_hex_str = self.list_to_hex_string_new(cmd)

        # self.send_text_text_browser.setText(cmd_hex_str)
        # self.data_send()

        input_bytes = bytes(cmd)
        self.serial.write(input_bytes)


    def print_text(self, res_common_data, text_type = 'ocr'):
        import binascii, functools

        data_status_dict = {}
        reserved_data = res_common_data.get('reserved').get('data')
        data_status = reserved_data[0]
        text = ''
        if text_type in [ 'ocr', 'translate']:
            data_status_dict = {
                                0xf0: {'name':'无结果'},
                                0xf1: {'name':'有结果'},
                                0x00: {'name':'有结果'}
            }

            data_status_name = data_status_dict.get(data_status)
            text = ''

            if data_status not in data_status_dict.keys():
                print(f'暂不支持{text_type}=>{data_status}')
                return
            
            language_type = None
            language_type_name = None
            if data_status_name.get('name') not in ['无结果']:
                hex_list = res_common_data.get('data').get('data') 
                hex_str = self.list_to_hex_string(hex_list)
                hex_str = hex_str.replace(' ','')
                hex = hex_str.encode('utf-8')
                str_bin = binascii.unhexlify(hex)
                text = str_bin.decode('utf-8')

                if text_type in [ 'ocr']: 
                    language_type = reserved_data[1]

                    data_language_dict = {
                                    0xf0: {'name':'中文'},
                                    0xf1: {'name':'英文'},
                                    0xf2: {'name':'拼音'},
                                    0xf3: {'name':'音标'},
                                    0x00: {'name':''}
                    }
                    language_type_name = data_language_dict.get(language_type)

            
            if language_type_name != None:
                result_text = '{}:[{}][{}][{}]'.format( text_type, text, data_status_name.get('name'), language_type_name.get('name') )
            else:
                result_text = '{}:[{}][{}]'.format( text_type, text, data_status_name.get('name') )
        elif text_type in ['seg']:
            hex_list = res_common_data.get('data').get('data') 
            hex_str = self.list_to_hex_string(hex_list)
            hex_str = hex_str.replace(' ','')
            hex = hex_str.encode('utf-8')
            str_bin = binascii.unhexlify(hex)
            text = str_bin.decode('utf-8')

            result_text = '{}:[{}]'.format(text_type, text)
        elif text_type in ['dict']:
            hex_list = res_common_data.get('data').get('data') 
            hex_str = self.list_to_hex_string(hex_list)
            hex_str = hex_str.replace(' ','')
            hex = hex_str.encode('utf-8')
            str_bin = binascii.unhexlify(hex)
            text = str_bin.decode('utf-8')

            result_text = '{}:[{}]'.format(text_type, text)
        else:
            print(f'暂不支持{text_type}')
            return


        print(result_text)

        return text

    def get_frame_res_common_data(self, frame_list):
        def get_frame_data_member(frame_list, name, pos, len, signed = False):
            if name in ['reserved', 'data']:
                return frame_list[pos:pos+len]
            else:
                return self.get_hex_from_list(frame_list[pos:pos+len], signed)
        
        frame_command_bytes = 3
        common_data_type_bytes = 1
        common_data_id_bytes = 4
        common_data_x_start_bytes = 2
        common_data_x_end_bytes = 2
        common_data_y_start_bytes = 2
        common_data_y_end_bytes = 2
        common_data_len_bytes = 4
        common_data_reserved_bytes = 16



        res_common_data_dict = {       0x00: {'name':'校准图片jpg格式', 'value': 0x00 } , \
                                        0x01: {'name':'OCR文本', 'value': 0x01 }, \
                                        0x02: {'name':'离线切行图片raw', 'value': 0x02  }, \
                                        0x03: {'name':'离线拼接图片', 'value': 0x03 } , \
                                        0x04: {'name':'在线裁剪图片', 'value': 0x04 } , \
                                        0x05: {'name':'tts合成音频', 'value': 0x05 } , \
                                        0x06: {'name':'离线原始图片', 'value': 0x06 } , \
                                        0x07: {'name':'翻译结果', 'value': 0x07 } , \
                                        0x08: {'name':'查词结果', 'value': 0x08 } , \
                                        0x09: {'name':'录音音频', 'value': 0x09 } , \
                                        0x0A: {'name':'分词结果', 'value': 0x0A}
        }
        frame_res_common_data = {'type': { 'size': 1, 'pos': 0, 'data': 0 , 'data_info': res_common_data_dict}, \
                                'id':{ 'size': 4, 'pos': 0, 'data': 0 }, \
                                'x_start':{ 'size': 2, 'pos': 0, 'data': 0 , 'signed': True }, \
                                'x_end':{ 'size': 2, 'pos': 0, 'data': 0 , 'signed': True }, \
                                'y_start':{ 'size': 2, 'pos': 0, 'data': 0 , 'signed': True }, \
                                'y_end':{ 'size': 2, 'pos': 0, 'data': 0 , 'signed': True }, \
                                'data_len':{ 'size': 4, 'pos': 0, 'data': 0 }, \
                                'reserved':{ 'size': 16, 'pos': 0, 'data': [] }, \
                                'data':{ 'size': 0, 'pos': 0, 'data': [] }
        }
                            
        

        pos = self.uartFrameHeaderLen + frame_command_bytes
        for item in frame_res_common_data.keys():
            frame_res_common_data.get(item)['pos'] = pos
            if 'signed' in frame_res_common_data.get(item).keys():
                res_signed = frame_res_common_data.get(item).get('signed')
                frame_res_common_data.get(item)['data'] = get_frame_data_member(frame_list, item, \
                                                        frame_res_common_data.get(item)['pos'], \
                                                        frame_res_common_data.get(item)['size'], \
                                                        res_signed)
            else:
                frame_res_common_data.get(item)['data'] = get_frame_data_member(frame_list, item, \
                                                        frame_res_common_data.get(item)['pos'], \
                                                        frame_res_common_data.get(item)['size'])
            
            if item == 'data_len':
                frame_res_common_data.get('data')['size'] = frame_res_common_data.get(item)['data']

            pos += frame_res_common_data.get(item)['size']

        return frame_res_common_data

    
    def print_frame_info(self, data):
        print(f'frame_info:{data}')
        try:
            # Check for correct frame header
            if len(data) >= 11 and data[0] == 0x58 and data[1] == 0x46:
                frame_length = int.from_bytes(data[2:4], byteorder='little')
                # frame_length = frame_length - 6
                frame_data = data[6:frame_length]
                # Verify frame header checksum
                header_checksum = sum(data[0:6]) % 256
                if header_checksum != 0:
                    print("帧头校验失败: ", header_checksum)
                    return

                # Verify frame data checksum
                data_checksum = sum(frame_data) % 256
                if data_checksum != 0:
                    print("帧数据校验失败", data_checksum)
                    return

                # Parse frame data
                print("test")
                frame_type = frame_data[0]
                cmd = frame_data[2]

                if frame_type == 0xF0 and cmd == 0x01:  # 命令帧 0x01 命令帧反馈
                    feedback_data = frame_data[3]
                    print("命令帧反馈：{}".format(feedback_data))

                elif frame_type == 0xFF and cmd == 0x53:  # 响应帧 0x53 上传通用数据
                    data_type = frame_data[3]
                    data_size = int.from_bytes(frame_data[13:15], byteorder='little')
                    data_payload = frame_data[15:15+data_size]

                    if data_type == 0x01:  # OCR文本
                        # text = data_payload.decode('utf-8')
                        # print("OCR TEXT", text)
                        res_common_data = self.get_frame_res_common_data(data) 
                        print(res_common_data)
                        print('OCR文本')
                        text = self.print_text(res_common_data, text_type='ocr')

                        # 在文本框中显示OCR文本
                        self.receive_text_text_browser.insertPlainText(text)
                        self.receive_text_text_browser.moveCursor(QTextCursor.End)
                        # 获取当前字体
                        font = self.receive_text_text_browser.font()
                        # 设置字体大小为 14
                        font.setPointSize(18)
                        self.receive_text_text_browser.setFont(font)

                    else:
                        print("未知数据类型：{}".format(data_type))
                elif frame_type == 0xFF and cmd == 0x50:
                    res_status = self.get_frame_res_work_status(data)
                    if not res_status:
                        return
                    ret = res_status.get('name')
                    print("0x50 反馈工作状态")
                    # 系统初始化完成
                    if res_status.get('value') == 0x05:
                        self.uart_response_feedback_send(0x50, res_status.get('value'))
                    pass
                elif frame_type == 0xFF and cmd == 0x01:  # 响应帧 0x01 命令帧反馈
                    feedback_data = frame_data[3]
                    print("命令帧反馈：{}".format(feedback_data))
                else:
                    print("未知帧类型或命令字：{}, {}".format(frame_type, cmd))
            else:
                print("收到的数据太短，无法解析, len:",len(data), ", ", data)
        except Exception as e:
            print('解析串口数据时发生异常:', e)

    def on_data_received(self, data):
        #将接收到的串口数据显示在文本框中
        print("on_data_received data:",data,"\n")
        self.uart_data_hexstr_parse(data)

        QApplication.processEvents()

    # def closeEvent(self, event):
        # if self.serial.is_open:
            # self.serial.close()

class ReadThread(QThread):
    data_received = pyqtSignal(str)
    def __init__(self, serial):
        super().__init__()
        self.serial = serial

    def run(self):
        while True:
            if not self.serial.isOpen():
                time.sleep(0.1)
                continue

            try:
                # 读取串口数据
                num = 0
                try:
                    num = self.serial.inWaiting()
                except Exception as err:
                    print(err)
                    return None
                if num >= 0:
                    try:
                        data = self.serial.read(num)
                    except Exception as err:
                        print(err)
        
                    num = len(data)
                    out_s = ''
                    for i in range(0, len(data)):
                        out_s = out_s + '{:02X}'.format(data[i]) + ' '
                    out_string = out_s.replace(' ', '')
                    if data:
                        print('recv:', data)
                        self.data_received.emit(out_string)
                else:
                    continue

            except Exception as e:
                print('读取串口数据时发生异常:', e)
                self.serial.close()
                time.sleep(0.01)
                self.serial.open()
                time.sleep(0.01)

    def stop(self):
        self._stop_event.set()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
