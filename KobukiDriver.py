import serial
import time
import threading
import logging
import traceback
from enum import IntEnum
from utils import *


class CmdName(IntEnum):
    BaseControl = 0x01
    Sound = 0x03
    SoundSequence = 0x04
    RequestExtra = 0x09
    GeneralPurposeOutput = 0x0c
    SetController = 0x0d
    GetController = 0x0e
    
    
class FeedbackName(IntEnum):
    BasicSensorData = 0x01
    DockingIR = 0x03
    InertialSensorData = 0x04
    CliffSensorData = 0x05
    Current = 0x06
    HardwareVersion = 0x0a
    FirmwareVersion = 0x0b
    RawGyro = 0x0d
    GeneralPurposeInput = 0x10
    UUID = 0x13
    ControllerInfo = 0x15


class KobukiDriver:
    def __init__(self, parameters):
        self.port = parameters['port']
        
        logging.basicConfig(format="%(asctime)s: %(message)s", level=logging.INFO,
                            datefmt="%H:%M:%S")

        self.ser = None
        
        self.shutdown_req = False
        self.is_enabled = False
        self.is_connected = False
        self.is_alive = False
        
        self.battery_capacity = parameters['battery_capacity']
        self.battery_low = parameters['battery_low']
        self.battery_dangerous = parameters['battery_dangerous']
        self.linear_velocity_max = parameters['linear_velocity_max']
        self.angular_velocity_max = parameters['angular_velocity_max']
        
        self.battery_warning_flag = False       
        self.battery_critical_flag = False

        self.heading_offset = None
        
        self.onreceive = {i: None for i in FeedbackName}
        self.connect()
        self.cmd_mutex = threading.Lock()
        self.connection = threading.Thread(target=KobukiDriver.spin, args=(self,))
        self.connection.start()
        
        self.get_version_info()
        self.get_controller_gain()
    
    
    def destroy(self):
        self.disable()
        self.shutdown_req = True
        self.connection.join()
        logging.info("Device: kobuki driver terminated.")
    
    def enable(self):
        self.is_enabled = True
        return True
    
    def disable(self):
        self.set_base_control(0, 0)
        self.is_enabled = False
        return True
    
    def connect(self):
        try:
            # Baud rate: 115200 BPS, Data bit: 8 bit, Stop bit: 1 bit, No Parity
            self.ser = serial.Serial(self.port, 115200, timeout=0.1)
            self.is_connected = True
            self.is_alive = True
            logging.info("Device is connected.")
            time.sleep(1)
        except serial.serialutil.SerialException:
            logging.critical(f"could not open port {self.port}, is the usb connected?")
            self.is_connected = False
            self.is_alive = False
    
    def spin(self):
        last_signal_time = time.perf_counter()
        buffer = bytearray(b'')
        # check connection
        while not self.shutdown_req:
            if self.ser is None or not self.ser.is_open:
                self.connect()
                if not self.is_connected:
                    logging.critical("device failed to open, waiting... ")
                    time.sleep(5)
                    continue
                    
            # read incoming
            
            # waste = self.ser.read(256) # header of the whole frame
            # logging.debug(f"Length of waste: {len(waste)}")
            # package_len = int.from_bytes(self.ser.read(1), byteorder='little') # total length of the frame
            # logging.debug(f"Serial read length: {package_len}")
            packages = self.ser.read(1024)
            # print(binascii.hexlify(packages[:100]))
            if len(packages) > 0:
                buffer += bytearray(packages)
                # logging.debug(f"Serial read: {packages.hex()}")
                payloads = buffer.split(b'\xaa\x55')
                for payload in payloads[:-1]:
                    if len(payload) > 0:
                        logging.debug(f"Payload: {payload.hex()}")
                        recv_length = payload[0]
                        real_length = len(payload) - 2
                        if recv_length != real_length:
                            logging.warning(f"Payload length unmatch: {recv_length} vs {real_length}")
                            continue
                        if not KobukiDriver.check_sum(payload):
                            logging.warning(f"Check sum failed: 0x{payload[-1]:x} vs 0x{KobukiDriver.cal_sum(payload[:-1]):x}")
                            continue
                            
                        self.parse_feedback(payload[1: -1]) # remove len and sum byte
                    
                    
                # keep last payload
                buffer = bytearray(payloads[-1])
                self.is_alive = True
            else:
                self.is_alive = False
                
        if self.ser.is_open:
            self.ser.close()
            
        logging.debug("Driver worker thread shutdown")

        
    def set_callback(self, frame_id, func):
        # func takes raw payload as input
        self.onreceive[frame_id] = func
    
    def parse_feedback(self, payload):
        i = 0
        l = len(payload)
        while i < l:
            frame_id = payload[i]
            data_len = payload[i + 1]
            frame_len = data_len + 2
            idx = i + 2
            
            if frame_id in self.onreceive:
                frame = payload[i: i + frame_len]
                if self.onreceive[frame_id] is not None:
                    self.onreceive[frame_id](frame)
                if frame_id == FeedbackName.BasicSensorData:
                    res = self.parse_basic_sensor_data(frame)
                elif frame_id == FeedbackName.DockingIR:
                    res = self.parse_docking_ir(frame)
                elif frame_id == FeedbackName.InertialSensorData:
                    res = self.parse_inertial_sensor(frame)
                elif frame_id == FeedbackName.CliffSensorData:
                    res = self.parse_cliff_sensor(frame)
                elif frame_id == FeedbackName.Current:
                    res = self.parse_current(frame)
                elif frame_id == FeedbackName.RawGyro:
                    res = self.parse_raw_gyro(frame)
                elif frame_id == FeedbackName.GeneralPurposeInput:
                    res = self.parse_general_purpose_input(frame)
                # ON request:
                elif frame_id == FeedbackName.UUID:
                    logging.info(f"UUID: {payload[idx: idx + 4].hex()}-{payload[idx + 4: idx + 8].hex()}-{payload[idx + 8: idx + 12].hex()}")
                elif frame_id == FeedbackName.HardwareVersion:
                    logging.info(f"HardwareVersion: {payload[idx + 2]}.{payload[idx + 1]}.{payload[idx]}")
                elif frame_id == FeedbackName.FirmwareVersion:
                    logging.info(f"FirmwareVersion: {payload[idx + 2]}.{payload[idx + 1]}.{payload[idx]}")
                elif frame_id == FeedbackName.ControllerInfo:
                    logging.info(f"Type: {'user' if payload[idx] else 'default'} " +  
                                    f"P: {lsb(payload[idx + 1: idx + 5]) / 1000.0} " + 
                                    f"I: {lsb(payload[idx + 5: idx + 9]) / 1000.0} " + 
                                    f"D: {lsb(payload[idx + 9: idx + 13]) / 1000.0}")
            else:
                logging.warning(f'Unknown feedback name: 0x{frame_id:x}')
                
            i += frame_len
            
            
    def set_base_control(self, linear_velocity, angular_velocity):
    
        def clip(v, limit):
            if v >= 0:
                return min(v, limit)
            else:
                return max(v, -limit)
    
        linear_velocity = clip(linear_velocity, self.linear_velocity_max)
        angular_velocity = clip(angular_velocity, self.angular_velocity_max)
        self.send_cmd([CmdName.BaseControl, 0x04] + signed2lsb(linear_velocity) + signed2lsb(angular_velocity))
        
    def send_cmd(self, payload):
        self.send_payload(payload)
        
    def send_payload(self, payload):
        if not self.is_alive or not self.is_connected:
            logging.debug("Device state is not ready yet.")
            if not self.is_alive:
                logging.debug(" - Device is not alive.")
            if not self.is_connected:
                logging.debug(" - Device is not connected.")
            return
        
        self.cmd_mutex.acquire()
        
        assert isinstance(payload, list)
        assert isinstance(payload[0], int)

        data = [0xAA, 0x55, len(payload)] + payload
        data.append(KobukiDriver.cal_sum(data[2:]))
        values = bytearray(data)
        self.ser.write(values)
        # self.ser.flush()
        logging.debug(f"Serial send: {values.hex()}")
        
        self.cmd_mutex.release()


    def get_version_info(self, mask=0x0a):
        """
            mask: 0x01 hardware version
                  0x02 firmware version
                  0x08 UUID
        """
        self.send_cmd([CmdName.RequestExtra, 0x02, mask, 0])

    def get_controller_gain(self):
        self.send_cmd([CmdName.RequestExtra, 0x01, 0])
    
    def parse_basic_sensor_data(self, frame):
        res = {}
        idx = 2
        res['timestamp'] = lsb(frame[idx: idx + 2])
        idx += 2
        res['bumper'] = frame[idx]
        res['bumper_left'] = frame[idx] & 0x04
        res['bumper_central'] = frame[idx] & 0x02
        res['bumper_right'] = frame[idx] & 0x01
        idx += 1
        res['wheel_drop'] = frame[idx]
        res['wheel_drop_left'] = frame[idx] & 0x02
        res['wheel_drop_right'] = frame[idx] & 0x01
        idx += 1
        res['cliff'] = frame[idx]
        res['cliff_left'] = frame[idx] & 0x04
        res['cliff_central'] = frame[idx] & 0x02
        res['cliff_right'] = frame[idx] & 0x01
        idx += 1
        res['left_encoder'] = lsb(frame[idx: idx + 2]) # 0->65535
        idx += 2
        res['right_encoder'] = lsb(frame[idx: idx + 2])
        idx += 2
        res['left_pwm'] = lsb2signed(frame[idx])
        idx += 1
        res['right_pwd'] = lsb2signed(frame[idx])
        idx += 1
        res['button'] = frame[idx]
        res['button0'] = frame[idx] & 0x01
        res['button1'] = frame[idx] & 0x02
        res['button2'] = frame[idx] & 0x04
        idx += 1
        res['charger'] = frame[idx]
        res['charger_des'] = charger_state[frame[idx]]
        idx += 1
        volt = frame[idx] * 0.1
        res['battery'] = volt
        volt_des = 'NORMAL'
        if volt < self.battery_dangerous:
            volt_des = 'DANGEROUS'
            if not self.battery_critical_flag:
                logging.critical("Battery DANGEROUS!")
                self.battery_critical_flag = True
                self.battery_warning_flag = True
        elif volt < self.battery_low:
            volt_des = 'LOW'
            if not self.battery_warning_flag:
                logging.warning("Battery LOW!")
                self.battery_warning_flag = True
            
        elif volt >= self.battery_capacity:
            volt_des = 'FULL'
            self.battery_critical_flag = False
            self.battery_warning_flag = False
        else:
            self.battery_critical_flag = False
            self.battery_warning_flag = False
                    
        res['battery_des'] = volt_des
        idx += 1
        res['overcurrent'] = frame[idx]
        res['overcurrent_left'] = frame[idx] & 0x01
        res['overcurrent_right'] = frame[idx] & 0x02
        return res
    
    
    def parse_docking_ir(self, frame):
        def parse_byte(b):
            return {ir_bit_state[bit]: b & bit for bit in ir_bit_state}
        
        p_right = parse_byte(frame[2])
        p_central = parse_byte(frame[3])
        p_left = parse_byte(frame[4])
        
        res = {'IR_RIGHT' + k: p_right[k] for k in p_right}
        res.update({'IR_CENTRAL' + k: p_central[k] for k in p_central})
        res.update({'IR_LEFT' + k: p_left[k] for k in p_left})
        return res
    
    
    def parse_inertial_sensor(self, frame):
        res = {
            # deg
            'z_angle_raw': lsb2signed(frame[2:4]) / 100.0,  # deg
            'z_angle_rate': lsb2signed(frame[4:6]) / 100.0, # deg / s ? 
        }
        if self.heading_offset is None:
            self.heading_offset = res['z_angle_raw']
        
        def wrap_angle(v):
            while v < 0:
                v += 360
            if v > 180:
                v -= 360
            return v
        
        res['z_angle'] = wrap_angle(res['z_angle_raw'] - self.heading_offset)
        return res
    
    
    def parse_cliff_sensor(self, frame):
        """
        ADC output of each PSD
        Data range: 0 ~ 4095 (0 ~ 3.3V)
        Distance range: 2 ~ 15 cm
        Distance is not linear w.r.t. ADC output.
        """
        
        res = {
            'cliff_right': parse_voltage(frame[2:4]),
            'cliff_central': parse_voltage(frame[4:6]),
            'cliff_left': parse_voltage(frame[6:8])
        }
        return res
        
    
    def parse_current(self, frame):
        res = {
            'motor_current_left': lsb(frame[2:4]) * 10.0, # mA
            'motor_current_right': lsb(frame[4:6]) * 10.0,
        }
        return res
        
        
    def parse_raw_gyro(self, frame):
        """
        This part might be wrong
        """
        
        def parse_data(b):
            cvt = 0.00875
            return {
                'x': -cvt * lsb(b[2:4]),  # rotate 90 deg
                'y':  cvt * lsb(b[:2]),
                'z':  cvt * lsb(b[4:6])
            }
            
        res = {
            'frame_id': frame[2],
            'raw_data': [ parse_data(frame[i: i + 6]) for i in range(4, len(frame), 6)]
        }
        
        return res
    
    
    def parse_general_purpose_input(self, frame):
        res = {
            'DigitalInput0': frame[2] & 0x01,
            'DigitalInput1': frame[2] & 0x02,
            'DigitalInput2': frame[2] & 0x04,
            'DigitalInput3': frame[2] & 0x08,
            'ADC0': parse_voltage(frame[4:6]),
            'ADC1': parse_voltage(frame[6:8]),
            'ADC2': parse_voltage(frame[8:10]),
            'ADC3': parse_voltage(frame[10:12])
        }
        return res


    @staticmethod
    def check_sum(bts):
        # bts: a list of ints
        s = KobukiDriver.cal_sum(bts[:-1])
        return bts[-1] == s
        
    
    @staticmethod
    def cal_sum(bts):
        # bts: a list of ints
        s = 0
        for b in bts:
            s ^= b
        return s
        


if __name__ == '__main__':
    para = {
        'port': 'COM8',
        'battery_capacity': 16.5, # http://yujinrobot.github.io/kobuki/classkobuki_1_1Parameters.html#a0c7b04175141d702f520c4a963876143
        'battery_dangerous': 13.2,
        'battery_low': 14.0,
        'linear_velocity_max': 300,
        'angular_velocity_max': 300,
    }
    kd = KobukiDriver(para)
    while True:
        try:
            kd.set_base_control(350, 50)
            time.sleep(0.1)
        except KeyboardInterrupt:
            break
        except:
            traceback.print_exc()
    kd.destroy()
    
