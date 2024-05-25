import time;

from dynamixel_sdk import *;
from motions.fundamental import *;
from devices.actuator import Actuator;

class Controller :
    __BAUDRATE = 1000000;
    __DEVICE_NAME = "/dev/ttyUSB0";

    def __is_MX(self, id) :
        return id <= 6;

    def __init__(self) :
        print("[CONTROLLER] Initializing...");
        self.port_handler = PortHandler(self.__DEVICE_NAME);
        self.packet_handler_1 = PacketHandler(Actuator.model.MX.protocol_version);
        self.packet_handler_2 = PacketHandler(Actuator.model.XM.protocol_version);

        if self.port_handler.openPort() :
            print("[CONTROLLER] Succeeded to open the port");
        else :
            raise Exception("[CONTROLLER] Failed to open the port");

        if self.port_handler.setBaudRate(self.__BAUDRATE) :
            print("[CONTROLLER] Succeeded to set the baudrate");
        else :
            raise Exception("[CONTROLLER] Failed to set the baudrate");
        
        for i in Actuator.index :
            self.set_speed(i, 10);

        initial_position(self);

        time.sleep(5);

        for i in Actuator.index :
            self.set_speed(i, 100);
            self.set_acceleration(i, 20);

        print(f"[CONTROLLER] Actuator speed set to {Actuator.speed}");

    def __del__(self) :
        self.port_handler.closePort();
        print("[CONTROLLER] Succeeded to close the port");
    
    def set_mode(self, id, mode) :
        self.packet_handler_2.write1ByteTxRx(self.port_handler, id, Actuator.model.XM.operating_mode, mode);
    
    def get_mode(self, id) :
        print(self.packet_handler_2.read1ByteTxRx(self.port_handler, id, Actuator.model.XM.operating_mode));
    

    def set_speed(self, id, speed) :
        if self.__is_MX(id) :
            self.packet_handler_1.write2ByteTxRx(self.port_handler, id, Actuator.model.MX.moving_speed, speed);
        else :
            self.packet_handler_2.write4ByteTxRx(self.port_handler, id, Actuator.model.XM.profile_velocity, speed);
    
    def set_all_speed(self, speed) :
        for i in Actuator.index :
            self.set_speed(i, speed);

    def set_acceleration(self, id, value) :
        if not self.__is_MX(id) :
            self.packet_handler_2.write4ByteTxRx(self.port_handler, id, Actuator.model.XM.profile_acceleration, value);

    def set_torque(self, id, status) :
        if self.__is_MX(id) :
            self.packet_handler_1.write1ByteTxRx(self.port_handler, id, Actuator.model.MX.enable_torque, status);
        else :
            self.packet_handler_2.write1ByteTxRx(self.port_handler, id, Actuator.model.XM.torque_enable, status);

    def set_position(self, id, position) :
        position = position / 360 * 4096 if id % 2 == 1 else 4096 - ( position / 360 * 4096 );
        if self.__is_MX(id) :
            self.packet_handler_1.write2ByteTxRx(self.port_handler, id, Actuator.model.MX.goal_position, int(position));
        else :
            self.packet_handler_2.write4ByteTxRx(self.port_handler, id, Actuator.model.XM.goal_position, int(position));

    def set_raw_position(self, id, position) :
        if self.__is_MX(id) :
            self.packet_handler_1.write2ByteTxRx(self.port_handler, id, Actuator.model.MX.goal_position, int(position));
        else :
            self.packet_handler_2.write4ByteTxRx(self.port_handler, id, Actuator.model.XM.goal_position, int(position));

    def get_position(self, id) :
        if self.__is_MX(id) :
            result, error, _ = self.packet_handler_1.read2ByteTxRx(self.port_handler, id, Actuator.model.MX.present_position);
        else :
            result, error, _ = self.packet_handler_2.read4ByteTxRx(self.port_handler, id, Actuator.model.XM.present_position);
        print(f"[CONTROLLER] ID : {id} \t Current Position: {result}");
        return result;
