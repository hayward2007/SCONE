from dynamixel_sdk import *;
from motions.fundamental import *;
from devices.actuator import Actuator;

class Controller :
    __BAUDRATE = 1000000;
    __DEVICE_NAME = "/dev/ttyUSB0";

    def __is_MX(self, id: int) :
        return id <= 6;

    def __init__(self) :
        print("[CONTROLLER] Initializing...");
        self.port_handler = PortHandler(self.__DEVICE_NAME);
        self.packet_handler_1 = PacketHandler(Actuator.model.MX.address.protocol_version);
        self.packet_handler_2 = PacketHandler(Actuator.model.XM.address.protocol_version);

        if self.port_handler.openPort() :
            print("[CONTROLLER] Succeeded to open the port");
        else :
            raise Exception("[CONTROLLER] Failed to open the port");

        if self.port_handler.setBaudRate(self.__BAUDRATE) :
            print("[CONTROLLER] Succeeded to set the baudrate");
        else :
            raise Exception("[CONTROLLER] Failed to set the baudrate");
        
        start_position(self);

        for i in Actuator.index :
            self.set_torque(i, 0);
            self.set_mode(i, 3);
            self.set_torque(i, 1);

        for i in Actuator.index :
            self.set_speed(i, 100);
            self.set_acceleration(i, 20);

        print(f"[CONTROLLER] Actuator speed set to {Actuator.speed}");

    def __del__(self) :
        self.port_handler.closePort();
        print("[CONTROLLER] Succeeded to close the port");
    
    def set_mode(self, id: int, mode: int) :
        if id in Actuator.lower_index :
            self.set_torque(id, Actuator.torque.off);
            result = self.packet_handler_2.write1ByteTxRx(self.port_handler, id, Actuator.model.XM.address.operating_mode, mode);
            self.set_torque(id, Actuator.torque.on);
            print(f"[CONTROLLER] Actuator ID : {id} / Mode : {mode} / Result : {result}");
        else :
            print("[CONTROLLER] Error to set mode, invalid id");
    
    def set_all_mode(self, mode: int) :
        for i in Actuator.lower_index :
            self.set_mode(i, mode);
    
    def get_mode(self, id: int) :
        print(self.packet_handler_2.read1ByteTxRx(self.port_handler, id, Actuator.model.XM.address.operating_mode));
    

    def set_speed(self, id: int, speed: int) :
        if self.__is_MX(id) :
            self.packet_handler_1.write2ByteTxRx(self.port_handler, id, Actuator.model.address.MX.moving_speed, speed);
        else :
            self.packet_handler_2.write4ByteTxRx(self.port_handler, id, Actuator.model.XM.address.profile_velocity, speed);
    
    def set_all_speed(self, speed: int) :
        for i in Actuator.index :
            self.set_speed(i, speed);

    def set_acceleration(self, id: int, acceleration: int) :
        if not self.__is_MX(id) :
            self.packet_handler_2.write4ByteTxRx(self.port_handler, id, Actuator.model.XM.address.profile_acceleration, acceleration);

    def set_torque(self, id: int, status: int) :
        if self.__is_MX(id) :
            self.packet_handler_1.write1ByteTxRx(self.port_handler, id, Actuator.model.MX.address.enable_torque, status);
        else :
            self.packet_handler_2.write1ByteTxRx(self.port_handler, id, Actuator.model.XM.address.torque_enable, status);

    def set_position(self, id: int, position) :
        position = int(position / 360 * 4096 if id % 2 == 1 else 4096 - ( position / 360 * 4096 ));
        if self.__is_MX(id) :
            self.packet_handler_1.write2ByteTxRx(self.port_handler, id, Actuator.model.MX.address.goal_position, position);
        else :
            self.packet_handler_2.write4ByteTxRx(self.port_handler, id, Actuator.model.XM.address.goal_position, position);

    def set_raw_position(self, id: int, position: int) :
        if self.__is_MX(id) :
            self.packet_handler_1.write2ByteTxRx(self.port_handler, id, Actuator.model.MX.address.goal_position, position);
        else :
            self.packet_handler_2.write4ByteTxRx(self.port_handler, id, Actuator.model.XM.address.goal_position, position);

    def get_position(self, id: int) :
        if self.__is_MX(id) :
            result, error, _ = self.packet_handler_1.read2ByteTxRx(self.port_handler, id, Actuator.model.MX.address.present_position);
        else :
            result, error, _ = self.packet_handler_2.read4ByteTxRx(self.port_handler, id, Actuator.model.XM.address.present_position);
        print(f"[CONTROLLER] Actuator ID : {id} \t Current Position: {result}");
        return result;
