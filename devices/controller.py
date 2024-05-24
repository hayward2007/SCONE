from dynamixel_sdk import *;
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
            print(self.__is_MX(i));
            self.set_speed(i, Actuator.speed);
            self.set_torque(i, 1);
            self.set_position(i, Actuator.position.center);

    def __del__(self) :
        self.port_handler.closePort();
        print("[CONTROLLER] Succeeded to close the port");
    
    def set_mode(self, id, mode) :
        self.packet_handler_2.write1ByteTxRx(self.port_handler, id, Actuator.model.XM.operating_mode, mode);

    def set_speed(self, id, speed) :
        if self.__is_MX(id) :
            self.packet_handler_1.write2ByteTxRx(self.port_handler, id, Actuator.model.MX.moving_speed, speed);
        else :
            # moving speed on position control
            self.packet_handler_2.write4ByteTxRx(self.port_handler, id, Actuator.model.XM.profile_velocity, speed);
        
    def set_torque(self, id, status) :
        if self.__is_MX(id) :
            self.packet_handler_1.write1ByteTxRx(self.port_handler, id, Actuator.model.MX.enable_torque, status);
        else :
            self.packet_handler_2.write1ByteTxRx(self.port_handler, id, Actuator.model.XM.torque_enable, status);

    def set_position(self, id, position) :
        if self.__is_MX(id) :
            self.packet_handler_1.write2ByteTxRx(self.port_handler, id, Actuator.model.MX.goal_position, position);
        else :
            self.packet_handler_2.write4ByteTxRx(self.port_handler, id, Actuator.model.XM.goal_position, position);

    # def get_position(self, id) :
    #     result, data, error = self.packet_handler.read2ByteTxRx(self.port_handler, id, 36);
    #     print(f"[CONTROLLER] ID : {id} \t Current Position: {result}");
    #     return result;
