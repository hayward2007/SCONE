from dynamixel_sdk import *;
from actuator import Actuator;

class Controller :
    __BAUDRATE = 1000000;
    __DEVICE_NAME = "/dev/ttyUSB0";
    __PROTOCOL_VERSION = 2.0;

    def __init__(self) :
        print("[CONTROLLER] Initializing...");
        self.port_handler = PortHandler(self.__DEVICE_NAME);
        self.packet_handler = PacketHandler(self.__PROTOCOL_VERSION);

        if self.port_handler.openPort() :
            print("[CONTROLLER] Succeeded to open the port");
        else :
            raise Exception("[CONTROLLER] Failed to open the port");

        if self.port_handler.setBaudRate(self.__BAUDRATE) :
            print("[CONTROLLER] Succeeded to set the baudrate");
        else :
            raise Exception("[CONTROLLER] Failed to set the baudrate");

        for i in self.index :
            self.set_speed(i, self.speed);

    def __del__(self) :
        self.port_handler.closePort();

    def set_speed(self, id, speed) :
        self.packet_handler.write2ByteTxRx(self.port_handler, id, 32, speed);
        
    def set_torque(self, id, status) :
        self.packet_handler.write1ByteTxRx(self.port_handler, id, 24, status);

    def set_position(self, id, position) :
        self.packet_handler.write2ByteTxRx(self.port_handler, id, 30, position);

    def get_position(self, id) :
        result, data, error = self.packet_handler.read2ByteTxRx(self.port_handler, id, 36);
        print(f"[CONTROLLER] ID : {id} \t Current Position: {result}");
        return result;
