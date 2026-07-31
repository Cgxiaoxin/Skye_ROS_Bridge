
# todo:使用该脚本请先上传modbus_tk package
import serial
import modbus_tk.defines as cst
from modbus_tk import modbus_tcp, modbus_rtu



class ModbusTCP:
    """
    线圈 - 可读可写布尔量
    离散输入 - 只读布尔量
    保持寄存器 - 可读可写寄存器(16位)
    输入寄存器 - 只读寄存器(16位)

    # supported modbus functions
    READ_COILS = 1
    READ_DISCRETE_INPUTS = 2
    READ_HOLDING_REGISTERS = 3
    READ_INPUT_REGISTERS = 4
    WRITE_SINGLE_COIL = 5
    WRITE_SINGLE_REGISTER = 6
    WRITE_MULTIPLE_COILS = 15
    WRITE_MULTIPLE_REGISTERS = 16
    """

    def __init__(self, ip='127.0.0.1', port=502, timeout=3.0, maxRetryTime=3):
        self.master = modbus_tcp.TcpMaster(ip, port, timeout)
        self.maxRetryTime = maxRetryTime

    def __execute(self, slave, function_code, starting_address, quantity_of_x=0, output_value=0, data_format="",
        expected_length=-1, write_starting_address_fc23=0, number_file=None, pdu="", returns_raw=False
    ):
        retryTime = 0
        while True:
            try:
                ret = self.master.execute(slave, function_code, starting_address, quantity_of_x, output_value, data_format,
            expected_length, write_starting_address_fc23, number_file, pdu, returns_raw)
                return ret
            except Exception as e:
                print(f"Modbus Exception {e}!!! Retry Time:{retryTime}")
            retryTime = retryTime + 1
            if retryTime > self.maxRetryTime: 
                assert("Modbus Retry Exception")


    def read_coils(self, st_addr=0, length=1, slave=1) -> tuple:
        """
        读取线圈
        @param st_addr:起始地址
        @param length:读取长度
        @param slave:从机ID
        @return:
        """
        return self.__execute(slave, cst.READ_COILS, st_addr, length)

    def read_discrete_inputs(self, st_addr=0, length=1, slave=1) -> tuple:
        """
        读取离散输入
        @param st_addr:起始地址
        @param length:读取长度
        @param slave:从机ID
        @return:
        """
        return self.__execute(slave, cst.READ_DISCRETE_INPUTS, st_addr, length)

    def read_holding_registers(self, st_addr=0, length=1, slave=1) -> tuple:
        """
        读取保持寄存器
        @param st_addr:起始地址
        @param length:读取长度
        @param slave:从机ID
        @return:
        """
        return self.__execute(slave, cst.READ_HOLDING_REGISTERS, st_addr, length)

    def read_input_registers(self, st_addr=0, length=1, slave=1) -> tuple:
        """
        读取输入寄存器
        @param st_addr:起始地址
        @param length:读取长度
        @param slave:从机ID
        @return:
        """
        return self.__execute(slave, cst.READ_INPUT_REGISTERS, st_addr, length)

    def write_single_coil(self, st_addr, output_value, slave=1):
        """
        写入单线圈
        @param st_addr:起始地址
        @param output_value:待写入的数据
        @param slave:从机ID
        @return:
        """
        return self.__execute(slave, cst.WRITE_SINGLE_COIL, st_addr, output_value=output_value)

    def write_multi_coils(self, st_addr, output_value, slave=1):
        """
        写入多线圈
        @param st_addr:起始地址
        @param output_value:待写入的数据
        @param slave:从机ID
        @return:
        """
        return self.__execute(slave, cst.WRITE_MULTIPLE_COILS, st_addr, output_value=output_value)

    def write_single_register(self, st_addr, output_value, slave=1):
        """
        写单一寄存器
        @param st_addr:起始地址
        @param output_value:待写入的数据
        @param slave:从机ID
        @return:
        """
        return self.__execute(slave, cst.WRITE_SINGLE_REGISTER, st_addr, output_value=output_value)

    def write_multi_registers(self, st_addr, output_value: list, slave=1):
        """
        写多寄存器
        @param st_addr:起始地址
        @param output_value:待写入的数据
        @param slave:从机ID
        @return:
        """
        return self.__execute(slave, cst.WRITE_MULTIPLE_REGISTERS, st_addr, output_value=output_value)


class ModbusRTU(ModbusTCP):
    def __init__(self, port, baudrate=9600, bytesize=8, parity='N', stopbits=1, xonxoff=0, timeout=1.0, maxRetryTime=3):
        """
        Modbus-RTU协议串口通信
        @param port: 串口
        @param baudrate: 波特率
        @param bytesize: 字节大小
        @param parity: 校验位
        @param stopbits: 停止位
        @param xonxoff: 读超时
        @param timeout: 写超时
        """
        self.master = modbus_rtu.RtuMaster(serial.Serial(port=port, baudrate=baudrate, bytesize=bytesize,
                                                             parity=parity, stopbits=stopbits, xonxoff=xonxoff))
        self.master.set_timeout(timeout)
        self.maxRetryTime = maxRetryTime


if __name__ == "__main__":
    t1 = ModbusTCP()
    t1.write_multi_registers(1, 300, [1])
    t1.write_single_register(1, 300, 12)