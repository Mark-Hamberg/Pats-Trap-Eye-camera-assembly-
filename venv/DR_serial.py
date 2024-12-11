import serial
import time

from DR_error import *

# =============================================================================================
# define serial constant

DR_FIVEBITES = 5
DR_SIXBITS = 6
DR_SEVENBITS = 7
DR_EIGHTBITS = 8

DR_PARITY_NONE = "N"
DR_PARITY_EVEN = "E"
DR_PARITY_ODD = "O"
DR_PARITY_MARK = "M"
DR_PARITY_SPACE = "S"

DR_STOPBITS_ONE = 1
DR_STOPBITS_ONE_POINT_FIVE = 1.5
DR_STOPBITS_TWO = 2

DR_SERIAL_DEF_INTER_TIMEOUT = 0.1

DR_SERIAL_CONN_LIST = dict()
DR_SERIAL_END_DATA = dict()


def serial_open(port=None, baudrate=115200, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE):
    """
    This function opens a serial communication port.

    :param port: str - E.g. "COM1", "COM2
    :param baudrate: int - Baud rate
    :param bytesize: int - Number of data bits
    :param parity: str - Parity checking
    :param stopbits: int - Number of stop bits
    :return: serial.Serial instance
    """
    try:
        # port
        if type(port) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : port")

        '''OMG
        if port!="COM1" or port!="COM2" or port!="COM3" or port!="COM4":
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : port")
        else:           
            if port=="COM1":
                port="/dev/ttyS0"
            if port=="COM2":
                port="/dev/ttyS1"
            if port=="COM3":
                port="/dev/ttyS2"
            if port=="COM4":
                port="/dev/ttyS3"
        '''
        if port=="COM1":
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : reserved port")

        if port=="COM" or port=="COM2" or port=="COM3" or port=="COM4":
            if port=="COM":         #COM = COM2
                port="/dev/ttyS1"
            if port=="COM2":
                port="/dev/ttyS1"
            if port=="COM3":
                port="/dev/ttyS2"
            if port=="COM4":
                port="/dev/ttyS3"
        elif port=="COM_USB" or port=="COM_USB0" or port=="COM_USB1" or port=="COM_USB2" or port=="COM_USB3":
            if port=="COM_USB":     #COM_USB = COM_USB0
                port="/dev/ttyUSB0"
            if port=="COM_USB0":
                port="/dev/ttyUSB0"
            if port=="COM_USB1":
                port="/dev/ttyUSB1"
            if port=="COM_USB2":
                port="/dev/ttyUSB2"
            if port=="COM_USB3":
                port="/dev/ttyUSB3"
        else:           
            port = port

        # baudrate
        if type(baudrate) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : baudrate")

        # bytesize
        if type(bytesize) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : bytesize")

        if bytesize not in (DR_FIVEBITES, DR_SIXBITS, DR_SEVENBITS, DR_EIGHTBITS):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : bytesize")

        # parity
        if type(parity) != str:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : parity")

        if parity not in (DR_PARITY_NONE, DR_PARITY_EVEN, DR_PARITY_ODD, DR_PARITY_MARK, DR_PARITY_SPACE):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : parity")

        # stopbits
        if type(stopbits) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : stopbits")

        if stopbits not in (DR_STOPBITS_ONE, DR_STOPBITS_ONE_POINT_FIVE, DR_STOPBITS_TWO):
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : stopbits")

        # try to open
        ser = serial.Serial(port, baudrate, bytesize, parity, stopbits)

        # reset buffer
        serial_reset_rx(ser)
        serial_reset_tx(ser)

        # ser.inter_byte_timeout = 0.01
        # ser.inter_byte_timeout = 1          # for debug...

        # connected
        DR_SERIAL_CONN_LIST[id(ser)] = ser

    except ValueError as e:
        print("XXXXXXXXXXXXXXXXXXXXXXXXX 111")
        raise DR_Error(DR_ERROR_VALUE, "serial_open() ValueError : ", e.args[0])

    except serial.SerialException as e:
        print("XXXXXXXXXXXXXXXXXXXXXXXXX 222")
        raise DR_Error(DR_ERROR_RUNTIME, "serial_open() serial.SerialException : ", e.args)

    return ser


def serial_close(ser):
    """
    This function closes a serial communication port.

    :param ser: serial.Serial - Serial instance
    :return: int - (0 -> Success, Negative value -> Error)
    """
    if type(ser) != serial.Serial:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'ser' is not serial object.")

    ser.close()

    # disconnected
    del DR_SERIAL_CONN_LIST[id(ser)]

    return 0


def serial_state(ser):
    """
    This function returns the status of a serial communication port.

    :param ser: serial.Serial - Serial instance
    :return: int - (1 -> Open, 0 -> Closed)
    """
    if type(ser) != serial.Serial:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'ser' is not serial object.")

    if True == ser.isOpen():
        return 1
    else:
        return 0


def serial_set_inter_byte_timeout(ser, timeout=None):
    """
    This function sets the timeout between the bytes (inter-byte) when reading and writing to the port.

    :param ser: serial.Serial - Serial instance
    :param timeout: float - Timeout between bytes during reading or writing (Continued processing of data that was processed before the timeout,
    None: inter-byte timeout not specified)
    :return: int - (0 -> Success, Negative value -> Error)
    """
    # ser
    if type(ser) != serial.Serial:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'ser' is not serial object.")

    ser.inter_byte_timeout = timeout

    return 0


def serial_end_data(ser, end_data="\r\n"):
    # ser
    if type(ser) != serial.Serial:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'ser' is not serial object.")

    # end_data
    if type(end_data) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : end_data")

    # set
    DR_SERIAL_END_DATA[str(id(ser))] = end_data

    return 0


def serial_write(ser, tx_data):
    """
    This function records the data (tx_data) to a serial port.

    :param ser: serial.Serial - Serial instance
    :param tx_data: byte - Data to be transmitted
    :return: int - (0 -> Success, Negative value -> Error)
    """
    try:
        # ser
        if type(ser) != serial.Serial:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'ser' is not serial object.")

        # data
        if type(tx_data) != bytes:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : tx_data")

        # check opened
        if ser.isOpen() != True:
            print("Serial port is not opened!!")
            return -1

        com_end_data = DR_SERIAL_END_DATA.get(str(id(ser)), "")
        ser.write(tx_data + bytes(com_end_data, 'ascii'))

    except serial.SerialException as e:
        print("serial_write() serial.SerialException : ", e.args)
        return -2

    return 0


def serial_read(ser, length=-1, timeout=-1):
    """
    This function reads the data from a serial port.

    :param ser: serial.Serial - Serial instance
    :param length: int - Number of bytes to read
    :param timeout: float - Read waiting time
    :return: int - (0 -> Success, Negative value -> Error)
    """
    rx_data = None

    try:
        # ser
        if type(ser) != serial.Serial:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'ser' is not serial object.")

        # length
        if type(length) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : length")

        if length != -1 and length < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : length")

        # timeout
        #if type(timeout) != int:
        if type(timeout) != int and type(timeout) != float:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : timeout")

        if timeout != -1 and timeout < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : timeout")

        # check opened
        if ser.isOpen() != True:
            print("Serial port is not opened!!")
            return -1, None

        # read
        if timeout == -1:
            ser.timeout = None

            # 무한 대기
            if length == -1:
                # length 미지정시, 데이터 수신이 되었을 때 수신된 데이터만 읽기
                ba_data = bytearray()
                recv_flag = False

                while True:
                    if ser.in_waiting != 0:
                        temp = ser.read(ser.in_waiting)
                        ba_data.extend(temp)
                        recv_flag = True

                        # print("_____temp : ", temp)
                        # print("_____ba_data : ", ba_data)
                    else:
                        if recv_flag == True:
                            break

                    if ser.inter_byte_timeout == None:
                        time.sleep(DR_SERIAL_DEF_INTER_TIMEOUT)
                    else:
                        time.sleep(ser.inter_byte_timeout + 0.01)

                rx_data = bytes(ba_data)
            else:
                # length 지정시, 지정한 데이터 크기만큼 읽기
                rx_data = ser.read(length)

        # 기 수신된 데이터만 읽기
        elif timeout == 0:
            ser.timeout = timeout

            if length == -1:
                # 현재 버퍼의 데이터만 읽기
                rx_data = ser.read(ser.in_waiting)
            else:
                # 현재 버퍼에 있는 데이터 중, length 크기만큼 읽음 (버퍼의 데이터 수가 적을 시, 버퍼의 데이터 수만큼만)
                rx_data = ser.read(length)

        # timeout 지정하여 읽기
        else:
            ser.timeout = timeout

            if length == -1:
                # length 미지정시, 데이터 수신이 되었을 때 수신된 데이터만 읽기
                ba_data = bytearray()
                recv_flag = False

                start_time = time.time()
                while time.time() - start_time < timeout:
                    if ser.in_waiting != 0:
                        temp = ser.read(ser.in_waiting)
                        ba_data.extend(temp)
                        recv_flag = True

                        # print("_____temp : ", temp)
                        # print("_____ba_data : ", ba_data)
                    else:
                        if recv_flag == True:
                            break

                    if ser.inter_byte_timeout == None:
                        time.sleep(DR_SERIAL_DEF_INTER_TIMEOUT)
                    else:
                        time.sleep(ser.inter_byte_timeout + 0.01)

                rx_data = bytes(ba_data)
            else:
                # length 지정시, 지정한 데이터 크기만큼 읽기
                rx_data = ser.read(length)

    except serial.SerialException as e:
        print("serial_write() serial.SerialException : ", e.args)
        return -2, None

    return len(rx_data), rx_data


def serial_get_output_bytes(ser):
    out_waiting = 0

    try:
        # ser
        if type(ser) != serial.Serial:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'ser' is not serial object.")

        # check opened
        if ser.isOpen() != True:
            print("Serial port is not opened!!")
            return -1
        else:
            out_waiting = ser.out_waiting

    except serial.SerialException as e:
        print("ser_get_output_bytes() serial.SerialException : ", e.args)
        return -2

    return out_waiting


def serial_get_input_bytes(ser):
    in_waiting = 0

    try:
        # ser
        if type(ser) != serial.Serial:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'ser' is not serial object.")

        # check opened
        if ser.isOpen() != True:
            print("Serial port is not opened!!")
            return -1
        else:
            in_waiting = ser.in_waiting

    except serial.SerialException as e:
        print("ser_get_input_bytes() serial.SerialException : ", e.args)
        return -2

    return in_waiting


def serial_flush(ser):
    try:
        # ser
        if type(ser) != serial.Serial:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'ser' is not serial object.")

        # check opened
        if ser.isOpen() != True:
            print("Serial port is not opened!!")
            return -1
        else:
            ser.reset_input_buffer()
            ser.reset_output_buffer()

    except serial.SerialException as e:
        print("ser_flush() serial.SerialException : ", e.args)
        return -2

    return 0


def serial_reset_tx(ser):
    try:
        # ser
        if type(ser) != serial.Serial:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'ser' is not serial object.")

        # check opened
        if ser.isOpen() != True:
            print("Serial port is not opened!!")
            return -1
        else:
            ser.reset_output_buffer()

    except serial.SerialException as e:
        print("serial_reset_tx() serial.SerialException : ", e.args)
        return -2

    return 0


def serial_reset_rx(ser):
    try:
        # ser
        if type(ser) != serial.Serial:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'ser' is not serial object.")

        # check opened
        if ser.isOpen() != True:
            print("Serial port is not opened!!")
            return -1
        else:
            ser.reset_output_buffer()

    except serial.SerialException as e:
        print("serial_reset_rx() serial.SerialException : ", e.args)
        return -2

    return 0


def clean_serial():
    print("         clean_serial() call")

    for ser_id in list(DR_SERIAL_CONN_LIST.keys()):
        ser = DR_SERIAL_CONN_LIST[ser_id]

        print("_____CLOSE SERIAL_PORT {0} : {1}".format(ser_id, ser))

        serial_close(ser)

    DR_SERIAL_END_DATA.clear()

    return None

def flange_serial_open(baudrate=115200, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE):
    """
    This function opens the flange serial communication port.

    :param baudrate: int - Baud rate
    :param bytesize: int - Number of data bits
    :param parity: str - Parity checking
    :param stopbits: int - Number of stop bits
    :return: serial.Serial instance
    """
    return None


def flange_serial_close():
    """
    This function closes the flange serial communication port.

    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0

def flange_serial_write(tx_data):
    """
    This function records the data (tx_data) to the flange serial port.

    :param tx_data: byte - Data to be transmitted
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0

def flange_serial_read():
    """
    This function reads the data from the flange serial port.

    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0