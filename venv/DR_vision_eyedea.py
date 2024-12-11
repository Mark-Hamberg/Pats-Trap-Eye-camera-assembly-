import socket
import select
import time


from DR_error import *

# =============================================================================================
# command


##--- OBJECT DETECTION ---------------------------------------------------------------------------------------
COMMAND_GET_FIND_OBJECT_INFO        = 12                  # 검출된 object의 정보를 읽어오는 command

COMMAND_GEOMETRY_DISTANCE           = 45
COMMAND_GEOMETRY_ANGLE              = 46
COMMAND_GEOMETRY_GET_MEET_POINT     = 77

COMMAND_GEOMETRY_GET_INSPECTION_DISTANCE            = 69
COMMAND_GEOMETRY_GET_INSPECTION_DISTANCE_TOL_RATE   = 71
COMMAND_GEOMETRY_GET_INSPECTION_ANGLE               = 74
COMMAND_GEOMETRY_GET_INSPECTION_ANGLE_TOL_RATE      = 76


##--- CAMERA CONTROL ------------------------------------------------------------------------------------------
COMMAND_CAMERA_CONFIG_SET           = 52
COMMAND_CAMERA_CONFIG_LOAD          = 54
COMMAND_CAMERA_CONFIG_SET_DEFAULT   = 55

SET_CAMERA_AUTO_EXPOSURE            = 701
SET_CAMERA_MANUAL_EXPOSURE          = 704
SET_CAMERA_MANUAL_EXPOSURE_EXPOSURE = 705
SET_CAMERA_MANUAL_EXPOSURE_GAIN     = 706
SET_CAMERA_LED_BRIGHTNESS           = 708
SET_CAMERA_LED_TIMER_ON             = 709



##--- HISTOGRAM INSPECTION ------------------------------------------------------------------------------------
VISION_CONFIG_USE_HISTOGRAM                  = 122

COMMAND_VISION_CONFIG_SET                    = 7
COMMAND_HISTOGRAM_GET_PIXEL_COUNT_INDEX      = 61 
COMMAND_HISTOGRAM_GET_PIXEL_COUNT_ID         = 62
COMMAND_HISTOGRAM_GET_INSPECTION_PIXEL_COUNT = 64
COMMAND_HISTOGRAM_GET_INSPECTION_PIXEL_COUNT_TOL_RATE = 66


# =============================================================================================
# protocol define

# packet length
DR_PACKET_HEADER_LEN_BASE = 15
DR_C2S_PACKET_LEN_BASE = 18
DR_S2C_PACKET_LEN_BASE = 19

# timeout
DR_VS_TIMEOUT = 3

# =============================================================================================
# global variable

# DR_vs_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
DR_vs_sock = None

# port
DR_VS_PORT = 4000

DR_VS_DEVELP_MODE = True

# =============================================================================================
##
# @brief      class for ERVS vision packet
# @details    ERVS와의 tcp 통신에 대한 정보를 관리하는 class
#
class vision_packet():
    ##
    # @brief      class for packet type (inner class)
    # @details    define packet type (3.2 기준 : enum 미지원에 대한
    #
    class packet_type():
        none = 0
        send = 1
        recv = 2

    ##
    # @brief      생성자
    # @details    vision packet() 정보를 초기화한다.
    # @return     없음
    # @exception  없음
    #
    def __init__(self):
        self._type = self.packet_type.none
        self._valid = False

        self._packet_data = None
        self._command = None
        self._scale = 1
        self._data_len = 0
        self._data = None

    ##
    # @brief      packet 정보의 유효화 여부를 리턴한다.
    # @return     True - valid
    #             False - invalid
    # @exception  없음
    #
    def is_valid(self):
        return self._valid

    ##
    # @brief      packet 구성 정보 전체를 리턴한다.
    # @return     packet data - bytes
    # @exception  없음
    #
    def get_packet_data(self):
        return bytes(self._packet_data)

    ##
    # @brief      packet 구성 정보 중 command를 리턴한다.
    # @return     command - byte
    # @exception  없음
    #
    def get_command(self):
        return self._command

    ##
    # @brief      packet 구성 정보 중 data length를 리턴한다.
    # @return     data length - byte
    # @exception  없음
    #
    def get_data_len(self):
        return self._data_len

    ##
    # @brief      packet 구성 정보 중 data(s)를 리턴한다.
    # @return     data - bytes
    # @exception  없음
    #
    def get_data(self):
        return self._data

    ##
    # @brief      packet 구성 정보를 전달 받아 packet을 구성하며, 유효성을 검증한다.
    # @details    - prefix --   - cmd -  - scale -  - data length ------  - data ----------   - suffix -----------------
    #             1 2 3 4 5 6   7        8 9 10 11  12   13    14   15    15+(1) ... 15+(n)   15+(n+1) ........ 15+(n)+3
    #             d o o s a n   command  scale      Len1 Len2  Len3 Len4  Data1  ... Data n   D        R        A
    # @return     없음
    # @exception  - DR_ERROR_TYPE : argument의 type 비정상
    #             - DR_ERROR_VALUE : argument의 value 비정상
    #
    def make_send_data(self, command, scale=1, data_len=0, data=None):
        # reset data
        self._type = self.packet_type.none
        self._valid = False
        self._packet_data = None

        # set data
        self._command = command
        self._scale = scale
        self._data_len = data_len
        self._data = data

        # command
        if type(command) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : command")

        # scale
        if type(scale) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : scale")

        if scale < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : scale")

        # data_len
        if type(data_len) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : data_len")

        if data_len < 0:
            raise DR_Error(DR_ERROR_VALUE, "Invalid value : data_len")

        # data
        if data != None:
            if type(data) != bytes:
                raise DR_Error(DR_ERROR_TYPE, "Invalid type : data")

        # make packet data

        # prefix
        self._packet_data = b"doosan"
        # command
        self._packet_data += (self._command).to_bytes(1, byteorder='big')
        # scale
        self._packet_data += (self._scale).to_bytes(4, byteorder='big')
        # data length
        self._packet_data +=  (self._data_len).to_bytes(4, byteorder='big')
        # data
        if self._data != None:
            self._packet_data += self._data
        # suffix
        self._packet_data += b"DRA"
        # set valid
        self._type = self.packet_type.send
        self._valid = True
        # print : send data
        if DR_VS_DEVELP_MODE == True:
            hex_packet_data = ''.join('{:02X}'.format(x) for x in self._packet_data)
            print("\n[SEND DATA] : {0}".format(hex_packet_data))

    ##
    # @brief      수신 packet을 parsing 한다. 또한 유효한 packet인지 검사한다.
    # @details    - prefix --   - cmd -  - scale -  - data length ------  - data ----------   - suffix -----------------
    #             1 2 3 4 5 6   7        8 9 10 11  12   13    14   15    15+(1) ... 15+(n)   15+(n+1) ........ 15+(n)+4
    #             e y e d e a   command  scale      Len1 Len2  Len3 Len4  Data1  ... Data n   E         R    V  S
    # @return     없음
    # @exception  - DR_ERROR_TYPE : argument의 type 비정상
    #             - DR_ERROR_VALUE : argument의 value 비정상
    #             - DR_ERROR_RUNTIME : 수신 packet이 유효하지 않음
    #
    def parse_recv_data(self, packet_data):
        # reset data
        self._type = self.packet_type.none
        self._valid = False
        self._packet_data = packet_data
        self._command = None
        self._scale = 1
        self._data_len = 0
        self._data = None

        # packet_data
        if type(packet_data) != bytes:
            #raise DR_Error(DR_ERROR_TYPE, "Invalid type : packet_data")
            return -100

        if packet_data == None:
            #raise DR_Error(DR_ERROR_VALUE, "Invalid value : packet_data")
            return -100

        # print : receive data
        if DR_VS_DEVELP_MODE == True:
            hex_str = ''.join('{:02X}'.format(x) for x in self._packet_data)
            print("[RECV DATA] : {0}".format(hex_str))

        # parse packet
        if len(self._packet_data) < DR_PACKET_HEADER_LEN_BASE:
            hex_str = ''.join('{:02X}'.format(x) for x in self._packet_data)
            print("[RECV DATA] : {0}".format(hex_str))

            self._valid = False
            #raise DR_Error(DR_ERROR_RUNTIME, "_packet_data < DR_PACKET_HEADER_LEN_BASE")
            return -100

        # scale
        self._scale = self._packet_data[7] << 24;
        self._scale += self._packet_data[8] << 16;
        self._scale += self._packet_data[9] << 8;
        self._scale += self._packet_data[10]

        if self._scale == 0:
            hex_str = ''.join('{:02X}'.format(x) for x in self._packet_data)
            print("[RECV DATA] : {0}".format(hex_str))

            self._valid = False
            #raise DR_Error(DR_ERROR_RUNTIME, "_scale = 0")
            return -100

        # data length
        self._data_len = self._packet_data[11] << 24;
        self._data_len += self._packet_data[12] << 16;
        self._data_len += self._packet_data[13] << 8;
        self._data_len += self._packet_data[14]

        # check length
        if self._data_len + DR_S2C_PACKET_LEN_BASE != len(self._packet_data):
            hex_str = ''.join('{:02X}'.format(x) for x in self._packet_data)
            print("[RECV DATA] : {0}".format(hex_str))

            self._valid = False
            #raise DR_Error(DR_ERROR_RUNTIME, "_data_len + DR_S2C_PACKET_LEN_BASE != len(_packet_data)")
            return -100

        # check prefix
        if self._packet_data.startswith(b"eyedea") != True:
            hex_str = ''.join('{:02X}'.format(x) for x in self._packet_data)
            print("[RECV DATA] : {0}".format(hex_str))

            self._valid = False
            #raise DR_Error(DR_ERROR_RUNTIME, "not started 'eyedea'")
            return -100

        # check suffix
        if self._packet_data.endswith(b"ERVS") != True:
            hex_str = ''.join('{:02X}'.format(x) for x in self._packet_data)
            print("[RECV DATA] : {0}".format(hex_str))

            self._valid = False
            #raise DR_Error(DR_ERROR_RUNTIME, "not ended 'ERVS'")
            return -100

        # command
        self._command = self._packet_data[6]

        # data
        self._data = self._packet_data[15: 15 + self._data_len]

        # set valid
        self._type = self.packet_type.recv
        self._valid = True

        if DR_VS_DEVELP_MODE == True:
            # print("[RECEIVED DATA] " + str(self._packet_data))
            print(" ---- command     : {0}".format(self._command))
            print(" ---- scale       : {0}".format(self._scale))
            print(" ---- data length : {0}".format(self._data_len))

            hex_str = ''.join('{:02X}'.format(x) for x in self._data)
            print(" ---- data        : {0}".format(hex_str))

        return 1


def svm_connect(ip, port=DR_VS_PORT):
    """
    This function establishes communication with the SVM.

    :param ip: Server IP address of vision module.
    :param port: Port number.
    :return: 0 Connection Success, -1 Connection Failure
    """
    return 0


def svm_disconnect():
    """
    This function terminates the connection to the SVM.

    """
    return None


def svm_set_job(job_id):
    """
    This function loads the Vision task corresponding to the input id into the SVM.

    :param job_id: int - Vision Task id (ex. 1000, 2000, …)
    :return: 0 Job Loading success, -1 Job Loading fail.
    """
    return 0


def svm_get_robot_pose (job_id):
    """
    The robot pose information(joint coordinate system) set in the vision task is loaded.
    Robot pose information is used as shoot_pose for vision task.

    :param job_id: int - Vision Task id (ex. 1000, 2000, …)
    :return: float[6] Robot joint coordinate information (posj type), -1 Failed.
    """
    return -1


def svm_get_object_info(job_id):
    """
    Performs the measurement command corresponding to the input vision task.
    The detailed information of the measurement command of the vision work should be entered
    in advance through the Workcell manager(WCM).

    :param job_id: int - Vision Task id (ex. 1000, 2000, …)
    :return: 1 Measurement success – One object was detected / measured successfully.
    0 Measurement failed – Failed to detect the corresponding vision work object. -1 Measurement failed – Communication error (timeout).
    """
    return -1


def svm_get_variable (tool_id, var_type):
    """
    If the object detection/measurement is successful(1) by executing svm_get_vision_info, the detection/measurement data is loaded. Enter the tool id and variable type for the data to be loaded.
    - Position tool: POSX_TYPE (Object location), VALUE_TYPE (Detection similarity)
    - Presence tool: INSP_TYPE (Presence inspection result), VALUE_TYPE (Pixel count)
    - Distance tool: INSP_TYPE (Distance inspection result), VALUE_TYPE (Distance measure)
    - Angle tool: INSP _TYPE (Angle inspection result), VALUE_TYPE (Angle measure)
    - Diameter tool: INSP _TYPE (Diameter inspection result), VALUE_TYPE (Diameter measure), POSX_TYPE (Circle center position)

    :param tool_id: int - Vision Tool id (ex. 1000, 1001, 1002, ...)
    :param var_type: POSX_TYPE: Vision measurement coordinate variable (posx)
                INSP_TYPE: Inspection result variable (int)
                VALEU_TYPE: Measurement result (int or float)
    :return: POSX_TYPE – Coordinate information variable, ex. Posx(x,y,z,rx,ry,rz)
            INSP_TYPE: Inspection result variable - int (Returns 1 if successful)
            VALEU_TYPE: Measurement result variable (int of float)
    """
    return -1


def svm_get_offset_pos (posx_robot_init, job_id, tool_id):
    """
    The robot task coordinate information reflecting the vision measurement result is loaded into the robot work coordinate input by the user.
*Procedure: Input posx_robot_init  Vision measurement Call svm_get_offset_pos  Changed robot work coordinates (posx_robot_offset) output

    :param posx_robot_init: Robot task coordinate information) (Input by direct teaching method.)
    :param job_id: Vision job id (ex. 1000, 2000, 3000, …)
    :param tool_id: Vision tool id (ex. 1000, 1001, 1002, …)
    :return: posx - Robot work coordinate information reflecting vision measurement result. -1 Failed – No measurement data or input variable error.
    """
    return -1


def svm_set_init_pos_data(Id_list, Pos_list):
    """
    Enter the initial id_list and posx_list information of the object to perform the vision guidance operation.

    :param Id_list: List(int) - Id list ([id, id, id, …])
    :param Pos_list: List(Posx) - Posx list (ex.[posx, posx, posx, …])
    :return:
    """
    return None


def svm_set_tp_popup(svm_flag):
    """
    Set whether (tp_popup) should be displayed when SVM error occurs.

    :param svm_flag: int 1(Activation), 0(Deactivation)
    """
    return None


def svm_get_object_info_func(id_index, nMaxObj=0, nPrev=0, filter=None):
    return None, None


def svm_get_distance(id_base, id_sub, info_dist=0,info_tol=0):
    return None, None


def svm_get_distance_inspection(id_base, id_sub):
    return None, None


def svm_get_angle(id_base, id_sub,info_angle=0,info_tol=0):
    return None, None


def svm_get_angle_inspection(id_base, id_sub):
    return None, None


def svm_get_intersection(id_base, id_sub):
    return None, None


def svm_set_camera_default():
    return 0


def svm_set_camera_load():
    """
    Load LED brightness , exp , gain and focus setting save d in the Job numbered job_id.

    :return: job id (ex - 1000, 2000, 3000)
    """
    return 0


def svm_set_led_timer(level, timer):
    return 0


def svm_set_camera_exp_auto():
    return 0


def svm_set_camera_exp_manual():
    return 0


def svm_set_camera_exp_val(value):
    """
    Set exposure value of the SVM.

    :param value: SVM exposure value (2,660,000 – 29,260,000)
    """
    return 0


def svm_set_camera_gain_val(value):
    """
    Set SVM gain value.

    :param value: SVM gain value (0-1600).
    """
    return 0


def svm_set_led_brightness(value):
    """
    Set SVM LED brightness value.

    :param value: int - LED brightness value (0-1000).
    """
    return 0


def svm_get_led_brightness():
    """
    Return the LED brightness value set in the SVM.

    :return: int - SVM brightness value (0-1000)
    """
    return 0


def svm_set_histogram_config(value):
    return 0


def svm_get_histogram_inspection(obj_id):
    return None, None


def svm_get_histogram_inspection_count(obj_id):
    return None


def svm_get_histogram_inspection_tol(obj_id):
    return None


def svm_get_histogram_measure(obj_id, order=1):
    return None


def svm_get_histogram_measure_idx(obj_idx1, obj_idx2):
    return None


def svm_get_inspection_info(obj_infos, idx=0, tol_score=0, tol_histo=0):
    return None, None


