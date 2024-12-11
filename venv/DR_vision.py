#-*- coding: utf-8 -*-

# ##
# @mainpage
# @file     DR_vision.py
# @brief    Define DRL vision functions
# @author   joonkil kim, sungjin lim
# @Last update date     2018-08-17
# @details
#
# history
#
# 2018-07-30 : joonkil kim
#            : first release
# 2018-08-13 : joonkil kim
#            : 에러처리 루틴 추가 - DR_ERROR
# 2018-08-17 : joonkil kim
#            : 코드 점검 - 1차 확인 요청


import socket
import select
import time


from DR_math import *
from DR_tcp_client import *
from DR_error import *


# =============================================================================================
# protocol define

# command
DR_VS_CUSTOM     = 0
DR_VS_COGNEX     = 1
DR_VS_SICK       = 2

# pos id
VS_POS1 = 1
VS_POS2 = 2
VS_POS3 = 3
VS_POS4 = 4
VS_POS5 = 5
VS_POS6 = 6
VS_POS7 = 7
VS_POS8 = 8
VS_POS9 = 9
VS_POS10= 10

# timeout
DR_VS_TIMEOUT   = 3
DR_VS_WAIT      = 0.1
DR_VS_WAIT_TRIG = 0.5


# =============================================================================================
# global variable
H_vs_sock   = None
telnet_sock = None
error_code  = "None"
vs_type     = 0

vis_init_pos1=[0,0,0,0,0,0]
vis_init_pos2=[0,0,0,0,0,0]
vis_init_pos3=[0,0,0,0,0,0]
vis_init_pos4=[0,0,0,0,0,0]
vis_init_pos5=[0,0,0,0,0,0]
vis_init_pos6=[0,0,0,0,0,0]
vis_init_pos7=[0,0,0,0,0,0]
vis_init_pos8=[0,0,0,0,0,0]
vis_init_pos9=[0,0,0,0,0,0]
vis_init_pos10=[0,0,0,0,0,0]

rob_init_pos1=[0,0,0,0,0,0]
rob_init_pos2=[0,0,0,0,0,0]
rob_init_pos3=[0,0,0,0,0,0]
rob_init_pos4=[0,0,0,0,0,0]
rob_init_pos5=[0,0,0,0,0,0]
rob_init_pos6=[0,0,0,0,0,0]
rob_init_pos7=[0,0,0,0,0,0]
rob_init_pos8=[0,0,0,0,0,0]
rob_init_pos9=[0,0,0,0,0,0]
rob_init_pos10=[0,0,0,0,0,0]


# =============================================================================================
# @brief      비전센서 타입 선택
# @details    로봇과 연결된 비전센서의 타입을 선택한다. 
# @param      vs_type - 비전센서 타입 선택(DR_VS_CUSTOM, DR_VS_COGNEX, DR_VS_SICK)
# @return     입력된 비전 센서 타입
# @exception  DR_ERROR_VALUE - 정의되지 않은 센서타입(vs_type) 입력
#
def vs_set_info(type):
    global vs_type
    vs_type=type

    if type!=0 and type!=1 and type!=2:
        raise DR_Error(DR_ERROR_VALUE, "Invalid input: type")

    return vs_type


# =============================================================================================
# @brief      비전센서 연결
# @param      IP Address (string), Port number (int)
# @return     0 : 연결성공,   -1 : 연결실패
# @exception  없음
#
def vs_connect(ip_addr, port_num=9999):
    global vs_type

    if vs_type == 0:
        if VS_Connect(ip_addr, port_num)==0:
            return 0
        else:
            return -1
    elif vs_type == 1:
        res=cognex_connect(ip_addr)
    elif vs_type == 2:
        res=sick_connect(ip_addr)

    if res==1:
        res=0

    return res


# =============================================================================================
# @brief      비전센서 해제
# @param      없음
# @return     0 : 해제성공,   -1 : 해제실패
# @exception  없음
#
def vs_disconnect():
    global vs_type

    if vs_type == 0:
        if VS_Disconnect()==0:
            return 0
        else:
            return -1
    elif vs_type == 1:
        res=cognex_disconnect()
    elif vs_type == 2:
        res=sick_disconnect()

    if res==1:
        res=0

    return res


# =============================================================================================
# @brief      센서내부 저장된 작업파일 불러오기
# @param      없음
# @return     job_list (list[string])
# @exception  없음
#
def vs_get_job_list():
    global vs_type

    if vs_type == 0:
        return -1
    elif vs_type == 1:
        res=cognex_get_job_list()
    elif vs_type == 2:
        res=sick_get_job_list()

    return res


# =============================================================================================
# @brief      현재 설정된 작업 정보 불러오기
# @param      없음
# @return     job_name (string)
# @exception  없음
#
def vs_get_job():
    global vs_type

    if vs_type == 0:
        return -1
    elif vs_type == 1:
        res=cognex_get_job()
    elif vs_type == 2:
        res=sick_get_job()

    return res


# =============================================================================================
# @brief      입력된 작업을 센서에 현재작업으로 설정
# @param      없음
# @return     설정성공 0, 설정실패 -1
# @exception  없음
#
def vs_set_job(job_name):
    global vs_type

    if vs_type == 0:
        return -1
    elif vs_type == 1:
        res=cognex_set_job(job_name)
    elif vs_type == 2:
        res=sick_set_job(job_name)

    if res==1:
        res=0

    return res


# =============================================================================================
# @brief      센서에 측정명령 전달, 측정결과 데이터 불러오기
# @details    센서 출력값 형식 : pos,x,y,z,angle,var1,var2,...
# @param      없음
# @return     posx, var_list / 측정실패시 -1, []
# @exception  없음
#
def vs_trigger():
    global vs_type

    if vs_type == 0:
        return -1,[]
    elif vs_type == 1:
        res=cognex_trigger()
    elif vs_type == 2:
        res=sick_trigger()

    return res


# =============================================================================================
# @brief      센서 재부팅
# @param      없음
# @return     없음
# @exception  없음
#
def vs_reset():
    global vs_type

    if vs_type == 0:
        return -1
    elif vs_type == 1:
        res=cognex_reset()
    elif vs_type == 2:
        res=sick_reset()

    return res


# =============================================================================================
# @brief      작업 대상물체에 대한 비전/로봇의 초기 좌표값 입력
# @details    
# @param      - vision_posx_init: 작업대상물체 비전측정 정보
#             - robot_posx_init: 작업대상물체 로봇작업좌표 정보
#             - vs_pos: 로봇작업 순서 (ID)
# @return     없음
# @exception  없음
#
def vs_set_init_pos(vis_posx, rob_posx, vs_pos=1):
    global vis_init_pos1
    global vis_init_pos2
    global vis_init_pos3
    global vis_init_pos4
    global vis_init_pos5
    global vis_init_pos6
    global vis_init_pos7
    global vis_init_pos8
    global vis_init_pos9
    global vis_init_pos10
    
    global rob_init_pos1
    global rob_init_pos2
    global rob_init_pos3
    global rob_init_pos4
    global rob_init_pos5
    global rob_init_pos6
    global rob_init_pos7
    global rob_init_pos8
    global rob_init_pos9
    global rob_init_pos10
    
    if vs_pos==1:
        vis_init_pos1=vis_posx
        rob_init_pos1=rob_posx
    elif vs_pos==2:
        vis_init_pos2=vis_posx
        rob_init_pos2=rob_posx
    elif vs_pos==3:
        vis_init_pos3=vis_posx
        rob_init_pos3=rob_posx
    elif vs_pos==4:
        vis_init_pos4=vis_posx
        rob_init_pos4=rob_posx
    elif vs_pos==5:
        vis_init_pos5=vis_posx
        rob_init_pos5=rob_posx
    elif vs_pos==6:
        vis_init_pos6=vis_posx
        rob_init_pos6=rob_posx
    elif vs_pos==7:
        vis_init_pos7=vis_posx
        rob_init_pos7=rob_posx
    elif vs_pos==8:
        vis_init_pos8=vis_posx
        rob_init_pos8=rob_posx
    elif vs_pos==9:
        vis_init_pos9=vis_posx
        rob_init_pos9=rob_posx
    elif vs_pos==10:
        vis_init_pos10=vis_posx
        rob_init_pos10=rob_posx
    else:
        vis_init_pos1=vis_posx
        rob_init_pos1=rob_posx
        
    return vs_pos


# =============================================================================================
# @brief      입력되어 잇는 작업 대상물체에 대한 비전/로봇의 초기 좌표값 출력
# @param      없음
# @return     - vision_posx_init: 작업대상물체 비전측정 정보
#             - robot_posx_init: 작업대상물체 로봇작업좌표 정보
#             - vs_pos: 로봇작업 순서 (ID)
# @exception  없음
#
def vs_get_init_pos(vs_pos=1):
    global vis_init_pos1
    global vis_init_pos2
    global vis_init_pos3
    global vis_init_pos4
    global vis_init_pos5
    global vis_init_pos6
    global vis_init_pos7
    global vis_init_pos8
    global vis_init_pos9
    global vis_init_pos10

    global rob_init_pos1
    global rob_init_pos2
    global rob_init_pos3
    global rob_init_pos4
    global rob_init_pos5
    global rob_init_pos6
    global rob_init_pos7
    global rob_init_pos8
    global rob_init_pos9
    global rob_init_pos10
    
    rob_posx=[0,0,0,0,0,0]
    vis_posx=[0,0,0,0,0,0]
    if vs_pos==1:
        vis_posx=vis_init_pos1
        rob_posx=rob_init_pos1
    elif vs_pos==2:
        vis_posx=vis_init_pos2
        rob_posx=rob_init_pos2
    elif vs_pos==3:
        vis_posx=vis_init_pos3
        rob_posx=rob_init_pos3
    elif vs_pos==4:
        vis_posx=vis_init_pos4
        rob_posx=rob_init_pos4
    elif vs_pos==5:
        vis_posx=vis_init_pos5
        rob_posx=rob_init_pos5
    elif vs_pos==6:
        vis_posx=vis_init_pos6
        rob_posx=rob_init_pos6
    elif vs_pos==7:
        vis_posx=vis_init_pos7
        rob_posx=rob_init_pos7
    elif vs_pos==8:
        vis_posx=vis_init_pos8
        rob_posx=rob_init_pos8
    elif vs_pos==9:
        vis_posx=vis_init_pos9
        rob_posx=rob_init_pos9
    elif vs_pos==10:
        vis_posx=vis_init_pos10
        rob_posx=rob_init_pos10
    else:
        vis_posx=vis_init_pos1
        rob_posx=rob_init_pos1
        
    return vis_posx, rob_posx



# =============================================================================================
# @brief      로봇 작업좌표정보 산출
# @details    비전센서에서 측정된 물체위치정보를 이용하여 변경된 로봇의 작업좌표 정보를 산출한다.
# @param      - posx_vision_meas: 측정된 물체 위치정보
#             - vs_pos: 변경할 로봇작업좌표의 ID
# @return     rob_pos_offset : 변경된 로봇의 작업좌표 정보
# @exception  없음
#
def vs_get_offset_pos(posx_vision_meas, vs_pos=1):
    #vis_pos_0  = posx_vision_init
    #rob_pos_0  = posx_robot_init

    vis_pos_0 ,rob_pos_0 = vs_get_init_pos(vs_pos)
    
    vis_to_rob_pos_0 = htrans(inverse_pose(vis_pos_0), rob_pos_0)
    
    vis_pos_1  = posx(posx_vision_meas[0], posx_vision_meas[1], posx_vision_meas[2], 0,0,0)
    
    rotang = posx_vision_meas[5]/57.295779513
    r_vec  = [0*rotang, 0*rotang, 1*rotang]    # get z-axis vector
    roteul = rotvec2eul(r_vec)
    
    rob_pos_0_offset=[0,0,0, roteul[0], roteul[1], roteul[2]]
    
    rob_pos_0_rot = htrans (rob_pos_0_offset, vis_to_rob_pos_0)
    
    rob_pos_offset = htrans (vis_pos_1, rob_pos_0_rot)
    
    return rob_pos_offset



# =============================================================================================
# @brief      코그넥스 비전 센서 연결
# @details    vs_connect 호출시 실행
# @param      ip address (string)
# @return     연결성공 1, 연결실패 -1
# @exception  DR_ERROR_VALUE : COGNEX SENSOR ERROR
#
def cognex_connect(ip_addr):
    global telnet_sock
    telnet_sock = client_socket_open(ip_addr, 23)   
    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)

    client_socket_write(telnet_sock, b"admin\r\n")  
    time.sleep(DR_VS_WAIT)
    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)  

    client_socket_write(telnet_sock, b"\r\n")      
    time.sleep(DR_VS_WAIT)
    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)

    client_socket_write(telnet_sock, b"GF\r\n")
    time.sleep(DR_VS_WAIT)
    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)
    
    if cognex_error_check(rx_data) !=1:
        raise DR_Error(DR_ERROR_VALUE, "Connect fail: {0}".format(vs_get_errorcode()))
        return -1
    else:        
        return cognex_set_online()



# =============================================================================================
# @brief      코그넥스 비전 센서 해제
# @details    vs_disconnect 호출시 실행
# @param      없음
# @return     해제성공 1, 해제실패 -1
# @exception  DR_ERROR_VALUE : COGNEX SENSOR ERROR
#
def cognex_disconnect():
    global telnet_sock
    res=client_socket_close(telnet_sock)   
    if res!=0:
        raise DR_Error(DR_ERROR_VALUE, "Disconnect fail")
        return -1
    
    return 1


# =============================================================================================
# @brief      비전센서 온라인으로 상태 변경
# @details    온라인 - 측정대기 상태, 트리거 입력시 측정을 실시한다.
# @param      없음
# @return     성공시 1, 실패시 -1
# @exception  DR_ERROR_VALUE : COGNEX SENSOR ERROR
#
def cognex_set_online():
    global telnet_sock
    client_socket_write(telnet_sock, b"so1\r\n")
    time.sleep(DR_VS_WAIT)
    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)
    
    if cognex_error_check(rx_data) !=1:
        raise DR_Error(DR_ERROR_VALUE, "Set online: {0}".format(vs_get_errorcode()))
        return -1
    else:
        return 1


# =============================================================================================
# @brief      비전센서 오프라인으로 상태 변경
# @details    오프라인 - 센서 설정 상태, 작업변경/저장/로딩 등의 센서 설정이 가능한 상태이다.
# @param      없음
# @return     성공시 1, 실패시 -1
# @exception  DR_ERROR_VALUE : COGNEX SENSOR ERROR
#
def cognex_set_offline():
    global telnet_sock
    client_socket_write(telnet_sock, b"so0\r\n")
    time.sleep(DR_VS_WAIT)
    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)
    
    if cognex_error_check(rx_data) !=1:
        raise DR_Error(DR_ERROR_VALUE, "Set offline: {0}".format(vs_get_errorcode()))
        return -1
    else:
        return 1


# =============================================================================================
# @brief      비전센서 재부팅
# @param      없음
# @return     성공시 1, 실패시 -1
# @exception  DR_ERROR_VALUE : COGNEX SENSOR ERROR
#
def cognex_reset():
    global telnet_sock
    client_socket_write(telnet_sock, b"RT\r\n")
    time.sleep(DR_VS_WAIT)

    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)
    
    if cognex_error_check(rx_data) ==-6:
        raise DR_Error(DR_ERROR_VALUE, "Reset system: {0}".format(vs_get_errorcode()))
        return -1
    else:
        return 1


# =============================================================================================
# @brief      저장된 작업파일 리스트 불러오기
# @details    비전작업파일(*.job)의 리스트를 얻어온다.
# @param      없음
# @return     성공시 job_name[list(string)] / 실패시 -1
# @exception  DR_ERROR_VALUE : COGNEX SENSOR ERROR
#
def cognex_get_job_list():   
    global telnet_sock
    client_socket_write(telnet_sock, b"get filelist\r\n") 
    time.sleep(DR_VS_WAIT)
    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)

    if cognex_error_check(rx_data) !=1:
        raise DR_Error(DR_ERROR_VALUE, "Get Fileist: {0}".format(vs_get_errorcode()))
        return -1

    rx_msg=rx_data.decode()
    
    data=rx_msg.split('\r\n')
    
    vs_data=[]
    for i in range(len(data)):
    	if data[i].count('.job') ==1:
    		vs_data.append(data[i])

    return vs_data

    
# =============================================================================================
# @brief      현재 설정된 작업 불러오기
# @param      없음
# @return     성공시 job_name(string), 실패시 -1
# @exception  DR_ERROR_VALUE : COGNEX SENSOR ERROR
#    
def cognex_get_job():   
    global telnet_sock
    client_socket_write(telnet_sock, b"GF\r\n")
    time.sleep(DR_VS_WAIT)

    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)

    if cognex_error_check(rx_data) !=1:
        raise DR_Error(DR_ERROR_VALUE, "Get File: {0}".format(vs_get_errorcode()))
        return -1
    
    rx_msg=rx_data.decode()
    data=rx_msg.split('\r\n')
    res=data[0]
    job=data[1]

    return job


# =============================================================================================
# @brief      입력된 작업을 비전센서의 설정작업으로 로딩함.
# @details    입력된 작업은 재부팅 후에도 초기작업으로 설정됨.
# @param      job_name(string) - 대상작업
# @return     성공시 1, 실패시 -1
# @exception  DR_ERROR_VALUE : COGNEX SENSOR ERROR
#    
def cognex_set_job(job_name):   
    global telnet_sock   

    job_list=cognex_get_job_list()
    list_check=-1

    for i in range(len(job_list)):
        if (job_list[i] == job_name):
            list_check=1
    
    if list_check==-1:
        raise DR_Error(DR_ERROR_VALUE, "No File: -1")
    
    if job_name.count('.job') ==1 and list_check==1:
        cognex_set_offline()
        client_socket_write(telnet_sock, b"LF")
        client_socket_write(telnet_sock, job_name.encode())
        client_socket_write(telnet_sock, b"\r\n")
        time.sleep(DR_VS_WAIT)

        
        res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)
        if cognex_error_check(rx_data) !=1:
            err_code=vs_get_errorcode()
            cognex_set_online()
            raise DR_Error(DR_ERROR_VALUE, "Load File: {0}".format(err_code))
            return -1

        client_socket_write(telnet_sock, b"EV SetStartup(\"")
        client_socket_write(telnet_sock, job_name.encode())
        client_socket_write(telnet_sock, b"\",1)")
        client_socket_write(telnet_sock, b"\r\n")
        time.sleep(DR_VS_WAIT)
        
        res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)
        if cognex_error_check(rx_data) !=1:
            err_code=vs_get_errorcode()
            cognex_set_online()
            raise DR_Error(DR_ERROR_VALUE, "Set Startup: {0}".format(err_code))
            return -1
        
        client_socket_write(telnet_sock, b"TS\r\n")
        time.sleep(DR_VS_WAIT)
        res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)
        if cognex_error_check(rx_data) !=1:
            err_code=vs_get_errorcode()
            cognex_set_online()            
            raise DR_Error(DR_ERROR_VALUE, "Store setting: {0}".format(err_code))
            return -1

        cognex_set_online()        
        res =1

    else:
        res=-1

    return res


# =============================================================================================
# @brief      비전센서 저장
# @details    현재 설정된 작업의 변경사항을 저장한다.
# @param      없음
# @return     성공시 1, 실패시 -1
# @exception  DR_ERROR_VALUE : COGNEX SENSOR ERROR
#    
def cognex_save_job():
    global telnet_sock
    client_socket_write(telnet_sock, b"GF\r\n")
    time.sleep(DR_VS_WAIT)

    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)    
    if cognex_error_check(rx_data) !=1:
        raise DR_Error(DR_ERROR_VALUE, "Get File: {0}".format(vs_get_errorcode()))
        return -1
    
    rx_msg=rx_data.decode()
    data=rx_msg.split('\r\n')
    res=data[0]
    job=data[1]
    
    client_socket_write(telnet_sock, b"TF")
    client_socket_write(telnet_sock, job.encode())
    client_socket_write(telnet_sock, b"\r\n")
    time.sleep(DR_VS_WAIT)
    
    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)
    if cognex_error_check(rx_data) !=1:
        raise DR_Error(DR_ERROR_VALUE, "Store File: {0}".format(vs_get_errorcode()))
        return -1
    else:
        return 1



# =============================================================================================
# @brief      센서에 측정명령 전달, 측정결과 데이터 불러오기
# @details    센서 출력값 형식 : pos,x,y,angle,var1,var2,...
# @param      없음
# @return     posx, var_list / 측정실패시 -1, []
# @exception  DR_ERROR_VALUE : COGNEX SENSOR ERROR
#
def cognex_trigger():
    global telnet_sock
    
    client_socket_write(telnet_sock, b"SW8\r\n")  
    time.sleep(DR_VS_WAIT_TRIG)

    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)
    if cognex_error_check(rx_data) !=1:
        raise DR_Error(DR_ERROR_VALUE, "Set Event&Wait: {0}".format(vs_get_errorcode()))
        return -1,[]
    
    
    client_socket_write(telnet_sock, b"GVJob.FormatString\r\n")     
    time.sleep(DR_VS_WAIT)

    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)  
    if cognex_error_check(rx_data) !=1:
        raise DR_Error(DR_ERROR_VALUE, "Get value: {0}".format(vs_get_errorcode()))
        return -1,[]
      
    rx_msg=rx_data.decode()
    data  = rx_msg.split('\r\n')
    data1 = data[1].split(',')

    pos=-1
    var_list=[]
    
    # data format = pos,xval,yval,tval,var1,var2,var3, ...
    # data format = pos,12.3,45.6,78.9,1,0,147.9, ...
    
    if len(data1) >= 4 and data1[0]=="pos":
        if isfloat(data1[1]) and isfloat(data1[2]) and isfloat(data1[3]):
            pos=[float(data1[1]), float(data1[2]), 0, 0, 0, float(data1[3])]

    for i in range(len(data1)):
        if i >=4:
            if isfloat(data1[i]):
                var_list.append(float(data1[i]))
                
    return pos, var_list



# =============================================================================================
# @brief      코그넥스 비전 센서의 오류발생을 확인
# @details    출력된 ERROR_CODE는 insight SW 매뉴얼의 Native mode command ERROR 테이블 참조
# @param      센서에서 출력 데이터
# @return     COGNEX ERROR_CODE
# @exception  없음
#
def cognex_error_check(rx_data):
    global error_code
    rx_msg=rx_data.decode()
    data=rx_msg.split('\r\n')
    if isfloat(data[0]):
        res=float(data[0])
    else:
        res=-1


    error_code = "COGNEX_(" + str(res)+")"    

    return res



# =============================================================================================
# @brief      씨크 비전 센서 연결
# @details    vs_connect 호출시 실행
# @param      ip address (string)
# @return     연결성공 1, 연결실패 -1
# @exception  DR_ERROR_VALUE : SICK SENSOR ERROR
#
def sick_connect(ip_addr):
    global telnet_sock    
    telnet_sock = client_socket_open(ip_addr, 2115)    
    client_socket_write(telnet_sock, b"gVER\r\n")   
    time.sleep(DR_VS_WAIT)
    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)

    if sick_error_check(rx_data)!=0:
        raise DR_Error(DR_ERROR_VALUE, "Connect fail: {0}".format(vs_get_errorcode()))
        return -1
    
    res=sick_set_online()

    return res


# =============================================================================================
# @brief      씨크 비전 센서 해제
# @details    vs_disconnect 호출시 실행
# @param      없음
# @return     해제성공 1, 해제실패 -1
# @exception  DR_ERROR_VALUE : SICK SENSOR ERROR
#
def sick_disconnect():
    global telnet_sock
    res=client_socket_close(telnet_sock)   
    if res!=0:
        raise DR_Error(DR_ERROR_VALUE, "Disconnect fail")
        return -1
    
    return 1


# =============================================================================================
# @brief      비전센서 온라인으로 상태 변경
# @details    온라인 - 측정대기 상태, 트리거 입력시 측정을 실시한다.
# @param      없음
# @return     성공시 1, 실패시 -1
# @exception  DR_ERROR_VALUE : SICK SENSOR ERROR
# 
def sick_set_online():
    global telnet_sock

    client_socket_write(telnet_sock, b"sMOD 0\r\n")
    time.sleep(DR_VS_WAIT)
    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)

    if sick_error_check(rx_data)!=0:
        raise DR_Error(DR_ERROR_VALUE, "Set online: {0}".format(vs_get_errorcode()))
        return -1

    return 1


# =============================================================================================
# @brief      비전센서 오프라인으로 상태 변경
# @details    오프라인 - 센서 설정 상태, 작업변경/저장/로딩 등의 센서 설정이 가능한 상태이다.
# @param      없음
# @return     성공시 1, 실패시 -1
# @exception  
#
def sick_set_offline():
    global telnet_sock
    client_socket_write(telnet_sock, b"sMOD 1\r\n")
    time.sleep(DR_VS_WAIT)
    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)

    if sick_error_check(rx_data)!=0:
        raise DR_Error(DR_ERROR_VALUE, "Set offline: {0}".format(vs_get_errorcode()))
        return -1

    return 1    


# =============================================================================================
# @brief      비전센서 재부팅
# @param      없음
# @return     성공시 1, 실패시 -1  
# @exception  
#
def sick_reset():
    global telnet_sock

    client_socket_write(telnet_sock, b"aACT 6\r\n")
    time.sleep(DR_VS_WAIT)
    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)

    if sick_error_check(rx_data)!=0:
        raise DR_Error(DR_ERROR_VALUE, "Sensor reset: {0}".format(vs_get_errorcode()))
        return -1

    return 1 



# =============================================================================================
# @brief      센서에 측정명령 전달, 측정결과 데이터 불러오기
# @details    센서 출력값 형식 : pos,x,y,z,angle,var1,var2,...
# @param      없음
# @return     posx, var_list / 측정실패시 -1, []
# @exception  
#    
def sick_trigger():
    global telnet_sock

	# Send Trigger signal
    client_socket_write(telnet_sock, b"TRIG\r\n")
    time.sleep(DR_VS_WAIT_TRIG)
    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)
    
    if sick_error_check(rx_data)!=0:
        raise DR_Error(DR_ERROR_VALUE, "Send trigger: {0}".format(vs_get_errorcode()))
        return -1,[]
    
    time.sleep(DR_VS_WAIT)

    
    # Recv Result
    client_socket_write(telnet_sock, b"gRES\r\n")
    time.sleep(DR_VS_WAIT)
    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)
    
    if sick_error_check(rx_data)!=0:
        raise DR_Error(DR_ERROR_VALUE, "Get result: {0}".format(vs_get_errorcode()))
        return -1,[]    


    rx_msg=rx_data.decode()
    rx_msg=rx_msg.replace('\x02', '')
    rx_msg=rx_msg.replace('\x03', '')
    data=rx_msg.split(' ')

    pos_string = data[2].split(',')


	# data format = pos,xval,yval,tval,var1,var2,var3, ...
    # data format = pos,12.3,45.6,78.9,1,0,147.9, ...
    # rgRES 0 pos,0.00,0.00,0.00,1,77713

    pos=-1
    var_list=[]

    if len(pos_string) >= 4 and pos_string[0]=="pos":
        #print("11111111 %s / %s / %s / %s", pos_string[1], pos_string[2], pos_string[3], pos_string[4])
        if isfloat(pos_string[1]) and isfloat(pos_string[2]) and isfloat(pos_string[3]) and isfloat(pos_string[4]):
            pos=[float(pos_string[1]), float(pos_string[2]), float(pos_string[3]), 0, 0, float(pos_string[4])]
    else:
    	return -1,[]


    for i in range(len(pos_string)):
        if i >=5:
            if isfloat(pos_string[i]):
                var_list.append(float(pos_string[i]))
            
    return pos, var_list


# =============================================================================================
# @brief      저장된 작업파일 리스트 불러오기
# @details    비전작업파일의 리스트를 얻어온다.
# @param      없음
# @return     성공시 job_name[list(string)] / 실패시 -1 
# @exception  
#
def sick_get_job_list():
    global telnet_sock
    client_socket_write(telnet_sock, b"gINT 2\r\n")
    time.sleep(DR_VS_WAIT)
    
    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)
    if sick_error_check(rx_data)!=0:
        raise DR_Error(DR_ERROR_VALUE, "Get job list: {0}".format(vs_get_errorcode()))
        return -1
    
    rx_msg=rx_data.decode()
    rx_msg=rx_msg.replace('\x02', '')
    rx_msg=rx_msg.replace('\x03', '')
    data=rx_msg.split(' ')
    job_num = int(data[3])

    vs_data=[]
    
    for i in range(0, job_num):
        _str_cmd = "gSTR 2 " + str(i) + "\r\n"
        client_socket_write(telnet_sock, _str_cmd.encode())
        time.sleep(DR_VS_WAIT)
        res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)
        
        if sick_error_check(rx_data)!=0:
            raise DR_Error(DR_ERROR_VALUE, "Get job name: {0}".format(vs_get_errorcode()))
            return -1
        
        rx_msg=rx_data.decode()
        data=rx_msg.split('\r\n')
        job = data[0][11:-1]
        vs_data.append(job)

    return vs_data


# =============================================================================================
# @brief      현재 설정된 작업 불러오기
# @param      없음
# @return     성공시 job_name(string), 실패시 -1
# @exception  
#
def sick_get_job():
    global telnet_sock
    client_socket_write(telnet_sock, b"gINT 1\r\n")
    time.sleep(DR_VS_WAIT)
    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)

    if sick_error_check(rx_data)!=0:
        raise DR_Error(DR_ERROR_VALUE, "Get job: {0}".format(vs_get_errorcode()))
        return -1
    
    rx_msg=rx_data.decode()
    data=rx_msg.split('\r\n')
    res = data[0]

    job_split = res.split(' ')
    
    _str_cmd = "gSTR 2 " + job_split[3][:-1] + "\r\n"
    client_socket_write(telnet_sock, _str_cmd.encode())
    time.sleep(DR_VS_WAIT)
    
    res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)
    if sick_error_check(rx_data)!=0:
        raise DR_Error(DR_ERROR_VALUE, "Get job name: {0}".format(vs_get_errorcode()))
        return -1

    rx_msg=rx_data.decode()
    data=rx_msg.split('\r\n')	

    job = data[0][11:-1]
    return job


# =============================================================================================
# @brief      입력된 작업을 비전센서의 설정작업으로 로딩함.
# @details    입력된 작업은 재부팅 후에도 초기작업으로 설정됨.
# @param      job_name(string) - 대상작업
# @return     성공시 1, 실패시 -1
# @exception  
#
def sick_set_job(job_name):
    global telnet_sock    
    
    sick_set_offline()    
    vs_data = sick_get_job_list()
    
    count = 0
    for i in range(len(vs_data)):
        if job_name == vs_data[i]:
            select_job_str = "sINT 1 " + str(i) + "\r\n"
            client_socket_write(telnet_sock, select_job_str.encode())
            time.sleep(DR_VS_WAIT)
            res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)           
            

            if sick_error_check(rx_data)!=0:
                err_code=vs_get_errorcode()
                sick_set_online()
                raise DR_Error(DR_ERROR_VALUE, "Set job: {0}".format(err_code))
                return -1


            client_socket_write(telnet_sock, b"aACT 1\r\n")

            for i in range(5):
                time.sleep(1)
                res, rx_data = client_socket_read(telnet_sock, -1, DR_VS_TIMEOUT)
                if rx_data!=None:
                    break

            if sick_error_check(rx_data)!=0:
                err_code=vs_get_errorcode()
                sick_set_online()
                raise DR_Error(DR_ERROR_VALUE, "Save setting: {0}".format(err_code))
                return -1
            else:
                sick_set_online()
                return 1
    
    sick_set_online()
    raise DR_Error(DR_ERROR_VALUE, "No File (-1)")
    return -1


# =============================================================================================
# @brief      씨크 비전 센서의 오류발생을 확인
# @details    출력된 ERROR_CODE는 SICK PIM60매뉴얼의 ERROR_CODE 테이블 참조
# @param      센서에서 출력 데이터
# @return     SICK ERROR_CODE
# @exception  없음
#
def sick_error_check(rx_data):
    global error_code

    rx_msg=rx_data.decode()
    rx_msg=rx_msg.replace('\x02', '')
    rx_msg=rx_msg.replace('\x03', '')
    
    data=rx_msg.split(' ')      


    if len(data) < 2:
        return -2

    
    if data[0]=="rgVER" or data[0]=="rsMOD" or data[0]=="rgMOD" or data[0]=="rTRIG" or data[0]=="rgRES":
        err_idx=1
    elif data[0]=="rsINT" or data[0]=="rgINT" or data[0]=="raACT" or data[0]=="rgSTR":
        err_idx=2
    else:
        raise DR_Error(DR_ERROR_VALUE, "SICK Error -Invalid command")
        return -1
    
    if isfloat(data[err_idx]):
        res=float(data[err_idx])
    else:
        res=-1
    
    error_code = "SICK_(" + str(res)+")"

    if res!=0:
        raise DR_Error(DR_ERROR_VALUE, "SICK Error - Code:{0}".format(res))

    return res


# =============================================================================================
# @brief      마지막에 발생된 에러코드 
# @param      없음
# @return     ERROR_CODE(STRING)
# @exception  없음
#
def vs_get_errorcode():
    global error_code
    return error_code


# =============================================================================================
# @brief      입력된 데이터가 FLOAT 형식인지 확인
# @param      data
# @return     float 일때 1
# @exception  없음
#
def isfloat(data):
    data1=data.replace('.','')
    data2=data1.replace('-','')
    data3=data2.replace(' ','')
    return data3.isdigit()


# =============================================================================================
# @brief      커스텀 비전 센서에 연결
# @details    사용자가 만든 비전서버에 연결 (TCP/IP Socket) - vs_connect 호출시 실행
# @param      ip address (string), port number (int)
# @return     성공시 0, 실패시 -1
# @exception  
#
def VS_Connect(ip, port):
    global H_vs_sock

    # close opened socket
    if H_vs_sock != None:
        H_vs_sock.close()
        time.sleep(0.5)

    try:
        H_vs_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        H_vs_sock.settimeout(3)

        server_address = (ip, port)
        H_vs_sock.connect(server_address)

    except socket.error as msg:
        print("Socket Error: {0}".format(msg))
        if H_vs_sock != None:
            H_vs_sock.close()
            H_vs_sock = None
            time.sleep(0.5)
        return -1

    return 0


# =============================================================================================
# @brief      커스텀 비전 센서에 연결해제
# @details    vs_disconnect 호출시 실행
# @param      없음
# @return     성공시 0, 실패시 -1
# @exception  
#
def VS_Disconnect():
    global H_vs_sock
    res = 0

    if H_vs_sock != None:
        res = H_vs_sock.close()
        H_vs_sock = None
        print("H_vs_sock.close()!!!")
    return res


# =============================================================================================
# @brief      커스텀 비전센서에 측정명령 전달
# @details    VS_request 함수 호출
# @param      cmd : 대상작업ID, (int : 1, 2, ...)
# @return     측정 성공시 0. 측정 실패시 -1
# @exception  
#
def vs_request(cmd):
    res = VS_request(cmd)
    return res


# =============================================================================================
# @brief      커스텀 비전센서에 측정명령 전달
# @details    측정을 실행할 작업을 선택하고, 측정의 성공/실패 여부에 대한 결과를 받는다.
#             사용자가 사전에 비전센서의 작업에 대한 정보를 알고있어야 하며, 센서에 복수 작업 로딩이 가능해야 한다.
# @param      cmd : 대상작업ID, (int : 1, 2, ...)
# @return     측정 성공시 0. 측정 실패시 -1
# @exception  없음
#
def VS_request(cmd):
    global H_vs_sock
    nRes=0

    _packet_data = b"MEAS_START"
    # command
    _packet_data += (cmd).to_bytes(4, byteorder='big')

    # print : send data
    hex_packet_data = ''.join('{:02X}'.format(x) for x in _packet_data)
    print("\n[SEND DATA(hex)] : {0}".format(hex_packet_data))

    H_vs_sock.sendall(_packet_data)

    ready = select.select([H_vs_sock], [], [], DR_VS_TIMEOUT)
    print("---> VS_request() receive O.K.") 

    if ready[0]:
        recv_packet = H_vs_sock.recv(1024)
    else:
        print("-2 = VS_request() : timeout")
        return -2

    print("\n[READ DATA] {0}".format(recv_packet)) 

    if recv_packet == b"MEAS_OK":
        nRes = 0
    else: 
        nRes = -1
    return nRes


# =============================================================================================
# @brief      커스텀 비전 센서의 측정 결과값을 받는다.
# @details    VS_result 함수 호출
# @param      없음
# @return     _cnt : 측정된 물체(obj) 수
#             obj_infos : 측정 결과 데이터(x, y, t 형태)
#             (예시) _ncnt=2, obj_infos=[x1,y1,t1,x2,y2,t2]
#             측정실패시 -2,[]
# @exception  없음
#
def vs_result():
    _cnt, obj_infos = VS_result()
    return _cnt, obj_infos


# =============================================================================================
# @brief      커스텀 비전 센서의 측정 결과값을 받는다.
#
def VS_result():
    global H_vs_sock
    nRes=0
    _cnt=0
    _x=0.0
    _y=0.0
    _t=0.0
    scale = 100
    obj_infos = []

    _packet_data = b"MEAS_REQUEST"

    # print : send data
    hex_packet_data = ''.join('{:02X}'.format(x) for x in _packet_data)
    print("\n[SEND DATA] : {0}".format(hex_packet_data))
    H_vs_sock.sendall(_packet_data)

    ready = select.select([H_vs_sock], [], [], DR_VS_TIMEOUT)
    print("---> VS_result() receive O.K.") 

    if ready[0]:
        recv_packet = H_vs_sock.recv(1024)
    else:
        print("-2 = HVIS_result() : timeout")
        return -2, []
    
    hex_packet_data = ''.join('{:02X}'.format(x) for x in recv_packet)

    # print : send data    
    #print("\n[READ DATA] {0}".format(recv_packet)) 
    #print("\n[READ DATA(hex)] : {0}".format(hex_packet_data))

    '''
    _cnt =  recv_packet[9]  << 24
    _cnt += recv_packet[10] << 16
    _cnt += recv_packet[11] << 8
    _cnt += recv_packet[12]
    '''
    _cnt = int.from_bytes(recv_packet[9:9+4], byteorder='big', signed=True)

    print("_cnt = {0}".format(_cnt))
    
    for i in range(_cnt):

        #b_robot_y = int.from_bytes(vp.get_data()[off:off + 4], byteorder='big', signed=True)
        '''
        _x =  recv_packet[(i*12)+13] << 24
        _x += recv_packet[(i*12)+14] << 16
        _x += recv_packet[(i*12)+15] << 8
        _x += recv_packet[(i*12)+16]

        _y =  recv_packet[(i*12)+17] << 24
        _y += recv_packet[(i*12)+18] << 16
        _y += recv_packet[(i*12)+19] << 8
        _y += recv_packet[(i*12)+20]

        _t =  recv_packet[(i*12)+21] << 24
        _t += recv_packet[(i*12)+22] << 16
        _t += recv_packet[(i*12)+23] << 8
        _t += recv_packet[(i*12)+24]
        '''
        _x = int.from_bytes(recv_packet[(i*12)+13:(i*12)+13+4], byteorder='big', signed=True)
        _y = int.from_bytes(recv_packet[(i*12)+17:(i*12)+17+4], byteorder='big', signed=True)
        _t = int.from_bytes(recv_packet[(i*12)+21:(i*12)+21+4], byteorder='big', signed=True)

        #print("_x={0},_y={1},_t={2}".format(_x,_y,_t))
        #info = [_x, _y, _t]
        info = [(_x/scale), (_y/scale), (_t/scale)]

        obj_infos.append(info)

    return _cnt, obj_infos
