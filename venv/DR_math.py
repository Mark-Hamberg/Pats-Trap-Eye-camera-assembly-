import random
import sys
from math import *
from typing import Tuple, List
from DR_common import *


def d2r(x) -> float:
    """
    This function returns the x radians value to degrees.

    :param x: float - The angle in degrees.
    :return: float - The angle in radians.
    """
    if type(x) != int and type(x) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : x")

    return radians(x)


def r2d(x) -> float:
    """
    This function returns the x radians value to degrees.

    :param x: float - The angle in radians.
    :return: float - The angle in degrees
    """
    if type(x) != int and type(x) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : x")

    return degrees(x)


def norm(x) -> float:
    """
    This function returns the L2 norm of x.

    :param x: float[3] - Point coordinate (x, y, z).
    :return: float - Norm of the point coordinate vector.
    """
    if type(x) != list or len(x) != 3:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : x")

    if is_number(x) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : x")

    temp = (x[0] * x[0]) + (x[1] * x[1]) + (x[2] * x[2])
    norm = sqrt(temp)

    return norm


def rotx(A) -> List[List[float]]:
    """
    This function returns a rotation matrix that rotates by the angle value along the x-axis.
    :param A: float - Rotating angle [deg].
    :return: Rotation matrix.
    """
    if type(A) != int and type(A) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : A")

    rotm_x = [[0] * 3 for i in range(3)]

    sin_a = sin(d2r(A))
    cos_a = cos(d2r(A))

    # rotation matrix -----
    rotm_x[0][0] = 1
    # rotm_x[0][1] = 0
    # rotm_x[0][2] = 0

    # rotm_x[1][0] = 0
    rotm_x[1][1] = cos_a
    rotm_x[1][2] = -sin_a

    # rotm_x[2][0] = 0
    rotm_x[2][1] = sin_a
    rotm_x[2][2] = cos_a

    return rotm_x


def roty(B) -> List[List[float]]:
    """
    This function returns a rotation matrix that rotates by the angle value along the y-axis.
    :param A: float - Rotating angle [deg].
    :return: Rotation matrix.
    """
    if type(B) != int and type(B) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : B")

    rotm_y = [[0] * 3 for i in range(3)]

    sin_b = sin(d2r(B))
    cos_b = cos(d2r(B))

    # rotation matrix -----
    rotm_y[0][0] = cos_b
    # rotm_y[0][1] = 0
    rotm_y[0][2] = sin_b

    # rotm_y[1][0] = 0
    rotm_y[1][1] = 1
    # rotm_y[1][2] = 0

    # rotm_y[2][0] = 0
    rotm_y[2][0] = -sin_b
    rotm_y[2][2] = cos_b

    return rotm_y


def rotz(C) -> List[List[float]]:
    """
    This function returns a rotation matrix that rotates by the angle value along the z-axis.
    :param A: float - Rotating angle [deg].
    :return: Rotation matrix.
    """
    if type(C) != int and type(C) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : C")

    rotm_z = [[0] * 3 for i in range(3)]

    sin_c = sin(d2r(C))
    cos_c = cos(d2r(C))

    # rotation matrix -----
    rotm_z[0][0] = cos_c
    rotm_z[0][1] = -sin_c
    # rotm_z[0][2] = 0

    rotm_z[1][0] = sin_c
    rotm_z[1][1] = cos_c
    # rotm_z[1][2] = 0

    # rotm_z[0][0] = 0
    # rotm_z[0][1] = 0
    rotm_z[2][2] = 1

    return rotm_z


def rotm2eul(rotm, flip=0) -> List[float]:
    """
    This function receives a rotation matrix and returns the Euler angle (zyz order) to degrees.
    Of the Euler angle (rx, ry, rz) returned as a result, ry is always a positive number.

    :param rotm: float[3][3] - Rotation matri
    :param flip:
    :return: float[3] - ZYZ Euler angle [deg].
    """
    # rotm
    if type(rotm) != list or len(rotm) != 3:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : rotm")

    for item in rotm:
        if type(item) != list or len(item) != 3:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : rotm[item]")

    if is_number(rotm) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : rotm")

    # rotation matrix (-1 ~ 1)
    for i in range(3):
        for j in range(3):
            if rotm[i][j] < -1 or rotm[i][j] > 1:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : rotm[item]")

    # rotation matrix (epsilon 미만의 값은 0으로 처리)
    for i in range(3):
        for j in range(3):
            if abs(rotm[i][j]) < sys.float_info.epsilon:
                rotm[i][j] = 0

    # flip
    if type(flip) != int:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : flip")

    if flip != 0 and flip != 1:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : flip")

    r11 = rotm[0][0]
    r12 = rotm[0][1]
    r13 = rotm[0][2]
    r21 = rotm[1][0]
    r22 = rotm[1][1]
    r23 = rotm[1][2]
    r31 = rotm[2][0]
    r32 = rotm[2][1]
    r33 = rotm[2][2]

    # calculate rx, ry, rz
    if abs(r13) < sys.float_info.epsilon and abs(r23) < sys.float_info.epsilon:
        rx = 0
        sp = 0
        cp = 1

        ry = atan2(cp * r13 + sp * r23, r33)
        rz = atan2(-sp * r11 + cp * r21, -sp * r12 + cp * r22)
    else:
        if flip == 0:
            rx = atan2(r23, r13)
            # ry = atan2(sqrt(r13 * r13 + r23 * r23), r33)
            # rz = atan2(r32, -r31)
        else: # -> (flip == 1)
            rx = atan2(-r23, -r13)
            #ry = atan2(-sqrt(r13 * r13 + r23 * r23), r33)
            #rz = atan2(-r32, r31)
        
        sp = sin(rx)
        cp = cos(rx)

        ry = atan2(cp * r13 + sp * r23, r33)
        rz = atan2(-sp * r11 + cp * r21, -sp * r12 + cp * r22)
       

    eulv = [r2d(rx), r2d(ry), r2d(rz)]

    return eulv


def eul2rotm(eulv) -> List[List[float]]:
    """
    This function transforms a Euler angle (zyz order) to a rotation matrix.

    :param eulv: float[3] - Euler angle (zyz) [deg].
    :return: float[3][3] - Rotation matrix.
    """
    if type(eulv) != list or len(eulv) != 3:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : eulv")

    if is_number(eulv) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : eulv")

    rotm = [[0] * 3 for i in range(3)]

    rx_r = d2r(eulv[0])
    ry_r = d2r(eulv[1])
    rz_r = d2r(eulv[2])

    sin_rx = sin(rx_r)
    cos_rx = cos(rx_r)

    sin_ry = sin(ry_r)
    cos_ry = cos(ry_r)

    sin_rz = sin(rz_r)
    cos_rz = cos(rz_r)

    # rotation matrix
    rotm[0][0] = (cos_rx * cos_ry * cos_rz) - (sin_rx * sin_rz)
    rotm[0][1] = -(cos_rx * cos_ry * sin_rz) - (sin_rx * cos_rz)
    rotm[0][2] = cos_rx * sin_ry

    rotm[1][0] = (sin_rx * cos_ry * cos_rz) + (cos_rx * sin_rz)
    rotm[1][1] = -(sin_rx * cos_ry * sin_rz) + (cos_rx * cos_rz)
    rotm[1][2] = sin_rx * sin_ry
    
    rotm[2][0] = -(sin_ry * cos_rz)
    rotm[2][1] = sin_ry * sin_rz
    rotm[2][2] = cos_ry

    return rotm


def rotm2rotvec(rotm) -> List[float]:
    """
    This  function  receives  a  rotation  matrix  and  returns  the  rotation  vector  (angle/axis representation).

    :param rotm: float[3][3] - Rotation Matrix
    :return: float[3] - rotation vector
    """
    if type(rotm) != list or len(rotm) != 3:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : rotm")

    for item in rotm:
        if type(item) != list or len(item) != 3:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : rotm[item]")

    if is_number(rotm) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : rotm")

    # rotation matrix (-1 ~ 1)
    for i in range(3):
        for j in range(3):
            if rotm[i][j] < -1 or rotm[i][j] > 1:
                raise DR_Error(DR_ERROR_VALUE, "Invalid value : rotm[item]")

    # rotation matrix (epsilon 미만의 값은 0으로 처리)
    for i in range(3):
        for j in range(3):
            if abs(rotm[i][j]) < sys.float_info.epsilon:
                rotm[i][j] = 0

    rotvec = [0, 0, 0]

    r11 = rotm[0][0]
    r12 = rotm[0][1]
    r13 = rotm[0][2]
    r21 = rotm[1][0]
    r22 = rotm[1][1]
    r23 = rotm[1][2]
    r31 = rotm[2][0]
    r32 = rotm[2][1]
    r33 = rotm[2][2]


    theta = acos((r11 + r22 + r33 - 1) / 2.0)
    sin_th = sin(theta)

    rv = [0, 0, 0]

    # special case
    if abs(sin_th) < sys.float_info.epsilon:
        if abs(theta) < sys.float_info.epsilon:
            rv = [1, 0, 0]
        else:
            if abs(theta - pi) < sys.float_info.epsilon:
#                if r11 > 0 and r22 < 0 and r33 < 0:
#                    rv = [1, 0, 0]
#                elif r11 < 0 and r22 > 0 and r33 < 0:
#                    rv = [0, 1, 0]
#                elif r11 < 0 and r22 < 0 and r33 > 0:
#                    rv = [0, 0, 1]
#                else:
                    # need to check
#                    rv = [1, 0, 0]
#            else:
                # need to check
#                rv = [1, 0, 0]
                if (r11 + 1) > sys.float_info.epsilon:
                    #a = pi / sqrt(2*(1+r11))
                    a = 1 / sqrt(2*(1+r11))
                    rv = [a*(1+r11), a*r21, a*r31]
                elif (r22 + 1) > sys.float_info.epsilon:
                    #a = pi / sqrt(2*(1+r22))
                    a = 1 / sqrt(2*(1+r22))
                    rv = [a*r12, a*(1+r22), a*r32]
                else:
                    #a = pi / sqrt(2*(1+r33))
                    a = 1 / sqrt(2*(1+r33))
                    rv = [a*r13, a*r23, a*(1+r33)]
    else:
        rv[0] = 1 / (2 * sin_th) * (r32 - r23)
        rv[1] = 1 / (2 * sin_th) * (r13 - r31)
        rv[2] = 1 / (2 * sin_th) * (r21 - r12)

    rotvec[0] = rv[0] * theta
    rotvec[1] = rv[1] * theta
    rotvec[2] = rv[2] * theta

    return rotvec


def rotvec2rotm(rotvec) -> List[List[float]]:
    """
    This function transforms a rotation vector to a rotation matrix.

    :param rotvec: float[3] - rotation vector.
    :return: float[3][3] - rotation matrix.
    """
    if type(rotvec) != list or len(rotvec) != 3:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : rotvec")

    if is_number(rotvec) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : rotvec")

    rotm = [[0] * 3 for i in range(3)]

    rv1 = rotvec[0]
    rv2 = rotvec[1]
    rv3 = rotvec[2]

    theta = sqrt((rv1 * rv1) + (rv2 * rv2) + (rv3 * rv3))
    if abs(theta) < sys.float_info.epsilon:
        r1 = 1.0
        r2 = 0.0
        r3 = 0.0
    else:
        r1 = rv1 / theta
        r2 = rv2 / theta
        r3 = rv3 / theta

    sin_th = sin(theta)
    cos_th = cos(theta)

    # rotation matrix
    rotm[0][0] = (r1 * r1 * (1 - cos_th)) + cos_th
    rotm[0][1] = (r1 * r2 * (1 - cos_th)) - (r3 * sin_th)
    rotm[0][2] = (r1 * r3 * (1 - cos_th)) + (r2 * sin_th)

    rotm[1][0] = (r1 * r2 * (1 - cos_th)) + (r3 * sin_th)
    rotm[1][1] = (r2 * r2 * (1 - cos_th)) + cos_th
    rotm[1][2] = (r2 * r3 * (1 - cos_th)) - (r1 * sin_th)

    rotm[2][0] = (r1 * r3 * (1 - cos_th)) - (r2 * sin_th)
    rotm[2][1] = (r2 * r3 * (1 - cos_th)) + (r1 * sin_th)
    rotm[2][2] = (r3 * r3 * (1 - cos_th)) + cos_th

    return rotm


def eul2rotvec(eulv) -> List[float]:
    """
    This function transforms a Euler angle (zyz order) to a rotation vector.

    :param eulv: float[3] - Euler angle (zyz) [deg].
    :return: float[3] - rotation vector.
    """
    # eulv
    if type(eulv) != list or len(eulv) != 3:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : eulv")

    if is_number(eulv) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : eulv")

    rotm = eul2rotm(eulv)
    rotvec = rotm2rotvec(rotm)

    return rotvec


def rotvec2eul(rotvec) -> List[float]:
    """
    This function transforms a rotation vector to a Euler angle (zyz).

    :param rotvec: float[3] - This function transforms a rotation vector to a Euler angle (zyz).
    :return: float[3] - ZYZ Euler angle [deg].
    """
    if type(rotvec) != list or len(rotvec) != 3:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : rotvec")

    if is_number(rotvec) != True:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : rotvec")

    rotm = rotvec2rotm(rotvec)
    eulv = rotm2eul(rotm)

    return eulv


def get_distance(posx1, posx2) -> float:
    """
    This function returns the distance between two pose positions in [mm].

    :param posx1: float[6] - posx
    :param posx2: float[6] - posx
    :return: float - distance [mm]
    """
    _posx1 = get_posx(posx1)
    _posx2 = get_posx(posx2)

    x1 = _posx1[0]
    y1 = _posx1[1]
    z1 = _posx1[2]

    x2 = _posx2[0]
    y2 = _posx2[1]
    z2 = _posx2[2]

    distance = sqrt((x2 - x1) * (x2 - x1) +
                    (y2 - y1) * (y2 - y1) +
                    (z2 - z1) * (z2 - z1))

    return distance


def htrans(posx1, posx2) -> posx:
    """
    This  function  returns  the  pose  corresponding  to  T1*T2  assuming  that  the  homogeneous transformation
    matrices obtained from posx1 and posx2 are T1 and T2, respectively.

    :param posx1: posx
    :param posx2: posx
    :return: posx
    """
    _posx1 = get_posx(posx1)
    _posx2 = get_posx(posx2)

    x1 = _posx1[0]
    y1 = _posx1[1]
    z1 = _posx1[2]
    rx1 = _posx1[3]
    ry1 = _posx1[4]
    rz1 = _posx1[5]

    x2 = _posx2[0]
    y2 = _posx2[1]
    z2 = _posx2[2]
    rx2 = _posx2[3]
    ry2 = _posx2[4]
    rz2 = _posx2[5]

    # trans, rotm
    trans1 = [x1, y1, z1]
    rotm1 = eul2rotm([rx1, ry1, rz1])

    trans2 = [x2, y2, z2]
    rotm2 = eul2rotm([rx2, ry2, rz2])

    # result
    #result_rotm = mat(rotm1) @ mat(rotm2)  #support in python3.5
    result_rotm = matrix_mul(mat(rotm1), mat(rotm2))
    
    transp_trans1 = transpose([trans1])
    transp_trans2 = transpose([trans2])

    #temp = mat(transp_trans1) + (mat(rotm1) @ mat(transp_trans2))  #support in python3.5
    temp = mat(transp_trans1) + matrix_mul(mat(rotm1), mat(transp_trans2)) 

    result_trans = transpose(temp)

    result_eulv = rotm2eul(result_rotm)

    result_posx = result_trans[0] + result_eulv
    # result_posx = posx(result_posx)

    return result_posx


def inverse_pose(posx) -> posx:
    """
    This function returns the posx value that represents the inverse of posx.

    :param posx: posx
    :return: posx
    """
    _posx = get_posx(posx)

    x = _posx[0]
    y = _posx[1]
    z = _posx[2]
    rx = _posx[3]
    ry = _posx[4]
    rz = _posx[5]

    trans = [x, y, z]
    rotm = eul2rotm([rx, ry, rz])

    transp_trans = transpose([trans])
    transp_rotm = transpose(rotm)

    # result
    #temp = -(mat(transp_rotm) @ mat(transp_trans))  #support in python3.5
    temp = -( mat( matrix_mul(mat(transp_rotm), mat(transp_trans)) ) )
    result_trans = transpose(temp)
    result_eulv = rotm2eul(transp_rotm)
    result_posx = result_trans[0] + result_eulv
    # result_posx = posx(result_posx)

    return result_posx


def add_pose(posx1, posx2) -> posx:
    """
    This function obtains the sum of two poses.

    :param posx1: posx
    :param posx2: posx
    :return: posx
    """
    _posx1 = get_posx(posx1)
    _posx2 = get_posx(posx2)

    x1 = _posx1[0]
    y1 = _posx1[1]
    z1 = _posx1[2]
    rx1 = _posx1[3]
    ry1 = _posx1[4]
    rz1 = _posx1[5]

    x2 = _posx2[0]
    y2 = _posx2[1]
    z2 = _posx2[2]
    rx2 = _posx2[3]
    ry2 = _posx2[4]
    rz2 = _posx2[5]

    # trans, rotm
    trans1 = [x1, y1, z1]
    rotm1 = eul2rotm([rx1, ry1, rz1])

    trans2 = [x2, y2, z2]
    rotm2 = eul2rotm([rx2, ry2, rz2])

    # result
    #result_rotm = mat(rotm1) @ mat(rotm2)  #support in python3.5
    result_rotm = matrix_mul(mat(rotm1), mat(rotm2))
    result_trans = mat([trans1]) + mat([trans2])
    result_eulv = rotm2eul(result_rotm)

    result_posx = result_trans[0] + result_eulv
    # result_posx = posx(result_posx)

    return result_posx


def subtract_pose(posx1, posx2) -> posx:
    """
    This function obtains the difference between two poses.

    :param posx1: posx
    :param posx2: posx
    :return: posx
    """
    _posx1 = get_posx(posx1)
    _posx2 = get_posx(posx2)

    x1 = _posx1[0]
    y1 = _posx1[1]
    z1 = _posx1[2]
    rx1 = _posx1[3]
    ry1 = _posx1[4]
    rz1 = _posx1[5]

    x2 = _posx2[0]
    y2 = _posx2[1]
    z2 = _posx2[2]
    rx2 = _posx2[3]
    ry2 = _posx2[4]
    rz2 = _posx2[5]

    # trans, rotm
    trans1 = [x1, y1, z1]
    rotm1 = eul2rotm([rx1, ry1, rz1])

    trans2 = [x2, y2, z2]
    rotm2 = eul2rotm([rx2, ry2, rz2])

    transp_rotm2 = transpose(rotm2)

    # result
    #result_rotm = mat(transp_rotm2) @ mat(rotm1)  #support in python3.5
    result_rotm = matrix_mul(mat(transp_rotm2), mat(rotm1))
    result_trans = mat([trans1]) - mat([trans2])
    result_eulv = rotm2eul(result_rotm)

    result_posx = result_trans[0] + result_eulv
    # result_posx = posx(result_posx)

    return result_posx


def get_intermediate_pose(posx1, posx2, alpha) -> posx:
    """
    This function returns posx  located at alpha  of the  linear  transition from posx1 to  posx2.
    It returns posx1 if alpha is 0, the median value of two poses ifalpha is 0.5, and posx2 if alpha is 1.

    :param posx1: posx
    :param posx2: posx
    :param alpha: float
    :return: posx
    """
    _posx1 = get_posx(posx1)
    _posx2 = get_posx(posx2)

    # alpha
    if type(alpha) != int and type(alpha) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : alpha")

    if alpha < 0 or alpha > 1:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : alpha")

    x1 = _posx1[0]
    y1 = _posx1[1]
    z1 = _posx1[2]
    rx1 = _posx1[3]
    ry1 = _posx1[4]
    rz1 = _posx1[5]

    x2 = _posx2[0]
    y2 = _posx2[1]
    z2 = _posx2[2]
    rx2 = _posx2[3]
    ry2 = _posx2[4]
    rz2 = _posx2[5]

    # trans, rotm
    trans1 = [x1, y1, z1]
    rotm1 = eul2rotm([rx1, ry1, rz1])

    trans2 = [x2, y2, z2]
    rotm2 = eul2rotm([rx2, ry2, rz2])

    #rotm = mat(transpose(rotm1)) @ mat(rotm2)  #support in python3.5
    rotm = matrix_mul(mat(transpose(rotm1)), mat(rotm2))
    rotvec = rotm2rotvec(rotm)

    rv1 = rotvec[0]
    rv2 = rotvec[1]
    rv3 = rotvec[2]

    theta = sqrt((rv1 * rv1) + (rv2 * rv2) + (rv3 * rv3))
    if abs(theta) < sys.float_info.epsilon:
        r1 = 1.0
        r2 = 0.0
        r3 = 0.0
    else:
        r1 = rv1 / theta
        r2 = rv2 / theta
        r3 = rv3 / theta

    result_theta = theta * alpha

    temp = mat([trans2]) - mat([trans1])

    temp = temp[0]
    temp[0] = temp[0] * alpha
    temp[1] = temp[1] * alpha
    temp[2] = temp[2] * alpha

    result_trans = mat([trans1]) + mat([temp])

    rv1 = r1 * result_theta
    rv2 = r2 * result_theta
    rv3 = r3 * result_theta
    result_rotvec = [rv1, rv2, rv3]

    # result
    inter_rotm = rotvec2rotm(result_rotvec)
    #result_rotm = mat(rotm1) @ mat(inter_rotm)  #support in python3.5
    result_rotm = matrix_mul(mat(rotm1), mat(inter_rotm))
    result_eulv = rotm2eul(result_rotm)

    result_posx = result_trans[0] + result_eulv

    return result_posx

# =============================================================================================
##
# @brief      class for matrix
# @details    Matrix에 대한 사칙연산을 위한 class
#
class mat(list):
    ##
    # @brief      생성자
    # @details    list에서 상속받은 class이므로, list 객체를 초기화한다.
    # @return     없음
    # @exception  없음
    #
    def __init__(self, data):
        list.__init__(self, data)

    ##
    # @brief      matrix의 dimension을 구한다.
    # @param      mat - matrix (2 dimension list)
    # @return     row, col
    #               row - row의 개수
    #               col - col의 개수
    # @exception  없음
    #
    def _get_dimension(self, mat):
        row = len(mat)
        col = len(mat[0])

        # print("row={0}, col={1}".format(row, col))
        return row, col

    ##
    # @brief      operator - (단항)
    # @details    자신(matrix)의 모든 item을 negative 계산한 새로운 matrix를 리턴한다.
    # @return     matrix - negative 계산한 새로운 matrix
    # @exception  없음
    #
    def __neg__(self):
        row, col = self._get_dimension(self)

        result = [[0] * col for x in range(row)]

        for i in range(row):
            for j in range(col):
                result[i][j] = -self[i][j]

        return mat(result)

    ##
    # @brief      operator +
    # @details    자신(matrix)과 other의 matrix + 연산을 계산한 새로운 matrix를 리턴한다.
    # @param      other - matrix (2 dimension list)
    # @return     matrix - + 계산한 matrix
    # @exception  - DR_ERROR_TYPE : argument의 type 비정상
    #
    def __add__(self, other):
        row_a, col_a = self._get_dimension(self)
        row_b, col_b = self._get_dimension(other)

        if row_a != row_b or col_a != col_b:
            raise DR_Error(DR_ERROR_TYPE, "Inconsistant type : self, other")

        result = [[0] * col_a for x in range(row_a)]

        for i in range(row_a):
            for j in range(col_b):
                result[i][j] = self[i][j] + other[i][j]

        return mat(result)

    ##
    # @brief      operator - (이항)
    # @details    자신(matrix)과 other의 matrix - 연산을 계산한 새로운 matrix를 리턴한다.
    # @param      other - matrix (2 dimension list)
    # @return     matrix - -를 계산한 matrix
    # @exception  - DR_ERROR_TYPE : argument의 type 비정상
    #
    def __sub__(self, other):
        row_a, col_a = self._get_dimension(self)
        row_b, col_b = self._get_dimension(other)

        if row_a != row_b or col_a != col_b:
            raise DR_Error(DR_ERROR_TYPE, "Inconsistant type : self, other")

        result = [[0] * col_a for x in range(row_a)]

        for i in range(row_a):
            for j in range(col_b):
                result[i][j] = self[i][j] - other[i][j]

        return mat(result)

    '''
    __matmul__(@) 연산자 오버로딩은 python3.5 이상 부터 지원 가능   
    현재 리눅스에서는 python3.2를 쓰고 있어서 연산자 오버로딩 사용할 수 없어서
    matrix_mul(A, B) 함수를 이용하여 matrix 곱하기를 수행함. by kabdol2 2017/04/27        
    '''

    ##
    # @brief      operator @
    # @details    자신(matrix)과 other의 matrix 행렬곱 연산을 계산한 새로운 matrix를 리턴한다.
    # @param      other - matrix (2 dimension list)
    # @return     matrix - 행렬곱 계산한 matrix
    # @exception  없음
    #
    def __matmul__(self, other):
        row_a, col_a = self._get_dimension(self)
        row_b, col_b = self._get_dimension(other)

        result = [[0] * col_b for x in range(row_a)]

        for i in range(row_a):
            for j in range(col_b):
                temp = 0

                for k in range(row_b):
                    temp += self[i][k] * other[k][j]

                result[i][j] = temp

        return mat(result)

# =============================================================================================
##
# @brief      matrix의 dimension을 구한다.
# @param      matrix - matrix (2 dimension list)
# @return     row, col
#               row - row의 개수
#               col - col의 개수
# @exception  없음
#
def cal_matrix(matrix):
    row = len(matrix)
    col = len(matrix[0])

    return row, col

# =============================================================================================
##
# @brief      matrix A와 matrix B의 행렬 곱을 계산한 새로운 matrix를 리턴한다.
# @param      A - matrix (2 dimension list)
# @param      B - matrix (2 dimension list)
# @return     result - 행렬곱 계산한 matrix
# @exception  없음
#
def matrix_mul(A, B):
    row_a, col_a = cal_matrix(A)
    row_b, col_b = cal_matrix(B)

    result = [[0] * col_b for x in range(row_a)]

    for i in range(row_a):
        for j in range(col_b):
            temp = 0
            for k in range(row_b):
                temp += A[i][k] * B[k][j]

            result[i][j] = temp

    return result

# =============================================================================================
##
# @brief      matrix의 transpose를 계산한 새로운 matrix를 리턴한다.
# @param      mat - matrix (2 dimension list)
# @return     result - transpose 계산한 matrix
# @exception  없음
#
def transpose(mat):
    row = len(mat)
    col = len(mat[0])

    # row -> col, col -> row
    result = [[0] * row for x in range(col)]

    for i in range(row):
        for j in range(col):
            result[j][i] = mat[i][j]

    return result


def dot_pose(posx1, posx2) -> float:
    """
    This  function  obtains  the  inner  product  of  the  translation  component  when  two  poses  are given.

    :param posx1: posx
    :param posx2: posx
    :return: float
    """
    _posx1 = get_posx(posx1)
    _posx2 = get_posx(posx2)

    x1 = _posx1[0]
    y1 = _posx1[1]
    z1 = _posx1[2]

    x2 = _posx2[0]
    y2 = _posx2[1]
    z2 = _posx2[2]

    _dot = (x1*x2) + (y1*y2) + (z1*z2) 
    return _dot


def cross_pose(posx1, posx2):
    """
    This  function  obtains  the  outer  product  of  the  translation  component  when  two  poses  are given.

    :param posx1: posx
    :param posx2: posx
    :return: posx
    """
    _posx1 = get_posx(posx1)
    _posx2 = get_posx(posx2)

    x1 = _posx1[0]
    y1 = _posx1[1]
    z1 = _posx1[2]

    x2 = _posx2[0]
    y2 = _posx2[1]
    z2 = _posx2[2]

    cross = [0, 0, 0]

    cross[0] = (y1 * z2) - (z1 * y2)
    cross[1] = (z1 * x2) - (x1 * z2)
    cross[2] = (x1 * y2) - (y1 * x2)

    return cross


def unit_pose(posx1):
    """
    This function obtains the unit vector of the given posx translation component.

    :param posx1: posx
    :return: float[3]
    """
    _posx1 = get_posx(posx1)

    x1 = _posx1[0]
    y1 = _posx1[1]
    z1 = _posx1[2]

    xyz = [x1, y1, z1]
    
    unit_vec = [0, 0, 0]

    unit_vec[0] = x1 / norm(xyz)
    unit_vec[1] = y1 / norm(xyz)
    unit_vec[2] = z1 / norm(xyz)

    return unit_vec

#______________________________________________________________________________________________
if __name__ == "__main__":
    # a = rotz(30)
    # print(a)

    print(norm([1, 2, 3]))
