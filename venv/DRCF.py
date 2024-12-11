from DR_common import *
from DR_math import *
from DR_thread import *
from DR_tcp_client import *
from DR_tcp_server import *
from DR_vision import *
from DR_ethernet import *
from DR_vision_eyedea import *
from DR_pickit import *
from robolink_v5 import *
from robodk_v5 import *

import getopt
from typing import Tuple, List

try:
    from DR_serial import *
except Exception:
    pass

try:
    from DR_variables_globals import *
except Exception:
    pass

try:
    from DR_variables_system import *
except Exception:
    pass

# Global Variables

# point count
POINT_COUNT = 6

# solution space
DR_SOL_MIN = 0
DR_SOL_MAX = 7

# posb seg_type
DR_LINE = 0
DR_CIRCLE = 1

# move reference
DR_BASE = 0
DR_TOOL = 1
DR_WORLD = 2
DR_TC_USER_MIN = 101
DR_TC_USER_MAX = 200

# move mod
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1

# move reaction
DR_MV_RA_NONE = 0
DR_MV_RA_DUPLICATE = 0
DR_MV_RA_OVERRIDE = 1

# move command type
DR_MV_COMMAND_NORM = 0

# movesx velocity
DR_MVS_VEL_NONE = 0
DR_MVS_VEL_CONST = 1

# motion state
DR_STATE_IDLE = 0
DR_STATE_INIT = 1
DR_STATE_BUSY = 2
DR_STATE_BLEND = 3
DR_STATE_ACC = 4
DR_STATE_CRZ = 5
DR_STATE_DEC = 6

# axis
DR_AXIS_X = 0
DR_AXIS_Y = 1
DR_AXIS_Z = 2
DR_AXIS_A = 10
DR_AXIS_B = 11
DR_AXIS_C = 12

# collision sensitivity
DR_COLSENS_DEFAULT = 20
DR_COLSENS_MIN = 1
DR_COLSENS_MAX = 300

# speed
DR_OP_SPEED_MIN = 1
DR_OP_SPEED_MAX = 100

# stop
DR_QSTOP_STO = 0
DR_QSTOP = 1
DR_SSTOP = 2
DR_PAUSE = 3
DR_HOLD = 4

DR_STOP_FIRST = DR_QSTOP_STO
DR_STOP_LAST = DR_HOLD

# condition
DR_COND_NONE = -10000

# digital I/O
DR_DIO_MIN_INDEX = 1
DR_DIO_MAX_INDEX = 16

# tool digital I/O
DR_TDIO_MIN_INDEX = 1
DR_TDIO_MAX_INDEX = 6

# I/O value
ON = 1
OFF = 0

# Analog I/O mode
DR_ANALOG_CURRENT = 0
DR_ANALOG_VOLTAGE = 1

# modbus type
DR_MODBUS_DIG_INPUT = 0
DR_MODBUS_DIG_OUTPUT = 1
DR_MODBUS_REG_INPUT = 2
DR_MODBUS_REG_OUTPUT = 3
DR_HOLDING_REGISTER = DR_MODBUS_REG_OUTPUT
DR_COIL = DR_MODBUS_DIG_OUTPUT

DR_MODBUS_ACCESS_MAX = 32
DR_MAX_MODBUS_NAME_SIZE = 32

# tp_popup pm_type
DR_PM_MESSAGE = 0
DR_PM_WARNING = 1
DR_PM_ALARM = 2

# tp_get_user_input type
DR_VAR_INT = 0
DR_VAR_FLOAT = 1
DR_VAR_STR = 2

# len
DR_VELJ_DT_LEN = 6
DR_ACCJ_DT_LEN = 6

DR_VELX_DT_LEN = 2
DR_ACCX_DT_LEN = 2

DR_ANGLE_DT_LEN = 2
DR_COG_DT_LEN =3
DR_WEIGHT_DT_LEN = 3
DR_VECTOR_DT_LEN = 3
DR_ST_DT_LEN = 6
DR_FD_DT_LEN = 6
DR_DIR_DT_LEN = 6
DR_INERTIA_DT_LEN = 6
DR_VECTOR_U1_LEN = 3
DR_VECTOR_V1_LEN = 3

# set_singular_handling mode 
DR_AVOID = 0
DR_TASK_STOP = 1
DR_VAR_VEL = 2

# alter motion
DR_DPOS = 0
DR_DVEL = 1

# homing options
DR_HOME_TARGET_MECHANIC = 0
DR_HOME_TARGET_USER = 1

# app type
DR_MV_APP_NONE = 0

# circular movement type
DR_MV_ORI_TEACH = 0
DR_MV_ORI_FIXED = 1
DR_MV_ORI_RADIAL = 2

# singularity mode
DR_SINGULARITY_ERROR = 0
DR_SINGULARITY_IGNORE = 1

# pallettizing mode
DR_OFF = 0
DR_ON = 1

# motion mode
DR_CHECK_OFF = 0
DR_CHECK_ON = 1

# flange reference
DR_CUR_TCP = 0
DR_FLANGE = 1

# tool replace policy
DR_REPLACE = 0
DR_ADD = 1
DR_REMOVE = 2

# port type
DR_CONTROLLER_DIGITAL = 0
DR_FLANGE_DIGITAL = 1
DR_CONTROLLER_ANALOG = 2
DR_FLANGE_ANALOG = 3

# =============================================================================================
# global variable

DR_CONFIG_PRT_EXT_RESULT = False
DR_CONFIG_PRT_RESULT = False

_g_blend_state = False
_g_blend_radius = 0.0

_g_velj = [0.0] * DR_VELJ_DT_LEN
_g_accj = [0.0] * DR_ACCJ_DT_LEN

_g_velx = [0.0] * DR_VELX_DT_LEN
_g_velx[0]= 0.0
_g_velx[1]= DR_COND_NONE

_g_accx = [0.0] * DR_ACCX_DT_LEN
_g_accx[0]= 0.0
_g_accx[1]= DR_COND_NONE

_g_coord = DR_BASE
_g_drl_result_th = None

_g_tp_lock = threading.Lock()       # only 1 execution allowed with TP

_g_test_cnt =0
_g_test_max =0

_g_analog_output_mode_ch1 = -1
_g_analog_output_mode_ch2 = -1

DR_FC_MOD_ABS = 0
DR_FC_MOD_REL = 1

# RoboDK implementation
_ROBODK_JOINT_SPACE_CONTROL = 1
_ROBODK_TASK_SPACE_CONTROL = 2
_ROBODK_POSITION_CONTROL = 3
_ROBODK_TORQUE_CONTROL = 4

_robodk_plugin_RDK = None
_robodk_plugin_RDK_backup = None
_robodk_plugin_path = None

options, args = getopt.getopt(
    sys.argv[1:], "p:",
    ["robodk_path="])

for name, value in options:
    if name in ('-p', '--robodk_path'):
        _robodk_plugin_path = value
        _robodk_plugin_RDK = Robolink(robodk_path=_robodk_plugin_path)
        _robodk_plugin_RDK_backup = Robolink(robodk_path=_robodk_plugin_path)
        _robodk_plugin_RDK.setSimulationSpeed(1)
        _robodk_plugin_robot = _robodk_plugin_RDK.Item('', ITEM_TYPE_ROBOT)
        _robodk_plugin_robot_backup = _robodk_plugin_RDK_backup.Item('', ITEM_TYPE_ROBOT)

        # add world frame
        world_frame = _robodk_plugin_RDK.Item('DR_WORLD', ITEM_TYPE_FRAME)
        if not world_frame.Valid():
            world_frame = _robodk_plugin_RDK.AddFrame("DR_WORLD")
            world_frame.setPose(transl(0, 0, 0))

        # add tool frame
        tool_frame = _robodk_plugin_RDK.Item('DR_TOOL', ITEM_TYPE_FRAME)
        if not tool_frame.Valid():
            tool_frame = _robodk_plugin_RDK.AddFrame("DR_TOOL")

        tool_frame.setVisible(False)
        pose_tool = _robodk_plugin_robot.PoseTool()
        tool_frame.setPose(pose_tool)
        tool_frame.setParent(_robodk_plugin_robot)

        # add flange frame
        flange_frame = _robodk_plugin_RDK.Item('DR_FLANGE', ITEM_TYPE_FRAME)
        if not flange_frame.Valid():
            flange_frame = _robodk_plugin_RDK.AddFrame("DR_FLANGE")

        flange_frame.setVisible(False)
        flange_frame.setPose(Pose(0, 0, 0, 0, 0, 0))
        flange_frame.setParent(_robodk_plugin_robot)



        _robodk_plugin_ref_map = {DR_WORLD: _robodk_plugin_RDK.Item('DR_WORLD', ITEM_TYPE_FRAME),
                                  DR_TOOL: _robodk_plugin_RDK.Item('DR_TOOL', ITEM_TYPE_FRAME),
                                  DR_BASE: _robodk_plugin_robot.Parent(),
                                  "DR_FLANGE": _robodk_plugin_RDK.Item('DR_FLANGE', ITEM_TYPE_FRAME)}

        print("RoboDK off-line simulation running!")

_robodk_plugin_j_vel = None
_robodk_plugin_j_acc = None
_robodk_plugin_vel = None
_robodk_plugin_acc = None
_robodk_plugin_r = -1
_robodk_plugin_ref = DR_BASE
_robodk_plugin_async = False
_robodk_control_mode = 3
_robodk_control_space = _ROBODK_JOINT_SPACE_CONTROL
_robodk_frame_count = 101

def _robodk_plugin_get_ref_frame(frame_id):
    if frame_id in _robodk_plugin_ref_map:
        return _robodk_plugin_ref_map[frame_id]
    else:
        return _robodk_plugin_RDK.Item('DR_FRAME_'+str(frame_id), ITEM_TYPE_FRAME)


def _robodk_plugin_get_ref_frame_backup(frame_id):
    if frame_id in _robodk_plugin_ref_map:
        return _robodk_plugin_ref_map[frame_id]
    else:
        return _robodk_plugin_RDK_backup.Item('DR_FRAME_'+str(frame_id), ITEM_TYPE_FRAME)


def _get_active_tool():
    pose_tool = _robodk_plugin_robot.PoseTool()
    tools = _robodk_plugin_RDK.ItemList(filter=ITEM_TYPE_TOOL, list_names=False)
    for t in tools:
        if t.Htool() == pose_tool:
            return t

# Functions
def get_control_mode() -> int:
    """
    This function returns the current control mode.

    :return: int (3 : Position control mode, 4 : Torque control mode)
    """
    if _robodk_plugin_RDK is not None:
        return _robodk_control_mode
    return None


def get_control_space() -> int:
    """
    This function returns the current control space.

    :return: int - (1 : Joint space control, 2 : Task space control)
    """
    if _robodk_plugin_RDK is not None:
        return _robodk_control_space
    return None


def get_current_posj() -> posj:
    """
    This function returns the current joint angles.

    :return: posj - Current joint angles.
    """
    if _robodk_plugin_RDK_backup is not None:
        return posj(_robodk_plugin_robot_backup.Joints().list())


def get_current_velj() -> List[float]:
    """
    This function returns the current joint velocity.

    :return: float[6] - Joint speed.
    """
    return None


def get_desired_posj() -> posj:
    """
    This function returns the current target joint angle. It cannot be used in the movel, movec, movesx, moveb, move_spiral, or move_periodic command.

    :return: posj - target joint angles.
    """
    return None


def get_desired_velj() -> List[float]:
    """
    This function returns the current target joint velocity. It cannot be used in the movel, movec, movesx, moveb, move_spiral, or move_periodic command.

    :return: float[6] - target joint angle velocities.
    """
    return None


def get_current_posx(ref=DR_BASE) -> Tuple[posx, int]:
    """
    This function returns the pose and solution space of the current coordinate system.
    The pose is based on the ref coordinate.

    :param ref: reference coordinate (DR_BASE, DR_WORLD, User Reference). Default: DR_BASE.
    :return: posx - Task space point, int - Solution space (0 ~ 7)
    """
    if _robodk_plugin_RDK_backup is not None:

        ref_frame = _robodk_plugin_get_ref_frame_backup(ref)
        tool_frame = _robodk_plugin_get_ref_frame_backup(DR_TOOL)

        ref_pose = ref_frame.PoseAbs()
        tool_frame = tool_frame.PoseAbs()

        ref_posx = posx(Pose_2_Comau(ref_pose))
        tool_posx = posx(Pose_2_Comau(tool_frame))

        #current_posx = subtract_pose(tool_posx, ref_posx)
        current_posx = htrans(inverse_pose(ref_posx), tool_posx)

        sol = get_current_solution_space()

        return current_posx, sol

    return None, None


def get_current_tool_flange_posx(ref=DR_BASE) -> posx:
    """
    This functionreturns the pose of the current tool flange based on the ref coordinate. In other words, it means the return to tcp=(0,0,0,0,0,0).

    :param ref: reference coordinate (DR_BASE, DR_WORLD, User Reference). Default: DR_BASE.
    :return: posx - Pose of tool flange.
    """
    if _robodk_plugin_RDK_backup is not None:
        ref_frame = _robodk_plugin_get_ref_frame_backup(ref)
        tool_frame = _robodk_plugin_get_ref_frame_backup('DR_FLANGE')

        ref_pose = ref_frame.PoseAbs()
        tool_frame = tool_frame.PoseAbs()

        ref_posx = posx(Pose_2_Comau(ref_pose))
        tool_posx = posx(Pose_2_Comau(tool_frame))

        current_posx = subtract_pose(tool_posx, ref_posx)

        return current_posx
    return None


def get_current_velx(ref=DR_BASE) -> List[float]:
    """
    This function returns the current tool velocity based on the ref coordinate.

    :param ref: reference coordinate (DR_BASE, DR_WORLD, User Reference). Default: DR_BASE.
    :return: float[6] - Tool velocity.
    """
    return None


def get_desired_posx(ref=DR_BASE) -> posx:
    """
    This function returns the target pose of the current tool. The pose is based on the ref coordinate.

    :param ref: reference coordinate (DR_BASE, DR_WORLD, User Reference). Default: DR_BASE.
    :return: posx - target Tool pose.
    """
    return None


def get_desired_velx(ref=DR_BASE) -> List[float]:
    """
    This function returns the target velocity of the current tool based on the ref coordinate. It cannot be used in the movej, movejx, or movesj command.

    :param ref: reference coordinate (DR_BASE, DR_WORLD, User Reference). Default: DR_BASE.
    :return: float[6] - target Tool velocity
    """
    return None


def get_current_solution_space() -> int:
    """
    This function returns the current solution space value.

    :return: int - Solution space (0 ~ 7)
    """
    joints = get_current_posj()
    return get_solution_space(joints)


def get_current_rotm(ref=DR_BASE) -> List[List[float]]:
    """
    This function returns the direction and matrix of the current toolbased on the ref coordinate.

    :param ref: reference coordinate (DR_BASE, DR_WORLD, User Reference). Default: DR_BASE.
    :return: float[3][3] - Rotation matrix.
    """
    if _robodk_plugin_RDK_backup is not None:
        current_posx,_ = get_current_posx(ref)
        return eul2rotm([current_posx[3], current_posx[4], current_posx[5]])


def get_joint_torque():
    """
    This function returns the sensor torque value of the current joints.

    :return: float[6] - JTS torque value.
    """
    return [0, 0, 0, 0, 0, 0]


def get_external_torque():
    """
    This function returns the torque value generated by the external force on each current joint.

    :return: float[6] - Torque value generated by an external force.
    """
    return [0, 0, 0, 0, 0, 0]


def get_tool_force(ref=DR_BASE):
    """
    This function returns the external force applied to the current toolbased on the ref coordinate. The force is based on the base coordinate while the moment is based on the tool coordinate.

    :param ref: reference coordinate (DR_BASE, DR_WORLD, User Reference). Default: DR_BASE.
    :return: float[6] - External force applied to the too
    """
    return [0, 0, 0, 0, 0, 0]


def addto(pos, add_val=None):
    """
    This function creates a new posj object by adding add_val to each joint value of posj.

    :param pos: posj - position list
    :param add_val: float[6] - List of add values to be added to the positio
    :return: posj - Joint space point.
    """
    # pos
    _pos = get_posj(pos)

    # return same position
    if add_val is None or add_val == []:
        return _pos

    if type(add_val) != list:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : add_val")

    if len(add_val) != POINT_COUNT:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : add_val")

    conv_pos = posj(0, 0, 0, 0, 0, 0)
    for i in range(0, POINT_COUNT):
        conv_pos[i] = _pos[i] + add_val[i]

    return conv_pos


def get_solution_space(pos) -> int:
    """
    This function obtains the solution space value.

    :param pos: posj - joint angle position
    :return: int - Solution space (0 ~ 7)
    """
    if _robodk_plugin_RDK_backup is not None:
        robodk_config = _robodk_plugin_robot_backup.JointsConfig(pos).list()
        return _sol_to_doosan(robodk_config)
    return None


def ikin(pos, sol_space=None, ref=DR_BASE, ref_pos_opt=None, iter_threshold=[0.005, 0.01]) -> posj:
    """
    This function returns the joint position corresponding to sol_space, which is equivalent to the robot pose in the operating space, among 8 joint shapes.

    :param pos: posx - tool position
    :param sol_space: int - Solution space (0 ~ 7)
    :param ref: reference coordinate (DR_BASE, DR_WORLD, User Reference). Default: DR_BASE.
    :param ref_pos_opt: int - Determine closest joint position depend on option among multi turn solutions. 0 : posj(0,0,0,0,0,0) is reference. 1 : current joint position is reference.
    :param iter_threshold: [float, float] If the accuracy has been corrected, the level of the accuracy correction algorithm.
    :return: posj - Joint space point
    """
    if _robodk_plugin_RDK_backup is not None:
        pose_tool = _robodk_plugin_robot_backup.PoseTool()
        frame = _robodk_plugin_get_ref_frame_backup(ref)
        pose = Comau_2_Pose(pos)
        base = _robodk_plugin_get_ref_frame_backup(DR_BASE)
        if sol_space is None:
            return _robodk_plugin_robot_backup.SolveIK(pose, tool=pose_tool, reference=base.PoseAbs().inv()*frame.PoseAbs()).list(), 0
        else:
            all_solutions = _robodk_plugin_robot_backup.SolveIK_All(pose, tool=pose_tool, reference=base.PoseAbs().inv()*frame.PoseAbs())
            for joint_sol in all_solutions:
                sol = get_solution_space(joint_sol)
                if sol_space == sol:
                    return posj(joint_sol[0:6]), 0

    return None, 1


def ikin_norm(pos, sol_space, ref=None, ref_pos_opt=None):
    """
    This function returns the joint position corresponding to sol_space, which is equivalent to the robot pose in the operating space, among 8 joint shapes.

    :param pos: posx - tool position
    :param sol_space: int - Solution space (0 ~ 7)
    :param ref: reference coordinate (DR_BASE, DR_WORLD, User Reference). Default: DR_BASE.
    :param ref_pos_opt: int - Determine closest joint position depend on option among multi turn solutions. 0 : posj(0,0,0,0,0,0) is reference. 1 : current joint position is reference.
    :return: posj - Joint space point
    """
    return ikin(pos, sol_space, ref, ref_pos_opt)


def fkin(pos, ref=DR_BASE) -> posx:
    """
    This function receives the input data of joint angles or equivalent forms (float[6]) in the joint space and returns the TCP (objects in the task space) based on the ref coordinate.

    :param pos: posj - joint position angle.
    :param ref: reference coordinate (DR_BASE, DR_WORLD, User Reference). Default: DR_BASE.
    :return: posx - Task space point.
    """
    if _robodk_plugin_RDK_backup is not None:
        pose_tool = _robodk_plugin_robot_backup.PoseTool()
        fkin_pose = _robodk_plugin_robot_backup.SolveFK(pos, tool=pose_tool)
        fkin_posx = Pose_2_Comau(fkin_pose)
        return coord_transform(fkin_posx,DR_BASE,ref)
    return None


def trans(pos, delta, ref=DR_BASE, ref_out=DR_BASE) -> posx:
    """
    Input parameter(pos)based on the ref coordinate is translated/rotated as delta based on the same coordinate and this function returns the result that is converted to the value based on the ref_out coordinate.
    In case that the ref coordinate is the tool coordinate, this function returns the value based on input parameter(pos)’s coordinate without ref_out coordinate.

    :param pos: posx - staring position
    :param delta: posx - delta increment to apply
    :param ref: reference coordinate (DR_BASE, DR_WORLD, User Reference). Default: DR_BASE.
    :param ref_out: reference coordinate (DR_BASE, DR_WORLD, User Reference). Default: DR_BASE.
    """
    new_pose = add_pose(pos, delta)
    return coord_transform(new_pose, ref_in=ref, ref_out=ref_out)


def set_velj(vel) -> int:
    """
    This  function  sets  the  global  velocity  in  joint  motion  (movej, movejx,  amovej,  or  amovejx) after using this command. The default velocity is applied to theglobally set vel if movej() is called without the explicit input of the velocity argument.

    :param vel: float or float[6] - velocity (same to all axes) or velocity (to each axis)
    """
    global _robodk_plugin_j_vel
    _robodk_plugin_j_vel = vel
    return 0


def set_accj(acc) -> int:
    """
    This function sets the global velocity in joint motion (movej, movejx,  amovej, or amovejx) after using this command. The globally set acceleration is applied as the default acceleration if movej() is called without the explicit input of the acceleration argument

    :param acc: float or float[6] - acceleration (same to all axes) or acceleration (to each axis)
    """
    global _robodk_plugin_j_acc
    _robodk_plugin_j_acc = acc
    return 0


def set_velx(vel1, vel2=DR_COND_NONE) -> int:
    """
    This function sets the velocity of the task space motion globally. The globally set velocity velx is  applied  as  the  default  velocity  if  the  task  motion  such  as  movel(),  amovel(),  movec(), movesx() is called without the explicit input of the velocity value. In the set value, vel1 and vel2 define the linear velocity and rotating velocity, relatively, of TCP.

    :param vel1: float - linear velocity
    :param vel2: float - rotation velocity
    """
    global _robodk_plugin_vel
    _robodk_plugin_vel = vel1

    return 0


def set_accx(acc1, acc2=DR_COND_NONE) -> int:
    """
    This  function  sets  the  acceleration  of  the  task  space  motion  globally.  The  globally  set acceleration accx is applied as the default acceleration if the task motion such as movel(), amovel(), movec(), movesx() is called without the explicit input of the acceleration value.In the  set  value,  acc1  and  acc2  define  the  linear  acceleration  and  rotating  acceleration, relatively, of the TCP.

    :param acc1: float - linear acceleration.
    :param acc2: float - rotational acceleration.
    """
    global _robodk_plugin_acc
    _robodk_plugin_acc = acc1

    return 0


def set_ref_coord(coord) -> int:
    """
    This function sets the reference coordinate system.

    :param coord: reference coordinate (DR_BASE, DR_WORLD, User Reference). Default: DR_BASE.
    """
    global _robodk_plugin_ref
    if _robodk_plugin_RDK is not None:
        ref_frame = _robodk_plugin_get_ref_frame(coord)
        _robodk_plugin_ref = coord
        _robodk_plugin_robot.setPoseFrame(ref_frame)
    return 0


def coord_transform(pose_in, ref_in, ref_out) -> posx:
    """
    This function transforms given task position expressed in reference coordinate, ‘ref_in’ to task position expressed in reference coordinate, ‘ref_out’. It returns transformed task position.

    :param pose_in: posx - Task space position.
    :param ref_in: reference coordinate (DR_BASE, DR_WORLD, User Reference). Default: DR_BASE.
    :param ref_out: reference coordinate (DR_BASE, DR_WORLD, User Reference). Default: DR_BASE.
    :return: posx - Task space position.
    """
    if _robodk_plugin_RDK_backup is not None:
        frame_in = _robodk_plugin_get_ref_frame_backup(ref_in)
        frame_out = _robodk_plugin_get_ref_frame_backup(ref_out)
        posx_in = Pose_2_Comau(frame_in.PoseAbs())
        posx_out = Pose_2_Comau(frame_out.PoseAbs())
        posx_abs = htrans(posx_in, pose_in)
        return htrans(inverse_pose(posx_out),posx_abs)


def movej(pos, vel=None, acc=None, time=None, radius=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None) -> int:
    """
    The robot moves to the target joint position (pos) from the current joint position.

    .. image:: data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAXwAAAEaCAYAAAASSuyNAAAACXBIWXMAAAsTAAALEwEAmpwYAAA4IGlUWHRYTUw6Y29tLmFkb2JlLnhtcAAAAAAAPD94cGFja2V0IGJlZ2luPSLvu78iIGlkPSJXNU0wTXBDZWhpSHpyZVN6TlRjemtjOWQiPz4KPHg6eG1wbWV0YSB4bWxuczp4PSJhZG9iZTpuczptZXRhLyIgeDp4bXB0az0iQWRvYmUgWE1QIENvcmUgNS41LWMwMjEgNzkuMTU0OTExLCAyMDEzLzEwLzI5LTExOjQ3OjE2ICAgICAgICAiPgogICA8cmRmOlJERiB4bWxuczpyZGY9Imh0dHA6Ly93d3cudzMub3JnLzE5OTkvMDIvMjItcmRmLXN5bnRheC1ucyMiPgogICAgICA8cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0iIgogICAgICAgICAgICB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmRjPSJodHRwOi8vcHVybC5vcmcvZGMvZWxlbWVudHMvMS4xLyIKICAgICAgICAgICAgeG1sbnM6cGhvdG9zaG9wPSJodHRwOi8vbnMuYWRvYmUuY29tL3Bob3Rvc2hvcC8xLjAvIgogICAgICAgICAgICB4bWxuczp4bXBNTT0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wL21tLyIKICAgICAgICAgICAgeG1sbnM6c3RFdnQ9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9zVHlwZS9SZXNvdXJjZUV2ZW50IyIKICAgICAgICAgICAgeG1sbnM6dGlmZj0iaHR0cDovL25zLmFkb2JlLmNvbS90aWZmLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmV4aWY9Imh0dHA6Ly9ucy5hZG9iZS5jb20vZXhpZi8xLjAvIj4KICAgICAgICAgPHhtcDpDcmVhdG9yVG9vbD5BZG9iZSBQaG90b3Nob3AgQ0MgKE1hY2ludG9zaCk8L3htcDpDcmVhdG9yVG9vbD4KICAgICAgICAgPHhtcDpDcmVhdGVEYXRlPjIwMTctMTEtMjBUMTY6NDU6NDYrMDk6MDA8L3htcDpDcmVhdGVEYXRlPgogICAgICAgICA8eG1wOk1vZGlmeURhdGU+MjAxNy0xMi0xM1QxNzo1MDoxNSswOTowMDwveG1wOk1vZGlmeURhdGU+CiAgICAgICAgIDx4bXA6TWV0YWRhdGFEYXRlPjIwMTctMTItMTNUMTc6NTA6MTUrMDk6MDA8L3htcDpNZXRhZGF0YURhdGU+CiAgICAgICAgIDxkYzpmb3JtYXQ+aW1hZ2UvcG5nPC9kYzpmb3JtYXQ+CiAgICAgICAgIDxwaG90b3Nob3A6Q29sb3JNb2RlPjM8L3Bob3Rvc2hvcDpDb2xvck1vZGU+CiAgICAgICAgIDx4bXBNTTpJbnN0YW5jZUlEPnhtcC5paWQ6NzVjYjU0NDEtMzgzYS00NzM3LTk1ZDctYjg2ZDJlMmZiMTI3PC94bXBNTTpJbnN0YW5jZUlEPgogICAgICAgICA8eG1wTU06RG9jdW1lbnRJRD54bXAuZGlkOjc1Y2I1NDQxLTM4M2EtNDczNy05NWQ3LWI4NmQyZTJmYjEyNzwveG1wTU06RG9jdW1lbnRJRD4KICAgICAgICAgPHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD54bXAuZGlkOjc1Y2I1NDQxLTM4M2EtNDczNy05NWQ3LWI4NmQyZTJmYjEyNzwveG1wTU06T3JpZ2luYWxEb2N1bWVudElEPgogICAgICAgICA8eG1wTU06SGlzdG9yeT4KICAgICAgICAgICAgPHJkZjpTZXE+CiAgICAgICAgICAgICAgIDxyZGY6bGkgcmRmOnBhcnNlVHlwZT0iUmVzb3VyY2UiPgogICAgICAgICAgICAgICAgICA8c3RFdnQ6YWN0aW9uPmNyZWF0ZWQ8L3N0RXZ0OmFjdGlvbj4KICAgICAgICAgICAgICAgICAgPHN0RXZ0Omluc3RhbmNlSUQ+eG1wLmlpZDo3NWNiNTQ0MS0zODNhLTQ3MzctOTVkNy1iODZkMmUyZmIxMjc8L3N0RXZ0Omluc3RhbmNlSUQ+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDp3aGVuPjIwMTctMTEtMjBUMTY6NDU6NDYrMDk6MDA8L3N0RXZ0OndoZW4+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDpzb2Z0d2FyZUFnZW50PkFkb2JlIFBob3Rvc2hvcCBDQyAoTWFjaW50b3NoKTwvc3RFdnQ6c29mdHdhcmVBZ2VudD4KICAgICAgICAgICAgICAgPC9yZGY6bGk+CiAgICAgICAgICAgIDwvcmRmOlNlcT4KICAgICAgICAgPC94bXBNTTpIaXN0b3J5PgogICAgICAgICA8dGlmZjpPcmllbnRhdGlvbj4xPC90aWZmOk9yaWVudGF0aW9uPgogICAgICAgICA8dGlmZjpYUmVzb2x1dGlvbj43MjAwMDAvMTAwMDA8L3RpZmY6WFJlc29sdXRpb24+CiAgICAgICAgIDx0aWZmOllSZXNvbHV0aW9uPjcyMDAwMC8xMDAwMDwvdGlmZjpZUmVzb2x1dGlvbj4KICAgICAgICAgPHRpZmY6UmVzb2x1dGlvblVuaXQ+MjwvdGlmZjpSZXNvbHV0aW9uVW5pdD4KICAgICAgICAgPGV4aWY6Q29sb3JTcGFjZT42NTUzNTwvZXhpZjpDb2xvclNwYWNlPgogICAgICAgICA8ZXhpZjpQaXhlbFhEaW1lbnNpb24+MzgwPC9leGlmOlBpeGVsWERpbWVuc2lvbj4KICAgICAgICAgPGV4aWY6UGl4ZWxZRGltZW5zaW9uPjI4MjwvZXhpZjpQaXhlbFlEaW1lbnNpb24+CiAgICAgIDwvcmRmOkRlc2NyaXB0aW9uPgogICA8L3JkZjpSREY+CjwveDp4bXBtZXRhPgogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAg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

    :param pos: posj - Target joints position [deg].
    :param vel: float or float[6] - velocity (same to all axes) or velocity (to each axis) [deg/s].
    :param acc: float or float[6] - acceleration (same to all axes) or acceleration (to each axis) [deg/s^2].
    :param time: float - Reach time [sec].
    :param radius: float - Radius for Blending [mm].
    :param mod: int - Movement basis (DR_MV_MOD_ABS: Absolute, DR_MV_MOD_REL: Relative). Default: DR_MV_MOD_ABS.
    :param ra: int - Reactive motion mode (DR_MV_RA_DUPLICATE: duplicate, DR_MV_RA_OVERRIDE: override)
    :param v: float or float[6] - velocity (same to all axes) or velocity (to each axis) [deg/s].
    :param a: float or float[6] - acceleration (same to all axes) or acceleration (to each axis) [deg/s^2].
    :param t: float - Reach time [sec].
    :param r: float - Radius for Blending [mm].

    :return: int - (0 -> Success, Negative value -> Error)
    """
    global _robodk_plugin_async
    global _robodk_control_space

    if _robodk_plugin_RDK is not None:

        # joint velocity
        if vel is not None:
            _robodk_plugin_robot.setSpeedJoints(vel)
        elif v is not None:
            _robodk_plugin_robot.setSpeedJoints(v)
        elif _robodk_plugin_j_vel is not None:
            _robodk_plugin_robot.setSpeedJoints(_robodk_plugin_j_vel)

        # joint acceleration
        if acc is not None:
            _robodk_plugin_robot.setAccelerationJoints(acc)
        elif a is not None:
            _robodk_plugin_robot.setAccelerationJoints(a)
        elif _robodk_plugin_j_acc is not None:
            _robodk_plugin_robot.setAccelerationJoints(_robodk_plugin_j_acc)

        # blending radius
        if radius is not None:
            _robodk_plugin_robot.setRounding(radius)
        elif r is not None:
            _robodk_plugin_robot.setRounding(r)
        else:
            _robodk_plugin_robot.setRounding(_robodk_plugin_r)

        global _robodk_plugin_async
        if _robodk_plugin_robot.Busy() and _robodk_plugin_async:
            _robodk_plugin_robot.Stop()

        _robodk_control_space = _ROBODK_JOINT_SPACE_CONTROL
        _robodk_plugin_async = False
        _robodk_plugin_robot.MoveJ(pos)
    return 0


def amovej(pos, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None):
    """
    The asynchronous movej motion operates in the same way as movej except that it does not have the radius  parameter for blending.
    The command is the asynchronous motion command, and the next command is executed at the same time the motion begins.

    .. image:: data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAXwAAAEaCAYAAAASSuyNAAAACXBIWXMAAAsTAAALEwEAmpwYAAA4IGlUWHRYTUw6Y29tLmFkb2JlLnhtcAAAAAAAPD94cGFja2V0IGJlZ2luPSLvu78iIGlkPSJXNU0wTXBDZWhpSHpyZVN6TlRjemtjOWQiPz4KPHg6eG1wbWV0YSB4bWxuczp4PSJhZG9iZTpuczptZXRhLyIgeDp4bXB0az0iQWRvYmUgWE1QIENvcmUgNS41LWMwMjEgNzkuMTU0OTExLCAyMDEzLzEwLzI5LTExOjQ3OjE2ICAgICAgICAiPgogICA8cmRmOlJERiB4bWxuczpyZGY9Imh0dHA6Ly93d3cudzMub3JnLzE5OTkvMDIvMjItcmRmLXN5bnRheC1ucyMiPgogICAgICA8cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0iIgogICAgICAgICAgICB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmRjPSJodHRwOi8vcHVybC5vcmcvZGMvZWxlbWVudHMvMS4xLyIKICAgICAgICAgICAgeG1sbnM6cGhvdG9zaG9wPSJodHRwOi8vbnMuYWRvYmUuY29tL3Bob3Rvc2hvcC8xLjAvIgogICAgICAgICAgICB4bWxuczp4bXBNTT0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wL21tLyIKICAgICAgICAgICAgeG1sbnM6c3RFdnQ9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9zVHlwZS9SZXNvdXJjZUV2ZW50IyIKICAgICAgICAgICAgeG1sbnM6dGlmZj0iaHR0cDovL25zLmFkb2JlLmNvbS90aWZmLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmV4aWY9Imh0dHA6Ly9ucy5hZG9iZS5jb20vZXhpZi8xLjAvIj4KICAgICAgICAgPHhtcDpDcmVhdG9yVG9vbD5BZG9iZSBQaG90b3Nob3AgQ0MgKE1hY2ludG9zaCk8L3htcDpDcmVhdG9yVG9vbD4KICAgICAgICAgPHhtcDpDcmVhdGVEYXRlPjIwMTctMTEtMjBUMTY6NDU6NDYrMDk6MDA8L3htcDpDcmVhdGVEYXRlPgogICAgICAgICA8eG1wOk1vZGlmeURhdGU+MjAxNy0xMi0xM1QxNzo1MDoxNSswOTowMDwveG1wOk1vZGlmeURhdGU+CiAgICAgICAgIDx4bXA6TWV0YWRhdGFEYXRlPjIwMTctMTItMTNUMTc6NTA6MTUrMDk6MDA8L3htcDpNZXRhZGF0YURhdGU+CiAgICAgICAgIDxkYzpmb3JtYXQ+aW1hZ2UvcG5nPC9kYzpmb3JtYXQ+CiAgICAgICAgIDxwaG90b3Nob3A6Q29sb3JNb2RlPjM8L3Bob3Rvc2hvcDpDb2xvck1vZGU+CiAgICAgICAgIDx4bXBNTTpJbnN0YW5jZUlEPnhtcC5paWQ6NzVjYjU0NDEtMzgzYS00NzM3LTk1ZDctYjg2ZDJlMmZiMTI3PC94bXBNTTpJbnN0YW5jZUlEPgogICAgICAgICA8eG1wTU06RG9jdW1lbnRJRD54bXAuZGlkOjc1Y2I1NDQxLTM4M2EtNDczNy05NWQ3LWI4NmQyZTJmYjEyNzwveG1wTU06RG9jdW1lbnRJRD4KICAgICAgICAgPHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD54bXAuZGlkOjc1Y2I1NDQxLTM4M2EtNDczNy05NWQ3LWI4NmQyZTJmYjEyNzwveG1wTU06T3JpZ2luYWxEb2N1bWVudElEPgogICAgICAgICA8eG1wTU06SGlzdG9yeT4KICAgICAgICAgICAgPHJkZjpTZXE+CiAgICAgICAgICAgICAgIDxyZGY6bGkgcmRmOnBhcnNlVHlwZT0iUmVzb3VyY2UiPgogICAgICAgICAgICAgICAgICA8c3RFdnQ6YWN0aW9uPmNyZWF0ZWQ8L3N0RXZ0OmFjdGlvbj4KICAgICAgICAgICAgICAgICAgPHN0RXZ0Omluc3RhbmNlSUQ+eG1wLmlpZDo3NWNiNTQ0MS0zODNhLTQ3MzctOTVkNy1iODZkMmUyZmIxMjc8L3N0RXZ0Omluc3RhbmNlSUQ+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDp3aGVuPjIwMTctMTEtMjBUMTY6NDU6NDYrMDk6MDA8L3N0RXZ0OndoZW4+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDpzb2Z0d2FyZUFnZW50PkFkb2JlIFBob3Rvc2hvcCBDQyAoTWFjaW50b3NoKTwvc3RFdnQ6c29mdHdhcmVBZ2VudD4KICAgICAgICAgICAgICAgPC9yZGY6bGk+CiAgICAgICAgICAgIDwvcmRmOlNlcT4KICAgICAgICAgPC94bXBNTTpIaXN0b3J5PgogICAgICAgICA8dGlmZjpPcmllbnRhdGlvbj4xPC90aWZmOk9yaWVudGF0aW9uPgogICAgICAgICA8dGlmZjpYUmVzb2x1dGlvbj43MjAwMDAvMTAwMDA8L3RpZmY6WFJlc29sdXRpb24+CiAgICAgICAgIDx0aWZmOllSZXNvbHV0aW9uPjcyMDAwMC8xMDAwMDwvdGlmZjpZUmVzb2x1dGlvbj4KICAgICAgICAgPHRpZmY6UmVzb2x1dGlvblVuaXQ+MjwvdGlmZjpSZXNvbHV0aW9uVW5pdD4KICAgICAgICAgPGV4aWY6Q29sb3JTcGFjZT42NTUzNTwvZXhpZjpDb2xvclNwYWNlPgogICAgICAgICA8ZXhpZjpQaXhlbFhEaW1lbnNpb24+MzgwPC9leGlmOlBpeGVsWERpbWVuc2lvbj4KICAgICAgICAgPGV4aWY6UGl4ZWxZRGltZW5zaW9uPjI4MjwvZXhpZjpQaXhlbFlEaW1lbnNpb24+CiAgICAgIDwvcmRmOkRlc2NyaXB0aW9uPgogICA8L3JkZjpSREY+CjwveDp4bXBtZXRhPgogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAg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

    :param pos: posj - Target joints position [deg].
    :param vel: float or float[6] - velocity (same to all axes) or velocity (to each axis) [deg/s].
    :param acc: float or float[6] - acceleration (same to all axes) or acceleration (to each axis) [deg/s^2].
    :param time: float - Reach time [sec].
    :param mod: int - Movement basis (DR_MV_MOD_ABS: Absolute, DR_MV_MOD_REL: Relative). Default: DR_MV_MOD_ABS.
    :param ra: int - Reactive motion mode (DR_MV_RA_DUPLICATE: duplicate, DR_MV_RA_OVERRIDE: override)
    :param v: float or float[6] - velocity (same to all axes) or velocity (to each axis) [deg/s].
    :param a: float or float[6] - acceleration (same to all axes) or acceleration (to each axis) [deg/s^2].
    :param t: float - Reach time [sec].

    :return: int - (0 -> Success, Negative value -> Error)
    """

    global _robodk_plugin_async
    global _robodk_control_space

    if _robodk_plugin_RDK is not None:

        # joint velocity
        if vel is not None:
            _robodk_plugin_robot.setSpeedJoints(vel)
        elif v is not None:
            _robodk_plugin_robot.setSpeedJoints(v)
        elif _robodk_plugin_j_vel is not None:
            _robodk_plugin_robot.setSpeedJoints(_robodk_plugin_j_vel)

        # joint acceleration
        if acc is not None:
            _robodk_plugin_robot.setAccelerationJoints(acc)
        elif a is not None:
            _robodk_plugin_robot.setAccelerationJoints(a)
        elif _robodk_plugin_j_acc is not None:
            _robodk_plugin_robot.setAccelerationJoints(_robodk_plugin_j_acc)

        if _robodk_plugin_robot.Busy() and _robodk_plugin_async:
            _robodk_plugin_robot.Stop()

        _robodk_plugin_async = True
        _robodk_control_space = _ROBODK_JOINT_SPACE_CONTROL
        _robodk_plugin_robot.MoveJ(pos, blocking=False)
    return 0


def movejx(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None, r=None):
    """
    The robot moves to the target position (pos) within the joint space.
    Since the target position is inputted as a posx form in the task space, it moves in the same way as movel.
    However, since this robot motion is performed in the joint space, it does not guarantee a linear path to the target position.
    In addition, one of 8 types of joint combination (robot configurations) corresponding to the task space coordinate system (posx)
    must be specified in sol (solution space).

    :param pos: posx - Task space position.
    :param vel: float or float[6] - velocity (same to all axes) or velocity (to each axis) [deg/s].
    :param acc: float or float[6] - acceleration (same to all axes) or acceleration (to each axis) [deg/s^2].
    :param time: float - Reach time [sec].
    :param radius: float - Radius for Blending [mm].
    :param mod: int - Movement basis (DR_MV_MOD_ABS: Absolute, DR_MV_MOD_REL: Relative). Default: DR_MV_MOD_ABS.
    :param ra: int - Reactive motion mode (DR_MV_RA_DUPLICATE: duplicate, DR_MV_RA_OVERRIDE: override)
    :param v: float or float[6] - velocity (same to all axes) or velocity (to each axis) [deg/s].
    :param a: float or float[6] - acceleration (same to all axes) or acceleration (to each axis) [deg/s^2].
    :param t: float - Reach time [sec].
    :param r: float - Radius for Blending [mm].
    :param ref: reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference). Default: DR_BASE.
    :param sol: int - Solution space.

    :return: int - (0 -> Success, Negative value -> Error)
    """
    global _robodk_plugin_async
    global _robodk_control_space

    if _robodk_plugin_RDK is not None:

        # joint velocity
        if vel is not None:
            _robodk_plugin_robot.setSpeedJoints(vel)
        elif v is not None:
            _robodk_plugin_robot.setSpeedJoints(v)
        elif _robodk_plugin_j_vel is not None:
            _robodk_plugin_robot.setSpeedJoints(_robodk_plugin_j_vel)

        # joint acceleration
        if acc is not None:
            _robodk_plugin_robot.setAccelerationJoints(acc)
        elif a is not None:
            _robodk_plugin_robot.setAccelerationJoints(a)
        elif _robodk_plugin_j_acc is not None:
            _robodk_plugin_robot.setAccelerationJoints(_robodk_plugin_j_acc)

        # blending radius
        if radius is not None:
            _robodk_plugin_robot.setRounding(radius)
        elif r is not None:
            _robodk_plugin_robot.setRounding(r)
        else:
            _robodk_plugin_robot.setRounding(_robodk_plugin_r)

        # reference frame
        if ref is None:
            ref = _robodk_plugin_ref

        global _robodk_plugin_async
        if _robodk_plugin_robot.Busy() and _robodk_plugin_async:
            _robodk_plugin_robot.Stop()

        pos, _ = ikin(pos, sol, ref)

        _robodk_control_space = _ROBODK_JOINT_SPACE_CONTROL
        _robodk_plugin_async = False
        _robodk_plugin_robot.MoveJ(pos)
    return 0

def amovejx(pos, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, sol=0, v=None, a=None, t=None):
    """
    The asynchronous movejx motion operates in the same way as movejx except that it does not have the radius  parameter for  blending.
    The command  is  the  asynchronous  motion  command,  and  the  next command is executed without waiting for the motion to terminate.

    :param pos: posx - Task space position.
    :param vel: float or float[6] - velocity (same to all axes) or velocity (to each axis) [deg/s].
    :param acc: float or float[6] - acceleration (same to all axes) or acceleration (to each axis) [deg/s^2].
    :param time: float - Reach time [sec].
    :param mod: int - Movement basis (DR_MV_MOD_ABS: Absolute, DR_MV_MOD_REL: Relative). Default: DR_MV_MOD_ABS.
    :param ra: int - Reactive motion mode (DR_MV_RA_DUPLICATE: duplicate, DR_MV_RA_OVERRIDE: override)
    :param v: float or float[6] - velocity (same to all axes) or velocity (to each axis) [deg/s].
    :param a: float or float[6] - acceleration (same to all axes) or acceleration (to each axis) [deg/s^2].
    :param t: float - Reach time [sec].
    :param ref: reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference). Default: DR_BASE.
    :param sol: int - Solution space.

    :return: int - (0 -> Success, Negative value -> Error)
    """

    global _robodk_plugin_async
    global _robodk_control_space

    if _robodk_plugin_RDK is not None:

        # joint velocity
        if vel is not None:
            _robodk_plugin_robot.setSpeedJoints(vel)
        elif v is not None:
            _robodk_plugin_robot.setSpeedJoints(v)
        elif _robodk_plugin_j_vel is not None:
            _robodk_plugin_robot.setSpeedJoints(_robodk_plugin_j_vel)

        # joint acceleration
        if acc is not None:
            _robodk_plugin_robot.setAccelerationJoints(acc)
        elif a is not None:
            _robodk_plugin_robot.setAccelerationJoints(a)
        elif _robodk_plugin_j_acc is not None:
            _robodk_plugin_robot.setAccelerationJoints(_robodk_plugin_j_acc)

        # reference frame
        if ref is None:
            ref = _robodk_plugin_ref

        global _robodk_plugin_async
        if _robodk_plugin_robot.Busy() and _robodk_plugin_async:
            _robodk_plugin_robot.Stop()

        pos, _ = ikin(pos, sol, ref)

        _robodk_control_space = _ROBODK_JOINT_SPACE_CONTROL
        _robodk_plugin_async = True
        _robodk_plugin_robot.MoveJ(pos, blocking=True)
    return 0

def movel(pos, vel=None, acc=None, time=None, radius=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, app_type=DR_MV_APP_NONE) -> int:
    """
    The robot moves along the straight line to the target position (pos) within the task space.

    .. image:: data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAXwAAADJCAYAAAAgl4m4AAAACXBIWXMAAAsTAAALEwEAmpwYAAA4IGlUWHRYTUw6Y29tLmFkb2JlLnhtcAAAAAAAPD94cGFja2V0IGJlZ2luPSLvu78iIGlkPSJXNU0wTXBDZWhpSHpyZVN6TlRjemtjOWQiPz4KPHg6eG1wbWV0YSB4bWxuczp4PSJhZG9iZTpuczptZXRhLyIgeDp4bXB0az0iQWRvYmUgWE1QIENvcmUgNS41LWMwMjEgNzkuMTU0OTExLCAyMDEzLzEwLzI5LTExOjQ3OjE2ICAgICAgICAiPgogICA8cmRmOlJERiB4bWxuczpyZGY9Imh0dHA6Ly93d3cudzMub3JnLzE5OTkvMDIvMjItcmRmLXN5bnRheC1ucyMiPgogICAgICA8cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0iIgogICAgICAgICAgICB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmRjPSJodHRwOi8vcHVybC5vcmcvZGMvZWxlbWVudHMvMS4xLyIKICAgICAgICAgICAgeG1sbnM6cGhvdG9zaG9wPSJodHRwOi8vbnMuYWRvYmUuY29tL3Bob3Rvc2hvcC8xLjAvIgogICAgICAgICAgICB4bWxuczp4bXBNTT0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wL21tLyIKICAgICAgICAgICAgeG1sbnM6c3RFdnQ9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9zVHlwZS9SZXNvdXJjZUV2ZW50IyIKICAgICAgICAgICAgeG1sbnM6dGlmZj0iaHR0cDovL25zLmFkb2JlLmNvbS90aWZmLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmV4aWY9Imh0dHA6Ly9ucy5hZG9iZS5jb20vZXhpZi8xLjAvIj4KICAgICAgICAgPHhtcDpDcmVhdG9yVG9vbD5BZG9iZSBQaG90b3Nob3AgQ0MgKE1hY2ludG9zaCk8L3htcDpDcmVhdG9yVG9vbD4KICAgICAgICAgPHhtcDpDcmVhdGVEYXRlPjIwMTctMTEtMjBUMTY6NDU6NDgrMDk6MDA8L3htcDpDcmVhdGVEYXRlPgogICAgICAgICA8eG1wOk1vZGlmeURhdGU+MjAxNy0xMi0xM1QxNzo0OToyMiswOTowMDwveG1wOk1vZGlmeURhdGU+CiAgICAgICAgIDx4bXA6TWV0YWRhdGFEYXRlPjIwMTctMTItMTNUMTc6NDk6MjIrMDk6MDA8L3htcDpNZXRhZGF0YURhdGU+CiAgICAgICAgIDxkYzpmb3JtYXQ+aW1hZ2UvcG5nPC9kYzpmb3JtYXQ+CiAgICAgICAgIDxwaG90b3Nob3A6Q29sb3JNb2RlPjM8L3Bob3Rvc2hvcDpDb2xvck1vZGU+CiAgICAgICAgIDx4bXBNTTpJbnN0YW5jZUlEPnhtcC5paWQ6NTE0N2RiNjMtNGZhZi00ZWE3LWEzNWYtZjRkY2Y5NDA2Njc2PC94bXBNTTpJbnN0YW5jZUlEPgogICAgICAgICA8eG1wTU06RG9jdW1lbnRJRD54bXAuZGlkOjUxNDdkYjYzLTRmYWYtNGVhNy1hMzVmLWY0ZGNmOTQwNjY3NjwveG1wTU06RG9jdW1lbnRJRD4KICAgICAgICAgPHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD54bXAuZGlkOjUxNDdkYjYzLTRmYWYtNGVhNy1hMzVmLWY0ZGNmOTQwNjY3NjwveG1wTU06T3JpZ2luYWxEb2N1bWVudElEPgogICAgICAgICA8eG1wTU06SGlzdG9yeT4KICAgICAgICAgICAgPHJkZjpTZXE+CiAgICAgICAgICAgICAgIDxyZGY6bGkgcmRmOnBhcnNlVHlwZT0iUmVzb3VyY2UiPgogICAgICAgICAgICAgICAgICA8c3RFdnQ6YWN0aW9uPmNyZWF0ZWQ8L3N0RXZ0OmFjdGlvbj4KICAgICAgICAgICAgICAgICAgPHN0RXZ0Omluc3RhbmNlSUQ+eG1wLmlpZDo1MTQ3ZGI2My00ZmFmLTRlYTctYTM1Zi1mNGRjZjk0MDY2NzY8L3N0RXZ0Omluc3RhbmNlSUQ+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDp3aGVuPjIwMTctMTEtMjBUMTY6NDU6NDgrMDk6MDA8L3N0RXZ0OndoZW4+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDpzb2Z0d2FyZUFnZW50PkFkb2JlIFBob3Rvc2hvcCBDQyAoTWFjaW50b3NoKTwvc3RFdnQ6c29mdHdhcmVBZ2VudD4KICAgICAgICAgICAgICAgPC9yZGY6bGk+CiAgICAgICAgICAgIDwvcmRmOlNlcT4KICAgICAgICAgPC94bXBNTTpIaXN0b3J5PgogICAgICAgICA8dGlmZjpPcmllbnRhdGlvbj4xPC90aWZmOk9yaWVudGF0aW9uPgogICAgICAgICA8dGlmZjpYUmVzb2x1dGlvbj43MjAwMDAvMTAwMDA8L3RpZmY6WFJlc29sdXRpb24+CiAgICAgICAgIDx0aWZmOllSZXNvbHV0aW9uPjcyMDAwMC8xMDAwMDwvdGlmZjpZUmVzb2x1dGlvbj4KICAgICAgICAgPHRpZmY6UmVzb2x1dGlvblVuaXQ+MjwvdGlmZjpSZXNvbHV0aW9uVW5pdD4KICAgICAgICAgPGV4aWY6Q29sb3JTcGFjZT42NTUzNTwvZXhpZjpDb2xvclNwYWNlPgogICAgICAgICA8ZXhpZjpQaXhlbFhEaW1lbnNpb24+MzgwPC9leGlmOlBpeGVsWERpbWVuc2lvbj4KICAgICAgICAgPGV4aWY6UGl4ZWxZRGltZW5zaW9uPjIwMTwvZXhpZjpQaXhlbFlEaW1lbnNpb24+CiAgICAgIDwvcmRmOkRlc2NyaXB0aW9uPgogICA8L3JkZjpSREY+CjwveDp4bXBtZXRhPgogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAg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MO/oATI5rjE5/vFubA4Ogzw0nAqFAsFgkHvuuYcLL7zwhIJrJCkWi0QiETweD7FYjAULFpBKpfj3f/939uzZQ19fX/kTS2trK3/4wx94//338Xg8nHPOOSxevPiITwHbt29n7dq17Ny5E13XOe+885g3b96QklYymWTdunVs2rSJdDrN9OnTufLKK49b9orFYnzrW98C4Gc/+xlbt249PRdHiQExXkT9GiGPQmuyQOAEs7tgQ3NM/0TLIw/kXW7/WZIb5+nctjBwxtvhgw8+4Nvf/jaPPPLIkMevv/56HnjgAS699NJRF/ZQGvPvOM6QxwYvAMVikUKhNK9i8+bNPPDAA6xevbq84cqvfvUrHnzwQXbt2lX+t++99x4PPvggb731FvX19YTDYZ544gkee+yx8vckEgm+//3v8/jjj5PL5airq2PDhg3cf//9bNy4ccS1kfTwxbgr63yQKHBW1YltWViwoSHkYV+6dKGYEj25rQ4PpBzu/EWSv1wcZMlZZ3abxN7eXh544AG6uro4++yzueKKK8bUsR0cw3+ozZs3k8/nmTJlChMmTMBxHH7+85+TTqe54ooruPvuu7Esix//+Mds2LCBxx9/nO9973tomsaqVatIp9N8+ctf5vrrrwdKK4IeOpro6aefZuvWrSxdupS77roLTdNYv349P/rRj3jxxRe58MILT/hiJYEvxDCbHvOzP2PSnbOp1EvLKRz3jXjwT8ir0ZoqnlTg70863PubFHd8KnBGwr63t5dQKEQgECiHytVXX825555LQ0PD2PsEF42yY8cOoFRP37lzJ5s3b8YwDC6++GIAXnnlFXp6epg4cSLXXHNNKQQ9Hi677DJ27NhBR0cHb7zxBhdddFH5a52dnaTTacLhMJdffvmHF/MDB9iyZQvTpk1j+fLl5Xscl1xyCVu2bGHjxo20tLQwbdo06eELcYY++FMT8HAgU6QhFCjvXXs8OQsmRXTeMawTnnnbm3X4xq+SLJunc+O801ceOXDgALt27WLDhg1s2rSJv/u7vyuPlqmqquJzn/vcmD2yVVVVbN68mWeeeQav14uu68yYMYNbb721HNR79+4ll8vR2NjI5MmTy/+2sbGRaDRKMpmko6MDgFtvvZWOjg5eeukl3n77bS688EIWLVpUnv3b1tZGLpdj0qRJvPLKK2iahmmahEIhMpkMuVyO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

    :param pos: posx - Task space position.
    :param vel: float or float[2] - linear velocity or [linear velocity, angular velocity].
    :param acc: float or float[2] - linear acceleration or [linear acceleration, angular acceleration].
    :param time: float - Reach time [sec].
    :param radius: float - Radius for Blending [mm].
    :param ref: reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference). Default: DR_BASE.
    :param mod: int - Movement basis (DR_MV_MOD_ABS: Absolute, DR_MV_MOD_REL: Relative). Default: DR_MV_MOD_ABS.
    :param ra: int - Reactive motion mode (DR_MV_RA_DUPLICATE: duplicate, DR_MV_RA_OVERRIDE: override)
    :param v: float or float[2] - linear velocity or [linear velocity, angular velocity].
    :param a: float or float[2] - linear acceleration or [linear acceleration, angular acceleration].
    :param t: float - Reach time [sec].
    :param r: float - Radius for Blending [mm].

    :return: int - (0 -> Success, Negative value -> Error)
    """
    global _robodk_plugin_async
    global _robodk_control_space

    if _robodk_plugin_RDK is not None:

        # linear velocity
        linear_vel = get_param(vel, v)
        if linear_vel is None:
            if _robodk_plugin_vel is not None:
                linear_vel = _robodk_plugin_vel
            else:
                linear_vel = -1
        else:
            if type(linear_vel) == list:
                linear_vel = linear_vel[0]

        # linear acceleration
        linear_acc = get_param(acc, a)
        if linear_acc is None:
            if _robodk_plugin_acc is not None:
                linear_acc = _robodk_plugin_acc
            else:
                linear_acc = -1
        else:
            if type(linear_acc) == list:
                linear_acc = linear_acc[0]

        _robodk_plugin_robot.setSpeed(linear_vel, accel_linear=linear_acc)

        # blending radius
        if radius is not None:
            _robodk_plugin_robot.setRounding(radius)
        elif r is not None:
            _robodk_plugin_robot.setRounding(r)
        else:
            _robodk_plugin_robot.setRounding(_robodk_plugin_r)

        # reference frame
        if ref is None:
            ref = _robodk_plugin_ref

        ref_frame = _robodk_plugin_get_ref_frame(ref)
        _robodk_plugin_robot.setPoseFrame(ref_frame)

        if mod == DR_FC_MOD_REL:
            current_pose = _robodk_plugin_robot.Pose()
            current_pose_doosan = Pose_2_Comau(current_pose)
            pos = add_pose(current_pose_doosan, pos)

        p = Comau_2_Pose(pos)

        if _robodk_plugin_robot.Busy() and _robodk_plugin_async:
            _robodk_plugin_robot.Stop()

        _robodk_plugin_async = False
        _robodk_control_space = _ROBODK_TASK_SPACE_CONTROL
        _robodk_plugin_robot.MoveL(p, blocking=True)
    return 0


def amovel(pos, vel=None, acc=None, time=None, ref=None, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, app_type=DR_MV_APP_NONE) -> int:
    """
    The asynchronous movel motion operates in the same way as movel except that it does not have the radius  parameter  for  blending.
    The command is the asynchronous motion command, and the next command is executed without waiting for the motion to terminate.

    .. image:: data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAXwAAADJCAYAAAAgl4m4AAAACXBIWXMAAAsTAAALEwEAmpwYAAA4IGlUWHRYTUw6Y29tLmFkb2JlLnhtcAAAAAAAPD94cGFja2V0IGJlZ2luPSLvu78iIGlkPSJXNU0wTXBDZWhpSHpyZVN6TlRjemtjOWQiPz4KPHg6eG1wbWV0YSB4bWxuczp4PSJhZG9iZTpuczptZXRhLyIgeDp4bXB0az0iQWRvYmUgWE1QIENvcmUgNS41LWMwMjEgNzkuMTU0OTExLCAyMDEzLzEwLzI5LTExOjQ3OjE2ICAgICAgICAiPgogICA8cmRmOlJERiB4bWxuczpyZGY9Imh0dHA6Ly93d3cudzMub3JnLzE5OTkvMDIvMjItcmRmLXN5bnRheC1ucyMiPgogICAgICA8cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0iIgogICAgICAgICAgICB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmRjPSJodHRwOi8vcHVybC5vcmcvZGMvZWxlbWVudHMvMS4xLyIKICAgICAgICAgICAgeG1sbnM6cGhvdG9zaG9wPSJodHRwOi8vbnMuYWRvYmUuY29tL3Bob3Rvc2hvcC8xLjAvIgogICAgICAgICAgICB4bWxuczp4bXBNTT0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wL21tLyIKICAgICAgICAgICAgeG1sbnM6c3RFdnQ9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9zVHlwZS9SZXNvdXJjZUV2ZW50IyIKICAgICAgICAgICAgeG1sbnM6dGlmZj0iaHR0cDovL25zLmFkb2JlLmNvbS90aWZmLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmV4aWY9Imh0dHA6Ly9ucy5hZG9iZS5jb20vZXhpZi8xLjAvIj4KICAgICAgICAgPHhtcDpDcmVhdG9yVG9vbD5BZG9iZSBQaG90b3Nob3AgQ0MgKE1hY2ludG9zaCk8L3htcDpDcmVhdG9yVG9vbD4KICAgICAgICAgPHhtcDpDcmVhdGVEYXRlPjIwMTctMTEtMjBUMTY6NDU6NDgrMDk6MDA8L3htcDpDcmVhdGVEYXRlPgogICAgICAgICA8eG1wOk1vZGlmeURhdGU+MjAxNy0xMi0xM1QxNzo0OToyMiswOTowMDwveG1wOk1vZGlmeURhdGU+CiAgICAgICAgIDx4bXA6TWV0YWRhdGFEYXRlPjIwMTctMTItMTNUMTc6NDk6MjIrMDk6MDA8L3htcDpNZXRhZGF0YURhdGU+CiAgICAgICAgIDxkYzpmb3JtYXQ+aW1hZ2UvcG5nPC9kYzpmb3JtYXQ+CiAgICAgICAgIDxwaG90b3Nob3A6Q29sb3JNb2RlPjM8L3Bob3Rvc2hvcDpDb2xvck1vZGU+CiAgICAgICAgIDx4bXBNTTpJbnN0YW5jZUlEPnhtcC5paWQ6NTE0N2RiNjMtNGZhZi00ZWE3LWEzNWYtZjRkY2Y5NDA2Njc2PC94bXBNTTpJbnN0YW5jZUlEPgogICAgICAgICA8eG1wTU06RG9jdW1lbnRJRD54bXAuZGlkOjUxNDdkYjYzLTRmYWYtNGVhNy1hMzVmLWY0ZGNmOTQwNjY3NjwveG1wTU06RG9jdW1lbnRJRD4KICAgICAgICAgPHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD54bXAuZGlkOjUxNDdkYjYzLTRmYWYtNGVhNy1hMzVmLWY0ZGNmOTQwNjY3NjwveG1wTU06T3JpZ2luYWxEb2N1bWVudElEPgogICAgICAgICA8eG1wTU06SGlzdG9yeT4KICAgICAgICAgICAgPHJkZjpTZXE+CiAgICAgICAgICAgICAgIDxyZGY6bGkgcmRmOnBhcnNlVHlwZT0iUmVzb3VyY2UiPgogICAgICAgICAgICAgICAgICA8c3RFdnQ6YWN0aW9uPmNyZWF0ZWQ8L3N0RXZ0OmFjdGlvbj4KICAgICAgICAgICAgICAgICAgPHN0RXZ0Omluc3RhbmNlSUQ+eG1wLmlpZDo1MTQ3ZGI2My00ZmFmLTRlYTctYTM1Zi1mNGRjZjk0MDY2NzY8L3N0RXZ0Omluc3RhbmNlSUQ+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDp3aGVuPjIwMTctMTEtMjBUMTY6NDU6NDgrMDk6MDA8L3N0RXZ0OndoZW4+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDpzb2Z0d2FyZUFnZW50PkFkb2JlIFBob3Rvc2hvcCBDQyAoTWFjaW50b3NoKTwvc3RFdnQ6c29mdHdhcmVBZ2VudD4KICAgICAgICAgICAgICAgPC9yZGY6bGk+CiAgICAgICAgICAgIDwvcmRmOlNlcT4KICAgICAgICAgPC94bXBNTTpIaXN0b3J5PgogICAgICAgICA8dGlmZjpPcmllbnRhdGlvbj4xPC90aWZmOk9yaWVudGF0aW9uPgogICAgICAgICA8dGlmZjpYUmVzb2x1dGlvbj43MjAwMDAvMTAwMDA8L3RpZmY6WFJlc29sdXRpb24+CiAgICAgICAgIDx0aWZmOllSZXNvbHV0aW9uPjcyMDAwMC8xMDAwMDwvdGlmZjpZUmVzb2x1dGlvbj4KICAgICAgICAgPHRpZmY6UmVzb2x1dGlvblVuaXQ+MjwvdGlmZjpSZXNvbHV0aW9uVW5pdD4KICAgICAgICAgPGV4aWY6Q29sb3JTcGFjZT42NTUzNTwvZXhpZjpDb2xvclNwYWNlPgogICAgICAgICA8ZXhpZjpQaXhlbFhEaW1lbnNpb24+MzgwPC9leGlmOlBpeGVsWERpbWVuc2lvbj4KICAgICAgICAgPGV4aWY6UGl4ZWxZRGltZW5zaW9uPjIwMTwvZXhpZjpQaXhlbFlEaW1lbnNpb24+CiAgICAgIDwvcmRmOkRlc2NyaXB0aW9uPgogICA8L3JkZjpSREY+CjwveDp4bXBtZXRhPgogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAg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MO/oATI5rjE5/vFubA4Ogzw0nAqFAsFgkHvuuYcLL7zwhIJrJCkWi0QiETweD7FYjAULFpBKpfj3f/939uzZQ19fX/kTS2trK3/4wx94//338Xg8nHPOOSxevPiITwHbt29n7dq17Ny5E13XOe+885g3b96QklYymWTdunVs2rSJdDrN9OnTufLKK49b9orFYnzrW98C4Gc/+xlbt249PRdHiQExXkT9GiGPQmuyQOAEs7tgQ3NM/0TLIw/kXW7/WZIb5+nctjBwxtvhgw8+4Nvf/jaPPPLIkMevv/56HnjgAS699NJRF/ZQGvPvOM6QxwYvAMVikUKhNK9i8+bNPPDAA6xevbq84cqvfvUrHnzwQXbt2lX+t++99x4PPvggb731FvX19YTDYZ544gkee+yx8vckEgm+//3v8/jjj5PL5airq2PDhg3cf//9bNy4ccS1kfTwxbgr63yQKHBW1YltWViwoSHkYV+6dKGYEj25rQ4PpBzu/EWSv1wcZMlZZ3abxN7eXh544AG6uro4++yzueKKK8bUsR0cw3+ozZs3k8/nmTJlChMmTMBxHH7+85+TTqe54ooruPvuu7Esix//+Mds2LCBxx9/nO9973tomsaqVatIp9N8+ctf5vrrrwdKK4IeOpro6aefZuvWrSxdupS77roLTdNYv349P/rRj3jxxRe58MILT/hiJYEvxDCbHvOzP2PSnbOp1EvLKRz3jXjwT8ir0ZoqnlTg70863PubFHd8KnBGwr63t5dQKEQgECiHytVXX825555LQ0PD2PsEF42yY8cOoFRP37lzJ5s3b8YwDC6++GIAXnnlFXp6epg4cSLXXHNNKQQ9Hi677DJ27NhBR0cHb7zxBhdddFH5a52dnaTTacLhMJdffvmHF/MDB9iyZQvTpk1j+fLl5Xscl1xyCVu2bGHjxo20tLQwbdo06eELcYY++FMT8HAgU6QhFCjvXXs8OQsmRXTeMawTnnnbm3X4xq+SLJunc+O801ceOXDgALt27WLDhg1s2rSJv/u7vyuPlqmqquJzn/vcmD2yVVVVbN68mWeeeQav14uu68yYMYNbb721HNR79+4ll8vR2NjI5MmTy/+2sbGRaDRKMpmko6MDgFtvvZWOjg5eeukl3n77bS688EIWLVpUnv3b1tZGLpdj0qRJvPLKK2iahmmahEIhMpkMuVyO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

    :param pos: posx - Task space position.
    :param vel: float or float[2] - linear velocity or [linear velocity, angular velocity].
    :param acc: float or float[2] - linear acceleration or [linear acceleration, angular acceleration].
    :param time: float - Reach time [sec].
    :param ref: reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference). Default: DR_BASE.
    :param mod: int - Movement basis (DR_MV_MOD_ABS: Absolute, DR_MV_MOD_REL: Relative). Default: DR_MV_MOD_ABS.
    :param ra: int - Reactive motion mode (DR_MV_RA_DUPLICATE: duplicate, DR_MV_RA_OVERRIDE: override)
    :param v: float or float[2] - linear velocity or [linear velocity, angular velocity].
    :param a: float or float[2] - linear acceleration or [linear acceleration, angular acceleration].
    :param t: float - Reach time [sec].

    :return: int - (0 -> Success, Negative value -> Error)
    """

    global _robodk_plugin_async
    global _robodk_control_space

    if _robodk_plugin_RDK is not None:

        # linear velocity
        linear_vel = get_param(vel, v)
        if linear_vel is None:
            if _robodk_plugin_vel is not None:
                linear_vel = _robodk_plugin_vel
            else:
                linear_vel = -1
        else:
            if type(linear_vel) == list:
                linear_vel = linear_vel[0]

        # linear acceleration
        linear_acc = get_param(acc, a)
        if linear_acc is None:
            if _robodk_plugin_acc is not None:
                linear_acc = _robodk_plugin_acc
            else:
                linear_acc = -1
        else:
            if type(linear_acc) == list:
                linear_acc = linear_acc[0]

        _robodk_plugin_robot.setSpeed(linear_vel, accel_linear=linear_acc)

        # reference frame
        if ref is None:
            ref = _robodk_plugin_ref

        ref_frame = _robodk_plugin_get_ref_frame(ref)
        _robodk_plugin_robot.setPoseFrame(ref_frame)

        if mod == DR_FC_MOD_REL:
            current_pose = _robodk_plugin_robot.Pose()
            current_pose_doosan = Pose_2_Comau(current_pose)
            pos = add_pose(current_pose_doosan, pos)

        p = Comau_2_Pose(pos)


        if _robodk_plugin_robot.Busy() and _robodk_plugin_async:
            _robodk_plugin_robot.Stop()

        _robodk_plugin_async = True
        _robodk_control_space = _ROBODK_TASK_SPACE_CONTROL
        _robodk_plugin_robot.MoveL(p, blocking=False)
    return 0


def movec(pos1, pos2, vel=None, acc=None, time=None, radius=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, r=None, an=None, ori=DR_MV_ORI_TEACH) -> int:
    """
    The robot moves along an arc to the target pos (pos2) via a waypoint (pos1) or to a specified angle from the current position in the task space.

    .. image:: data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAXwAAAC5CAYAAAArtYR5AAAACXBIWXMAAAsTAAALEwEAmpwYAAA4IGlUWHRYTUw6Y29tLmFkb2JlLnhtcAAAAAAAPD94cGFja2V0IGJlZ2luPSLvu78iIGlkPSJXNU0wTXBDZWhpSHpyZVN6TlRjemtjOWQiPz4KPHg6eG1wbWV0YSB4bWxuczp4PSJhZG9iZTpuczptZXRhLyIgeDp4bXB0az0iQWRvYmUgWE1QIENvcmUgNS41LWMwMjEgNzkuMTU0OTExLCAyMDEzLzEwLzI5LTExOjQ3OjE2ICAgICAgICAiPgogICA8cmRmOlJERiB4bWxuczpyZGY9Imh0dHA6Ly93d3cudzMub3JnLzE5OTkvMDIvMjItcmRmLXN5bnRheC1ucyMiPgogICAgICA8cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0iIgogICAgICAgICAgICB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmRjPSJodHRwOi8vcHVybC5vcmcvZGMvZWxlbWVudHMvMS4xLyIKICAgICAgICAgICAgeG1sbnM6cGhvdG9zaG9wPSJodHRwOi8vbnMuYWRvYmUuY29tL3Bob3Rvc2hvcC8xLjAvIgogICAgICAgICAgICB4bWxuczp4bXBNTT0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wL21tLyIKICAgICAgICAgICAgeG1sbnM6c3RFdnQ9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9zVHlwZS9SZXNvdXJjZUV2ZW50IyIKICAgICAgICAgICAgeG1sbnM6dGlmZj0iaHR0cDovL25zLmFkb2JlLmNvbS90aWZmLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmV4aWY9Imh0dHA6Ly9ucy5hZG9iZS5jb20vZXhpZi8xLjAvIj4KICAgICAgICAgPHhtcDpDcmVhdG9yVG9vbD5BZG9iZSBQaG90b3Nob3AgQ0MgKE1hY2ludG9zaCk8L3htcDpDcmVhdG9yVG9vbD4KICAgICAgICAgPHhtcDpDcmVhdGVEYXRlPjIwMTctMTEtMjBUMTY6NDU6NDgrMDk6MDA8L3htcDpDcmVhdGVEYXRlPgogICAgICAgICA8eG1wOk1vZGlmeURhdGU+MjAxNy0xMi0xM1QxNzo1MjozMSswOTowMDwveG1wOk1vZGlmeURhdGU+CiAgICAgICAgIDx4bXA6TWV0YWRhdGFEYXRlPjIwMTctMTItMTNUMTc6NTI6MzErMDk6MDA8L3htcDpNZXRhZGF0YURhdGU+CiAgICAgICAgIDxkYzpmb3JtYXQ+aW1hZ2UvcG5nPC9kYzpmb3JtYXQ+CiAgICAgICAgIDxwaG90b3Nob3A6Q29sb3JNb2RlPjM8L3Bob3Rvc2hvcDpDb2xvck1vZGU+CiAgICAgICAgIDx4bXBNTTpJbnN0YW5jZUlEPnhtcC5paWQ6YmY5NDg1ODktMWJhMi00YmRlLWI3NWYtNGNiZDhhOTEyZGM4PC94bXBNTTpJbnN0YW5jZUlEPgogICAgICAgICA8eG1wTU06RG9jdW1lbnRJRD54bXAuZGlkOmJmOTQ4NTg5LTFiYTItNGJkZS1iNzVmLTRjYmQ4YTkxMmRjODwveG1wTU06RG9jdW1lbnRJRD4KICAgICAgICAgPHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD54bXAuZGlkOmJmOTQ4NTg5LTFiYTItNGJkZS1iNzVmLTRjYmQ4YTkxMmRjODwveG1wTU06T3JpZ2luYWxEb2N1bWVudElEPgogICAgICAgICA8eG1wTU06SGlzdG9yeT4KICAgICAgICAgICAgPHJkZjpTZXE+CiAgICAgICAgICAgICAgIDxyZGY6bGkgcmRmOnBhcnNlVHlwZT0iUmVzb3VyY2UiPgogICAgICAgICAgICAgICAgICA8c3RFdnQ6YWN0aW9uPmNyZWF0ZWQ8L3N0RXZ0OmFjdGlvbj4KICAgICAgICAgICAgICAgICAgPHN0RXZ0Omluc3RhbmNlSUQ+eG1wLmlpZDpiZjk0ODU4OS0xYmEyLTRiZGUtYjc1Zi00Y2JkOGE5MTJkYzg8L3N0RXZ0Omluc3RhbmNlSUQ+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDp3aGVuPjIwMTctMTEtMjBUMTY6NDU6NDgrMDk6MDA8L3N0RXZ0OndoZW4+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDpzb2Z0d2FyZUFnZW50PkFkb2JlIFBob3Rvc2hvcCBDQyAoTWFjaW50b3NoKTwvc3RFdnQ6c29mdHdhcmVBZ2VudD4KICAgICAgICAgICAgICAgPC9yZGY6bGk+CiAgICAgICAgICAgIDwvcmRmOlNlcT4KICAgICAgICAgPC94bXBNTTpIaXN0b3J5PgogICAgICAgICA8dGlmZjpPcmllbnRhdGlvbj4xPC90aWZmOk9yaWVudGF0aW9uPgogICAgICAgICA8dGlmZjpYUmVzb2x1dGlvbj43MjAwMDAvMTAwMDA8L3RpZmY6WFJlc29sdXRpb24+CiAgICAgICAgIDx0aWZmOllSZXNvbHV0aW9uPjcyMDAwMC8xMDAwMDwvdGlmZjpZUmVzb2x1dGlvbj4KICAgICAgICAgPHRpZmY6UmVzb2x1dGlvblVuaXQ+MjwvdGlmZjpSZXNvbHV0aW9uVW5pdD4KICAgICAgICAgPGV4aWY6Q29sb3JTcGFjZT42NTUzNTwvZXhpZjpDb2xvclNwYWNlPgogICAgICAgICA8ZXhpZjpQaXhlbFhEaW1lbnNpb24+MzgwPC9leGlmOlBpeGVsWERpbWVuc2lvbj4KICAgICAgICAgPGV4aWY6UGl4ZWxZRGltZW5zaW9uPjE4NTwvZXhpZjpQaXhlbFlEaW1lbnNpb24+CiAgICAgIDwvcmRmOkRlc2NyaXB0aW9uPgogICA8L3JkZjpSREY+CjwveDp4bXBtZXRhPgogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAg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

    :param pos1: posx - Task space position 1.
    :param pos2: posx - Task space position 2.
    :param angle: float or float[2] - target angle.
    :param an: float or float[2] - target angle.
    :param vel: float or float[2] - linear velocity or [linear velocity, angular velocity].
    :param acc: float or float[2] - linear acceleration or [linear acceleration, angular acceleration].
    :param time: float - Reach time [sec].
    :param radius: float - Radius for Blending [mm].
    :param ref: reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference). Default: DR_BASE.
    :param mod: int - Movement basis (DR_MV_MOD_ABS: Absolute, DR_MV_MOD_REL: Relative). Default: DR_MV_MOD_ABS.
    :param ra: int - Reactive motion mode (DR_MV_RA_DUPLICATE: duplicate, DR_MV_RA_OVERRIDE: override)
    :param v: float or float[2] - linear velocity or [linear velocity, angular velocity].
    :param a: float or float[2] - linear acceleration or [linear acceleration, angular acceleration].
    :param t: float - Reach time [sec].
    :param r: float - Radius for Blending [mm].
    :param ori: int - Orientation mode: DR_MV_ORI_TEACH, DR_MV_ORI_FIXED, DR_MV_ORI_RADIAL.

    :return: int - (0 -> Success, Negative value -> Error)
    """

    global _robodk_plugin_async
    global _robodk_control_space

    if _robodk_plugin_RDK is not None:

        # linear velocity
        linear_vel = get_param(vel, v)
        if linear_vel is None:
            if _robodk_plugin_vel is not None:
                linear_vel = _robodk_plugin_vel
            else:
                linear_vel = -1
        else:
            if type(linear_vel) == list:
                linear_vel = linear_vel[0]

        # linear acceleration
        linear_acc = get_param(acc, a)
        if linear_acc is None:
            if _robodk_plugin_acc is not None:
                linear_acc = _robodk_plugin_acc
            else:
                linear_acc = -1
        else:
            if type(linear_acc) == list:
                linear_acc = linear_acc[0]

        _robodk_plugin_robot.setSpeed(linear_vel, accel_linear=linear_acc)

        # blending radius
        if radius is not None:
            _robodk_plugin_robot.setRounding(radius)
        elif r is not None:
            _robodk_plugin_robot.setRounding(r)
        else:
            _robodk_plugin_robot.setRounding(_robodk_plugin_r)

        # reference frame
        if ref is None:
            ref = _robodk_plugin_ref

        ref_frame = _robodk_plugin_get_ref_frame(ref)
        _robodk_plugin_robot.setPoseFrame(ref_frame)

        if mod == DR_FC_MOD_REL:
            current_pose = _robodk_plugin_robot.Pose()
            current_pose_doosan = Pose_2_Comau(current_pose)
            pos1 = add_pose(current_pose_doosan, pos1)
            pos2 = add_pose(current_pose_doosan, pos2)

        p1 = Comau_2_Pose(pos1)
        p2 = Comau_2_Pose(pos2)

        if _robodk_plugin_robot.Busy() and _robodk_plugin_async:
            _robodk_plugin_robot.Stop()

        _robodk_plugin_async = False
        _robodk_control_space = _ROBODK_TASK_SPACE_CONTROL
        _robodk_plugin_robot.MoveC(p1, p2, blocking=True)
    return 0


def amovec(pos1, pos2, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, angle=None, ra=DR_MV_RA_DUPLICATE, v=None, a=None, t=None, an=None) -> int:
    """
    The asynchronous movec motion operates in the same way as movec except that it does not have the radius  parameter for blending.
    The  command  is  an  asynchronous  motion  command,  and  the  next command is executed without waiting for the motion to terminate.

    .. image:: data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAXwAAAC5CAYAAAArtYR5AAAACXBIWXMAAAsTAAALEwEAmpwYAAA4IGlUWHRYTUw6Y29tLmFkb2JlLnhtcAAAAAAAPD94cGFja2V0IGJlZ2luPSLvu78iIGlkPSJXNU0wTXBDZWhpSHpyZVN6TlRjemtjOWQiPz4KPHg6eG1wbWV0YSB4bWxuczp4PSJhZG9iZTpuczptZXRhLyIgeDp4bXB0az0iQWRvYmUgWE1QIENvcmUgNS41LWMwMjEgNzkuMTU0OTExLCAyMDEzLzEwLzI5LTExOjQ3OjE2ICAgICAgICAiPgogICA8cmRmOlJERiB4bWxuczpyZGY9Imh0dHA6Ly93d3cudzMub3JnLzE5OTkvMDIvMjItcmRmLXN5bnRheC1ucyMiPgogICAgICA8cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0iIgogICAgICAgICAgICB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmRjPSJodHRwOi8vcHVybC5vcmcvZGMvZWxlbWVudHMvMS4xLyIKICAgICAgICAgICAgeG1sbnM6cGhvdG9zaG9wPSJodHRwOi8vbnMuYWRvYmUuY29tL3Bob3Rvc2hvcC8xLjAvIgogICAgICAgICAgICB4bWxuczp4bXBNTT0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wL21tLyIKICAgICAgICAgICAgeG1sbnM6c3RFdnQ9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9zVHlwZS9SZXNvdXJjZUV2ZW50IyIKICAgICAgICAgICAgeG1sbnM6dGlmZj0iaHR0cDovL25zLmFkb2JlLmNvbS90aWZmLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmV4aWY9Imh0dHA6Ly9ucy5hZG9iZS5jb20vZXhpZi8xLjAvIj4KICAgICAgICAgPHhtcDpDcmVhdG9yVG9vbD5BZG9iZSBQaG90b3Nob3AgQ0MgKE1hY2ludG9zaCk8L3htcDpDcmVhdG9yVG9vbD4KICAgICAgICAgPHhtcDpDcmVhdGVEYXRlPjIwMTctMTEtMjBUMTY6NDU6NDgrMDk6MDA8L3htcDpDcmVhdGVEYXRlPgogICAgICAgICA8eG1wOk1vZGlmeURhdGU+MjAxNy0xMi0xM1QxNzo1MjozMSswOTowMDwveG1wOk1vZGlmeURhdGU+CiAgICAgICAgIDx4bXA6TWV0YWRhdGFEYXRlPjIwMTctMTItMTNUMTc6NTI6MzErMDk6MDA8L3htcDpNZXRhZGF0YURhdGU+CiAgICAgICAgIDxkYzpmb3JtYXQ+aW1hZ2UvcG5nPC9kYzpmb3JtYXQ+CiAgICAgICAgIDxwaG90b3Nob3A6Q29sb3JNb2RlPjM8L3Bob3Rvc2hvcDpDb2xvck1vZGU+CiAgICAgICAgIDx4bXBNTTpJbnN0YW5jZUlEPnhtcC5paWQ6YmY5NDg1ODktMWJhMi00YmRlLWI3NWYtNGNiZDhhOTEyZGM4PC94bXBNTTpJbnN0YW5jZUlEPgogICAgICAgICA8eG1wTU06RG9jdW1lbnRJRD54bXAuZGlkOmJmOTQ4NTg5LTFiYTItNGJkZS1iNzVmLTRjYmQ4YTkxMmRjODwveG1wTU06RG9jdW1lbnRJRD4KICAgICAgICAgPHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD54bXAuZGlkOmJmOTQ4NTg5LTFiYTItNGJkZS1iNzVmLTRjYmQ4YTkxMmRjODwveG1wTU06T3JpZ2luYWxEb2N1bWVudElEPgogICAgICAgICA8eG1wTU06SGlzdG9yeT4KICAgICAgICAgICAgPHJkZjpTZXE+CiAgICAgICAgICAgICAgIDxyZGY6bGkgcmRmOnBhcnNlVHlwZT0iUmVzb3VyY2UiPgogICAgICAgICAgICAgICAgICA8c3RFdnQ6YWN0aW9uPmNyZWF0ZWQ8L3N0RXZ0OmFjdGlvbj4KICAgICAgICAgICAgICAgICAgPHN0RXZ0Omluc3RhbmNlSUQ+eG1wLmlpZDpiZjk0ODU4OS0xYmEyLTRiZGUtYjc1Zi00Y2JkOGE5MTJkYzg8L3N0RXZ0Omluc3RhbmNlSUQ+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDp3aGVuPjIwMTctMTEtMjBUMTY6NDU6NDgrMDk6MDA8L3N0RXZ0OndoZW4+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDpzb2Z0d2FyZUFnZW50PkFkb2JlIFBob3Rvc2hvcCBDQyAoTWFjaW50b3NoKTwvc3RFdnQ6c29mdHdhcmVBZ2VudD4KICAgICAgICAgICAgICAgPC9yZGY6bGk+CiAgICAgICAgICAgIDwvcmRmOlNlcT4KICAgICAgICAgPC94bXBNTTpIaXN0b3J5PgogICAgICAgICA8dGlmZjpPcmllbnRhdGlvbj4xPC90aWZmOk9yaWVudGF0aW9uPgogICAgICAgICA8dGlmZjpYUmVzb2x1dGlvbj43MjAwMDAvMTAwMDA8L3RpZmY6WFJlc29sdXRpb24+CiAgICAgICAgIDx0aWZmOllSZXNvbHV0aW9uPjcyMDAwMC8xMDAwMDwvdGlmZjpZUmVzb2x1dGlvbj4KICAgICAgICAgPHRpZmY6UmVzb2x1dGlvblVuaXQ+MjwvdGlmZjpSZXNvbHV0aW9uVW5pdD4KICAgICAgICAgPGV4aWY6Q29sb3JTcGFjZT42NTUzNTwvZXhpZjpDb2xvclNwYWNlPgogICAgICAgICA8ZXhpZjpQaXhlbFhEaW1lbnNpb24+MzgwPC9leGlmOlBpeGVsWERpbWVuc2lvbj4KICAgICAgICAgPGV4aWY6UGl4ZWxZRGltZW5zaW9uPjE4NTwvZXhpZjpQaXhlbFlEaW1lbnNpb24+CiAgICAgIDwvcmRmOkRlc2NyaXB0aW9uPgogICA8L3JkZjpSREY+CjwveDp4bXBtZXRhPgogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAg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

    :param pos1: posx - Task space position 1.
    :param pos2: posx - Task space position 2.
    :param angle: float or float[2] - target angle.
    :param an: float or float[2] - target angle.
    :param vel: float or float[2] - linear velocity or [linear velocity, angular velocity].
    :param acc: float or float[2] - linear acceleration or [linear acceleration, angular acceleration].
    :param time: float - Reach time [sec].
    :param ref: reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference). Default: DR_BASE.
    :param mod: int - Movement basis (DR_MV_MOD_ABS: Absolute, DR_MV_MOD_REL: Relative). Default: DR_MV_MOD_ABS.
    :param ra: int - Reactive motion mode (DR_MV_RA_DUPLICATE: duplicate, DR_MV_RA_OVERRIDE: override)
    :param v: float or float[2] - linear velocity or [linear velocity, angular velocity].
    :param a: float or float[2] - linear acceleration or [linear acceleration, angular acceleration].
    :param t: float - Reach time [sec].

    :return: int - (0 -> Success, Negative value -> Error)
    """

    global _robodk_plugin_async
    global _robodk_control_space

    if _robodk_plugin_RDK is not None:

        # linear velocity
        linear_vel = get_param(vel, v)
        if linear_vel is None:
            if _robodk_plugin_vel is not None:
                linear_vel = _robodk_plugin_vel
            else:
                linear_vel = -1
        else:
            if type(linear_vel) == list:
                linear_vel = linear_vel[0]

        # linear acceleration
        linear_acc = get_param(acc, a)
        if linear_acc is None:
            if _robodk_plugin_acc is not None:
                linear_acc = _robodk_plugin_j_acc
            else:
                linear_acc = -1
        else:
            if type(linear_acc) == list:
                linear_acc = linear_acc[0]

        _robodk_plugin_robot.setSpeed(linear_vel, accel_linear=linear_acc)

        # reference frame
        if ref is None:
            ref = _robodk_plugin_ref

        ref_frame = _robodk_plugin_get_ref_frame(ref)
        _robodk_plugin_robot.setPoseFrame(ref_frame)

        if mod == DR_FC_MOD_REL:
            current_pose = _robodk_plugin_robot.Pose()
            current_pose_doosan = Pose_2_Comau(current_pose)
            pos1 = add_pose(current_pose_doosan, pos1)
            pos2 = add_pose(current_pose_doosan, pos2)

        p1 = Comau_2_Pose(pos1)
        p2 = Comau_2_Pose(pos2)

        if _robodk_plugin_robot.Busy() and _robodk_plugin_async:
            _robodk_plugin_robot.Stop()

        _robodk_plugin_async = False
        _robodk_control_space = _ROBODK_JOINT_SPACE_CONTROL
        _robodk_plugin_robot.MoveC(p1, p2, blocking=False)
    return 0


def movesj(pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None) -> int:
    """
    The robot moves along a spline curve path that connects the current position to the target position
    (the last waypoint in pos_list) via the waypoints of the joint space input in pos_list.
    The  input  velocity/acceleration  means  the  maximum  velocity/acceleration  in  the  path,
    and  the acceleration and deceleration during the motion are determined according to the position of the waypoint.

    .. image:: data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAXwAAADOCAYAAAA9krkAAAAACXBIWXMAAAsTAAALEwEAmpwYAAA4IGlUWHRYTUw6Y29tLmFkb2JlLnhtcAAAAAAAPD94cGFja2V0IGJlZ2luPSLvu78iIGlkPSJXNU0wTXBDZWhpSHpyZVN6TlRjemtjOWQiPz4KPHg6eG1wbWV0YSB4bWxuczp4PSJhZG9iZTpuczptZXRhLyIgeDp4bXB0az0iQWRvYmUgWE1QIENvcmUgNS41LWMwMjEgNzkuMTU0OTExLCAyMDEzLzEwLzI5LTExOjQ3OjE2ICAgICAgICAiPgogICA8cmRmOlJERiB4bWxuczpyZGY9Imh0dHA6Ly93d3cudzMub3JnLzE5OTkvMDIvMjItcmRmLXN5bnRheC1ucyMiPgogICAgICA8cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0iIgogICAgICAgICAgICB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmRjPSJodHRwOi8vcHVybC5vcmcvZGMvZWxlbWVudHMvMS4xLyIKICAgICAgICAgICAgeG1sbnM6cGhvdG9zaG9wPSJodHRwOi8vbnMuYWRvYmUuY29tL3Bob3Rvc2hvcC8xLjAvIgogICAgICAgICAgICB4bWxuczp4bXBNTT0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wL21tLyIKICAgICAgICAgICAgeG1sbnM6c3RFdnQ9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9zVHlwZS9SZXNvdXJjZUV2ZW50IyIKICAgICAgICAgICAgeG1sbnM6dGlmZj0iaHR0cDovL25zLmFkb2JlLmNvbS90aWZmLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmV4aWY9Imh0dHA6Ly9ucy5hZG9iZS5jb20vZXhpZi8xLjAvIj4KICAgICAgICAgPHhtcDpDcmVhdG9yVG9vbD5BZG9iZSBQaG90b3Nob3AgQ0MgKE1hY2ludG9zaCk8L3htcDpDcmVhdG9yVG9vbD4KICAgICAgICAgPHhtcDpDcmVhdGVEYXRlPjIwMTctMTEtMjBUMTY6NDU6NDgrMDk6MDA8L3htcDpDcmVhdGVEYXRlPgogICAgICAgICA8eG1wOk1vZGlmeURhdGU+MjAxNy0xMi0xM1QxNzoxNzozMyswOTowMDwveG1wOk1vZGlmeURhdGU+CiAgICAgICAgIDx4bXA6TWV0YWRhdGFEYXRlPjIwMTctMTItMTNUMTc6MTc6MzMrMDk6MDA8L3htcDpNZXRhZGF0YURhdGU+CiAgICAgICAgIDxkYzpmb3JtYXQ+aW1hZ2UvcG5nPC9kYzpmb3JtYXQ+CiAgICAgICAgIDxwaG90b3Nob3A6Q29sb3JNb2RlPjM8L3Bob3Rvc2hvcDpDb2xvck1vZGU+CiAgICAgICAgIDx4bXBNTTpJbnN0YW5jZUlEPnhtcC5paWQ6ZGZmNGQ0ZGUtMWJhNS00MWMyLTgzYTUtOTNhYmQzNWI5MTNhPC94bXBNTTpJbnN0YW5jZUlEPgogICAgICAgICA8eG1wTU06RG9jdW1lbnRJRD54bXAuZGlkOmRmZjRkNGRlLTFiYTUtNDFjMi04M2E1LTkzYWJkMzViOTEzYTwveG1wTU06RG9jdW1lbnRJRD4KICAgICAgICAgPHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD54bXAuZGlkOmRmZjRkNGRlLTFiYTUtNDFjMi04M2E1LTkzYWJkMzViOTEzYTwveG1wTU06T3JpZ2luYWxEb2N1bWVudElEPgogICAgICAgICA8eG1wTU06SGlzdG9yeT4KICAgICAgICAgICAgPHJkZjpTZXE+CiAgICAgICAgICAgICAgIDxyZGY6bGkgcmRmOnBhcnNlVHlwZT0iUmVzb3VyY2UiPgogICAgICAgICAgICAgICAgICA8c3RFdnQ6YWN0aW9uPmNyZWF0ZWQ8L3N0RXZ0OmFjdGlvbj4KICAgICAgICAgICAgICAgICAgPHN0RXZ0Omluc3RhbmNlSUQ+eG1wLmlpZDpkZmY0ZDRkZS0xYmE1LTQxYzItODNhNS05M2FiZDM1YjkxM2E8L3N0RXZ0Omluc3RhbmNlSUQ+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDp3aGVuPjIwMTctMTEtMjBUMTY6NDU6NDgrMDk6MDA8L3N0RXZ0OndoZW4+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDpzb2Z0d2FyZUFnZW50PkFkb2JlIFBob3Rvc2hvcCBDQyAoTWFjaW50b3NoKTwvc3RFdnQ6c29mdHdhcmVBZ2VudD4KICAgICAgICAgICAgICAgPC9yZGY6bGk+CiAgICAgICAgICAgIDwvcmRmOlNlcT4KICAgICAgICAgPC94bXBNTTpIaXN0b3J5PgogICAgICAgICA8dGlmZjpPcmllbnRhdGlvbj4xPC90aWZmOk9yaWVudGF0aW9uPgogICAgICAgICA8dGlmZjpYUmVzb2x1dGlvbj43MjAwMDAvMTAwMDA8L3RpZmY6WFJlc29sdXRpb24+CiAgICAgICAgIDx0aWZmOllSZXNvbHV0aW9uPjcyMDAwMC8xMDAwMDwvdGlmZjpZUmVzb2x1dGlvbj4KICAgICAgICAgPHRpZmY6UmVzb2x1dGlvblVuaXQ+MjwvdGlmZjpSZXNvbHV0aW9uVW5pdD4KICAgICAgICAgPGV4aWY6Q29sb3JTcGFjZT42NTUzNTwvZXhpZjpDb2xvclNwYWNlPgogICAgICAgICA8ZXhpZjpQaXhlbFhEaW1lbnNpb24+MzgwPC9leGlmOlBpeGVsWERpbWVuc2lvbj4KICAgICAgICAgPGV4aWY6UGl4ZWxZRGltZW5zaW9uPjIwNjwvZXhpZjpQaXhlbFlEaW1lbnNpb24+CiAgICAgIDwvcmRmOkRlc2NyaXB0aW9uPgogICA8L3JkZjpSREY+CjwveDp4bXBtZXRhPgogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAg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q5kzZw4PP/wwBQUFvPnmm9x9990MGTKEAwcOcOutt7JkyRIuuOACFi9ezEUXXcTNN99cy10zefJkRo4cybBhw/jyyy95/fXXufjii+u13isqKli8eDHDhw9PfDnUR7pTYXALL0sKqumT4yXLZaEyykm3IEvih9W1EZXmqXbxUAvBFxxJ8DVN46OPPiIcDpOXl8dVV10FwIYNGzjllFMSlnvHjh3JyMhg9erVXHfddRQUFHDnnXdy5pln8n//939kZGRQUFBAWloaU6ZMIRqNUlpaSnV1tejoH+Hz+di2bRvr16/H4/Fw++23079/fxYvXkxycjIdOnQAICMjg7Zt27Jp0yYuuOACnnzySV555RVuueUWLrnkEs4//3x27tyJLMvs27ePd999l8LCQmKxGOXl5fUK/sSJE7FarYwYMeKo55nmVDirmZOvDwQ4q7mXVIdMIHZyWfqKBcrCOooEuV6buLmE4AsORzAYxG63M27cOHw+X602t9uN3++nqqqK1NRUqqurCQaDWCzx7+a//e1vXHXVVbz66quMHTuWl156ifT0dDZv3kyrVq2w2+307t37sBE/iqIgy3KTdOkcOHCA66+/nmHDhtX6yjEMg0gkQjgcBiAajRIIBBKul+7duzNx4kQWLFjAq6++imEYdOzYkWg0iqIodOrUiU6dOnHppZeSnZ1d57ivv/463333HU899dQRrftDaeG1E9FMVu2rYmDLJFxWiZB68oi+TQZ/VCPVoSD/jJMqKyvju+++Q1VVwuEwPp+Prl271vuyPJ6oqsq6des4cOAA2dnZnH766Ud0sf0URJSO4LA3naqqOByOOm1Dhw6lqqqKiRMn4vf7eeedd3A4HFx++eUsWrSId999l+zsbLp160ZxcTGyLHPeeedRWFiIJEn07t2bWCyWeEHUYBgGBw4cYOvWrZimSUVFBcXFxcRisSbR56ZpEolEEpOqh7q0evToQUpKChMnTuTgwYPMnj2bgoKCRNjqU089xYEDBxg0aBAul4tdu3bRunVr8vLy2LBhA926dSM3N5dQKITH46l13H//+998+OGHDBs2DLvdzurVqzlw4MAxnXO7FAfdMhx8vreSmGHissLJ8p42gYimk+n6eXbt3r17eemll5g9ezZffPEFEydO5IEHHmD37t0n9Lxfe+01nnzyST7//HOef/553nrrreO2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m9c65sCBAxk4cCAQz9b54IMPMmrUqMSCjQY5WIqCw+GgX79+XHXVVQC8+OKLvPPOO/Tq1YukpCQikQhLly4lFosxaNAgvF5vLVH77rvv6Nq1K6eddlri3w8ePMiiRYto164dvXv3rnPcXbt28f7773PzzTdz/vnnU1paetKL0k/F7XaTnZ3NqFGjyMjIYPv27YwdO5a5c+dy5ZVXArB9+3bWrFlD586da/VfIBBg8eLFmKZJv379yMjISLStWLGCgoICzjvvvHqj12ru6zvvvLPB9NWAAQMYMGCAUM+mIPg2m41HH330Zy07rvH1ZWRkkJmZSTQaBeCdd95h3rx5JCcnM2/ePP70pz8xcOBATNPklVdeYefOnSiKwpo1a3jwwQfxer3Mnj2bDz/8ELvdztKlS3nggQdo1arVYa1TTdO45JJLGvRg1ecrbdmyJStWrEhkIXz44YcpLi5GlmUWLFjA/fffT7NmzZg3bx4ffPABaWlpLF68mBtuuIGBAwdSUFDAk08+iaqqfPLJJ/zxj39kxIgRtY6xcuVKcnJyGDRoEFVVVfh8Pmw2W6N+MDIzM0lKSiIYDAKwaNEi3nrrLXw+H5988gnnnnsuN9xwA9FolOeff56ysjJUVeXbb7/loYceAuDf//43ixYtwul0smrVKh588MFaL4MawXc4HGzZsgWPx3NSGiSqqrJ3715yc3PFCvqm6NJxuVw/6YGXJAlZliktLQVg6tSpbN++nQsvvJBly5Yxffp0/v73vzNhwgQGDRrECy+8gKZpfPvtt3zwwQeMGjWKF198MSH2W7duZfr06TzwwANMmjSJvLw83nvvvXqPbZomX3/99WGz4zUkTNPEarUSCoWA+NzI9OnT6devH16vl6effpqioiL+9a9/8dZbbyHLMq+++iqhUIgZM2bQrl07nnzySV577TX69u0LwAsvvEDHjh2ZNGkSd999Nx9++CElJSV1PuFTU1OZOnUqd911F3fffTe7du1qVA+CJElIkkRZWRmmaTJ16lTKysq48MIL2bdvH//85z+56KKLmDBhAqNGjWLatGls27aNVatWsWTJEu644w5effVV7rjjjsRL8tNPP+WZZ57hjTfewOl0MnPmzDrHTUtLo7q6msmTJ3PPPfcc9j7+LZk2bRq33357nftC0PA4bguvjsTOnTvZtWsXW7ZsYebMmezevZvRo0dzxhln8M4775CVlcW1114LQFJSEgsXLiQ1NZV+/fpRXl7OBx98QGFhIaeeeipOp5OFCxeyb98+ZFlmx44dbN26ldLSUi644II6+bY3bdrE/Pnzufzyy8nMzGzQgxUIBNi0aRP79u3jo48+YtmyZXTv3p0777wTv9/PpEmT+P3vf0+PHj0AsFqtfPLJJwwYMICuXbvywQcf8OWXX9KsWTNycnL4/vvvWbRoEdnZ2RQUFLBlyxa+/fZbunbtWsvSXLNmDdu2bWP48OH8+c9/ZtmyZWzcuJEhQ4Y0mgdhy5Yt7Nmzhw0bNjBjxgzKysoYPXo0nTt35oMPPmD//v3ceuutuFwumjdvztq1a6mqquKSSy5h//79zJ49m9LSUrp3747VamXOnDmEQiFUVWXLli1s3rwZXddrJfWC+JL+wYMHM2zYMEzTZNq0aZx11lkkJyefFP3y5ptv8uGHH3LffffRpUsXoZgNm9hP8suEQiF27NhB+/btf1KEhmEYVFdXc8UVV9C+fXs8Hg9JSUlAvJzYoaXEVFVNRCwA3HXXXWzcuJH//Oc/3Hnnnbz44oskJSWhqip2ux3TNBk8eDAZGRkYhlEnpen27dux2+11fPwNldLSUnr27MmFF14IkCgIEQ6HkSSpVl/GYjF0XScUCtGzZ0/++c9/MnfuXJ555hlGjhxJ586d0XUdSZJQFIWcnBzGjBlTZ/FccnIyVquV0047DYvFQteuXVm4cCGBQOCERlr9mmiahqqqjBw5kubNm5OSkpK4NtM00TQtMeFfc8/W5Hx56KGHWL16NbNmzeLvf/87zz33HB6Pp9Y9etFFF9VbvOPQ/mvVqhUWiyXhRvqt2bp1K0uXLmX8+PG15iwETcSls3r1ah5//HEmTpzI5MmTWbVqFZqmHZPga5pG69atycnJSYg9QP/+/fn222+ZO3cuEC9v5vF4OP/889m7dy9LliyhW7duXH/99RQUFPDtt99y9tlnEwwGadGiBZdffjn9+/enY8eO9YayHThwgIyMDHw+X6MYsHA4jM/nIycnp5aANGvWjH79+vHxxx+zc+dOSkpK+M9//kOfPn1o3bo1n3/+OT6fjxtuuIFmzZqxYMGCxD7Ky8u5+OKLGTFiBD179qwTtjpkyBAqKioSkTkbNmygXbt2jUbsa0TcMAzatm1LixYtal3bgAEDUFWV//znPwB88sknFBQUMHz4cL7//nuWL1/OmWeeyRVXXEF+fj579+5l4MCBlJSU0LZtW6644grOOuusOkEO33zzDc888wzFxcWEw2GWLFmC1+ulRYsWJ0Wf5Obm8txzzwmxb0Qcs4W/bds25syZQ8uWLdmxYwehUIiFCxdy6qmncscddxzRpx8KhRIJlX7MeeedRzgcZsqUKUyePJnk5GTuvfdenE4nxcXFvPvuu4l81pdddhn9+/cHYPTo0bzxxhv8+9//RlVV/vznP3P22WfX2f/27duRZblRFDPQdZ2SkhKqqqrqbb/xxhsJhUI88MADmKZJ+/btGTt2LBaLhc8++4w333wTu91OcnIyt956KwC33347zzzzTMKl1q9fP2655ZZa+23WrBljxozh9ddfZ/r06eTk5DS6bKiBQICioqJ6C1efcsop3Hfffbz66qtceeWV2O12brvtNjp37szSpUt54403ePvtt9F1nT/96U906NABgOuuu47nn38ep9OJruvcdttt9OrVK7HfzMxMiouLGTt2LNFoNHHv/1bGSVVVVSKrJcQjl9xut1DJRsQxpVaIRqOMGzcOv99PVlZWwg1gmiZ79+7l7LPPZtSoUYfd/sCBA+zatYuuXbvWChM8lD179rBv3z46duxYK/dGZWUl+fn5eDweOnbsWCs6aP/+/ezevZsWLVoc1irasGEDdrudjh07NvjBikQibNiwgezs7MNGJBmGwaZNm1BVla5duya+egzDYPPmzQQCATp27FjLRxyJRPjuu++w2Wx06tTpsIt+CgoK2LNnD926dWs0X0yH3n9FRUV069btsO7K0tJStm7dSqtWrWrNcZSXl5Ofn09aWhrt27evNY9UWFjI3r17adu2LdnZ2fW+xLds2UIgEKB9+/ZHLQ5yotiwYQPPPfccmZmZPPvss0IZGyfHlktn3bp1vPbaa/XesDWrZ++///562wUCwcnNG2+8wcyZMxkwYAA33ngjWVlZolMaqeAfk0unqKgIwzASVn0tn9APFnd1dbUQfIGggWGaJsFgkHvvvZfBgweLDmnkHJPg+3y+OuGOh7oQTNNsVHlVBILGyu7duyksLEzMhUmSxF//+lfRMULw/0fnzp1xOBxUVlaSnJyMYRgAyLJMZWUleXl5J01kQWNi/fr1bNu2DdM0admyJb169Wr0K1x/TTRNY926dezatQvTNGnbti1nnHFGo8z+mJ+fz/Lly5k/fz4dO3ZMCL5ACH4dUlJSuOKKK3jzzTcJh8OkpKSgqirhcJjk5GSuvvpq0ZPH+TP7zTffZPny5ciyjCzLRCIRevbsyS233CK+po4DhmEwceJEVqxYgdPpxDRNYrEY/fv3Z/To0Y3qWqPRKK+//jqxWIzRo0cnckwJmh4/qQDK1q1b+fjjjzlw4AApKSnIsszOnTsZNmwYF198caMIfTwZePfdd/n444855ZRTavXpzp076dGjR4NKtHWy8sILL7B27Vpat26dsOgNw2DXrl0MGjSoQYadRiIR1q5dy5o1a7j44osTkVyapuH3+0lLSxO565s21T9ppW2HDh1o3749fr8fl8uF3W7nk08+Ydq0aZxxxhmivOFxoKCggFWrViVWXR46SZ6bm8uuXbvYs2fPYcMyBUdn7969bNu2LRFaWdPHsizTsmVL1qxZw+DBg8nLyzvpzj0cDlNUVERKSkqtBYyzZ89m2rRpuN1uMjMza825KYpCenq6GHgBv7jEoWma+P1+vF5vImKnpKSEV155BUmS0DQNn8/HmDFjEq6InTt3MnHiRDweD4FAgNatW3PzzTcn9rl582beeustUlJSqK6uplWrVrUWA+3atYuJEyficrkIBoN12tevX8+UKVNIT0+nrKyMHj16cM011yTav/jiC2bNmkVWVhalpaX07duXyy67LNH+zTffMGXKFFJTUykvL+f0009PLEwCWLp0KbNnzyY7O5uioiKGDBlSK8PkRx99xJIlSxILay6++GIGDRqUaP/888/58MMPE+2Hbv/VV18xadIkmjdvXiciSpIkotEoVqsVr9eLpmmUl5dz3XXX0b1798TvZs2axcqVK0lPT6eoqIjLLrusVjrb9957j3Xr1pGWlkZpaSnXXXcdp59+eqL9tddeY8+ePbjdbkKhEDfddFOtlNU17V6vl4qKCv70pz/RuXPnRPuECRPYvXt3YnxvuummxCrTSCTCyy+/TFVVFYqiYJomt912WyKLZFVVFS+//DKxWAxZluu0h8NhXn75Zaqrq+vd/mj3XkFBAc899xw2mw2Hw1FvH8diMRRFwWKx4PV6uf322xNzJz++d49075WXl3Paaadx3XXX1dvu9/vp0KFDrTUsP753a+699evX8/rrr+N2u9m7dy/nnHNOrWdm9erVFBYWMnjw4N8sll/QyCz8et8YklTnBrPZbAnrSNd1XC5XLYvD6XQm8vFEIpE6cb8ej4cOHTrg8XgIh8N1kp7VbG+324lEInXafT4fHTt2xOfzkZGRUSePTkpKCh07diQlJYX09PQ6x/f5fHTo0OGI23fq1Ink5GRSUlLqHD8zMzOxuCk1NbVOEefU1NRa7Yduf7RKR7qu07JlS3JycgiHw/WmjcjKykpcf0pKSp3xad68OaFQCK/XS3p6ep3tW7Rogd1ux+FwEI1G61R9qml3Op315tNp0aJFQlAjkUit7WVZpnXr1oRCoYS76tCJaEVRaNu2LZqmJdwPh7bLskybNm0Ih8OJe+rQ9qPde16vl5YtW7Jv377D5vSPxWLk5uYmLOVD3SA/vnd/6r13aHsgEKiTDvnQ9vT09MT2ycnJ9OzZk7y8PDIzM+ssYDzzzDMbRUZYwUlu4QuO/yf7+PHjCQaDpKam1omIkiSJ8ePHN7qVrr8m5eXlPPzwwyiKQlJSUq0+9vv9GIbBo48+KixlQaOz8GXRBycXTqeTK6+8Er/fn8jtomkapaWliRTQQux/GampqVx22WWUl5dTXFyMqqqoqkpRURF+v5/rr79eiL1AWPiCX4/Vq1ezcOFC/H4/pmnicrkYMmRIrbkAwS9jxYoVLFy4kGAwiGmaeL1ezjvvvERxGIGgsVn4QvBPYkzTpLi4GF3XycrKEmGvJ6iPDx48CMTnPg63olwgEIIvEAgEggYj+MKcEQgEgiaCEHyBQCAQgi8QCAQCIfgCgUAgEIIvEAgEAiH4AoFAIBCCLxAIBAIh+AKBQCAQgi8QCAQCIfgCgUAgEIIvEAgEQvAFAoFA0IhRgGriydOqRXcIBAJBo8QLVP//AQCtABhFQRIF5gAAAABJRU5ErkJggg==

    :param pos_list: list[posj] - List of target joints positions [deg].
    :param vel: float or float[6] - velocity (same to all axes) or velocity (to each axis) [deg/s].
    :param acc: float or float[6] - acceleration (same to all axes) or acceleration (to each axis) [deg/s^2].
    :param time: float - Reach time [sec].
    :param mod: int - Movement basis (DR_MV_MOD_ABS: Absolute, DR_MV_MOD_REL: Relative). Default: DR_MV_MOD_ABS.
    :param v: float or float[6] - velocity (same to all axes) or velocity (to each axis) [deg/s].
    :param a: float or float[6] - acceleration (same to all axes) or acceleration (to each axis) [deg/s^2].
    :param t: float - Reach time [sec].
    :param r: float - Radius for Blending [mm].

    :return: int - (0 -> Success, Negative value -> Error)
    """

    for p in pos_list:
        movej(p, vel=vel, acc=acc, time=time, radius=None, mod= mod, ra=DR_MV_RA_DUPLICATE, v=v, a=a, t=t, r=None)
    return 0


def amovesj(pos_list, vel=None, acc=None, time=None, mod= DR_MV_MOD_ABS, v=None, a=None, t=None) -> int:
    """
    The asynchronous movesj motion operates in the same way as movesj except for the asynchronous processing.
    Generating a new command for the motion before the amovesj motion results in an error for safety reasons.
    Therefore, the termination of the amovesj motion must be confirmed using mwait() or check_motion() between amovesj() and the following motion command.

    .. image:: data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAXwAAADOCAYAAAA9krkAAAAACXBIWXMAAAsTAAALEwEAmpwYAAA4IGlUWHRYTUw6Y29tLmFkb2JlLnhtcAAAAAAAPD94cGFja2V0IGJlZ2luPSLvu78iIGlkPSJXNU0wTXBDZWhpSHpyZVN6TlRjemtjOWQiPz4KPHg6eG1wbWV0YSB4bWxuczp4PSJhZG9iZTpuczptZXRhLyIgeDp4bXB0az0iQWRvYmUgWE1QIENvcmUgNS41LWMwMjEgNzkuMTU0OTExLCAyMDEzLzEwLzI5LTExOjQ3OjE2ICAgICAgICAiPgogICA8cmRmOlJERiB4bWxuczpyZGY9Imh0dHA6Ly93d3cudzMub3JnLzE5OTkvMDIvMjItcmRmLXN5bnRheC1ucyMiPgogICAgICA8cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0iIgogICAgICAgICAgICB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmRjPSJodHRwOi8vcHVybC5vcmcvZGMvZWxlbWVudHMvMS4xLyIKICAgICAgICAgICAgeG1sbnM6cGhvdG9zaG9wPSJodHRwOi8vbnMuYWRvYmUuY29tL3Bob3Rvc2hvcC8xLjAvIgogICAgICAgICAgICB4bWxuczp4bXBNTT0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wL21tLyIKICAgICAgICAgICAgeG1sbnM6c3RFdnQ9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9zVHlwZS9SZXNvdXJjZUV2ZW50IyIKICAgICAgICAgICAgeG1sbnM6dGlmZj0iaHR0cDovL25zLmFkb2JlLmNvbS90aWZmLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmV4aWY9Imh0dHA6Ly9ucy5hZG9iZS5jb20vZXhpZi8xLjAvIj4KICAgICAgICAgPHhtcDpDcmVhdG9yVG9vbD5BZG9iZSBQaG90b3Nob3AgQ0MgKE1hY2ludG9zaCk8L3htcDpDcmVhdG9yVG9vbD4KICAgICAgICAgPHhtcDpDcmVhdGVEYXRlPjIwMTctMTEtMjBUMTY6NDU6NDgrMDk6MDA8L3htcDpDcmVhdGVEYXRlPgogICAgICAgICA8eG1wOk1vZGlmeURhdGU+MjAxNy0xMi0xM1QxNzoxNzozMyswOTowMDwveG1wOk1vZGlmeURhdGU+CiAgICAgICAgIDx4bXA6TWV0YWRhdGFEYXRlPjIwMTctMTItMTNUMTc6MTc6MzMrMDk6MDA8L3htcDpNZXRhZGF0YURhdGU+CiAgICAgICAgIDxkYzpmb3JtYXQ+aW1hZ2UvcG5nPC9kYzpmb3JtYXQ+CiAgICAgICAgIDxwaG90b3Nob3A6Q29sb3JNb2RlPjM8L3Bob3Rvc2hvcDpDb2xvck1vZGU+CiAgICAgICAgIDx4bXBNTTpJbnN0YW5jZUlEPnhtcC5paWQ6ZGZmNGQ0ZGUtMWJhNS00MWMyLTgzYTUtOTNhYmQzNWI5MTNhPC94bXBNTTpJbnN0YW5jZUlEPgogICAgICAgICA8eG1wTU06RG9jdW1lbnRJRD54bXAuZGlkOmRmZjRkNGRlLTFiYTUtNDFjMi04M2E1LTkzYWJkMzViOTEzYTwveG1wTU06RG9jdW1lbnRJRD4KICAgICAgICAgPHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD54bXAuZGlkOmRmZjRkNGRlLTFiYTUtNDFjMi04M2E1LTkzYWJkMzViOTEzYTwveG1wTU06T3JpZ2luYWxEb2N1bWVudElEPgogICAgICAgICA8eG1wTU06SGlzdG9yeT4KICAgICAgICAgICAgPHJkZjpTZXE+CiAgICAgICAgICAgICAgIDxyZGY6bGkgcmRmOnBhcnNlVHlwZT0iUmVzb3VyY2UiPgogICAgICAgICAgICAgICAgICA8c3RFdnQ6YWN0aW9uPmNyZWF0ZWQ8L3N0RXZ0OmFjdGlvbj4KICAgICAgICAgICAgICAgICAgPHN0RXZ0Omluc3RhbmNlSUQ+eG1wLmlpZDpkZmY0ZDRkZS0xYmE1LTQxYzItODNhNS05M2FiZDM1YjkxM2E8L3N0RXZ0Omluc3RhbmNlSUQ+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDp3aGVuPjIwMTctMTEtMjBUMTY6NDU6NDgrMDk6MDA8L3N0RXZ0OndoZW4+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDpzb2Z0d2FyZUFnZW50PkFkb2JlIFBob3Rvc2hvcCBDQyAoTWFjaW50b3NoKTwvc3RFdnQ6c29mdHdhcmVBZ2VudD4KICAgICAgICAgICAgICAgPC9yZGY6bGk+CiAgICAgICAgICAgIDwvcmRmOlNlcT4KICAgICAgICAgPC94bXBNTTpIaXN0b3J5PgogICAgICAgICA8dGlmZjpPcmllbnRhdGlvbj4xPC90aWZmOk9yaWVudGF0aW9uPgogICAgICAgICA8dGlmZjpYUmVzb2x1dGlvbj43MjAwMDAvMTAwMDA8L3RpZmY6WFJlc29sdXRpb24+CiAgICAgICAgIDx0aWZmOllSZXNvbHV0aW9uPjcyMDAwMC8xMDAwMDwvdGlmZjpZUmVzb2x1dGlvbj4KICAgICAgICAgPHRpZmY6UmVzb2x1dGlvblVuaXQ+MjwvdGlmZjpSZXNvbHV0aW9uVW5pdD4KICAgICAgICAgPGV4aWY6Q29sb3JTcGFjZT42NTUzNTwvZXhpZjpDb2xvclNwYWNlPgogICAgICAgICA8ZXhpZjpQaXhlbFhEaW1lbnNpb24+MzgwPC9leGlmOlBpeGVsWERpbWVuc2lvbj4KICAgICAgICAgPGV4aWY6UGl4ZWxZRGltZW5zaW9uPjIwNjwvZXhpZjpQaXhlbFlEaW1lbnNpb24+CiAgICAgIDwvcmRmOkRlc2NyaXB0aW9uPgogICA8L3JkZjpSREY+CjwveDp4bXBtZXRhPgogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAg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q5kzZw4PP/wwBQUFvPnmm9x9990MGTKEAwcOcOutt7JkyRIuuOACFi9ezEUXXcTNN99cy10zefJkRo4cybBhw/jyyy95/fXXufjii+u13isqKli8eDHDhw9PfDnUR7pTYXALL0sKqumT4yXLZaEyykm3IEvih9W1EZXmqXbxUAvBFxxJ8DVN46OPPiIcDpOXl8dVV10FwIYNGzjllFMSlnvHjh3JyMhg9erVXHfddRQUFHDnnXdy5pln8n//939kZGRQUFBAWloaU6ZMIRqNUlpaSnV1tejoH+Hz+di2bRvr16/H4/Fw++23079/fxYvXkxycjIdOnQAICMjg7Zt27Jp0yYuuOACnnzySV555RVuueUWLrnkEs4//3x27tyJLMvs27ePd999l8LCQmKxGOXl5fUK/sSJE7FarYwYMeKo55nmVDirmZOvDwQ4q7mXVIdMIHZyWfqKBcrCOooEuV6buLmE4AsORzAYxG63M27cOHw+X602t9uN3++nqqqK1NRUqqurCQaDWCzx7+a//e1vXHXVVbz66quMHTuWl156ifT0dDZv3kyrVq2w2+307t37sBE/iqIgy3KTdOkcOHCA66+/nmHDhtX6yjEMg0gkQjgcBiAajRIIBBKul+7duzNx4kQWLFjAq6++imEYdOzYkWg0iqIodOrUiU6dOnHppZeSnZ1d57ivv/463333HU899dQRrftDaeG1E9FMVu2rYmDLJFxWiZB68oi+TQZ/VCPVoSD/jJMqKyvju+++Q1VVwuEwPp+Prl271vuyPJ6oqsq6des4cOAA2dnZnH766Ud0sf0URJSO4LA3naqqOByOOm1Dhw6lqqqKiRMn4vf7eeedd3A4HFx++eUsWrSId999l+zsbLp160ZxcTGyLHPeeedRWFiIJEn07t2bWCyWeEHUYBgGBw4cYOvWrZimSUVFBcXFxcRisSbR56ZpEolEEpOqh7q0evToQUpKChMnTuTgwYPMnj2bgoKCRNjqU089xYEDBxg0aBAul4tdu3bRunVr8vLy2LBhA926dSM3N5dQKITH46l13H//+998+OGHDBs2DLvdzurVqzlw4MAxnXO7FAfdMhx8vreSmGHissLJ8p42gYimk+n6eXbt3r17eemll5g9ezZffPEFEydO5IEHHmD37t0n9Lxfe+01nnzyST7//HOef/553nrrreO2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m9c65sCBAxk4cCAQz9b54IMPMmrUqMSCjQY5WIqCw+GgX79+XHXVVQC8+OKLvPPOO/Tq1YukpCQikQhLly4lFosxaNAgvF5vLVH77rvv6Nq1K6eddlri3w8ePMiiRYto164dvXv3rnPcXbt28f7773PzzTdz/vnnU1paetKL0k/F7XaTnZ3NqFGjyMjIYPv27YwdO5a5c+dy5ZVXArB9+3bWrFlD586da/VfIBBg8eLFmKZJv379yMjISLStWLGCgoICzjvvvHqj12ru6zvvvLPB9NWAAQMYMGCAUM+mIPg2m41HH330Zy07rvH1ZWRkkJmZSTQaBeCdd95h3rx5JCcnM2/ePP70pz8xcOBATNPklVdeYefOnSiKwpo1a3jwwQfxer3Mnj2bDz/8ELvdztKlS3nggQdo1arVYa1TTdO45JJLGvRg1ecrbdmyJStWrEhkIXz44YcpLi5GlmUWLFjA/fffT7NmzZg3bx4ffPABaWlpLF68mBtuuIGBAwdSUFDAk08+iaqqfPLJJ/zxj39kxIgRtY6xcuVKcnJyGDRoEFVVVfh8Pmw2W6N+MDIzM0lKSiIYDAKwaNEi3nrrLXw+H5988gnnnnsuN9xwA9FolOeff56ysjJUVeXbb7/loYceAuDf//43ixYtwul0smrVKh588MFaL4MawXc4HGzZsgWPx3NSGiSqqrJ3715yc3PFCvqm6NJxuVw/6YGXJAlZliktLQVg6tSpbN++nQsvvJBly5Yxffp0/v73vzNhwgQGDRrECy+8gKZpfPvtt3zwwQeMGjWKF198MSH2W7duZfr06TzwwANMmjSJvLw83nvvvXqPbZomX3/99WGz4zUkTNPEarUSCoWA+NzI9OnT6devH16vl6effpqioiL+9a9/8dZbbyHLMq+++iqhUIgZM2bQrl07nnzySV577TX69u0LwAsvvEDHjh2ZNGkSd999Nx9++CElJSV1PuFTU1OZOnUqd911F3fffTe7du1qVA+CJElIkkRZWRmmaTJ16lTKysq48MIL2bdvH//85z+56KKLmDBhAqNGjWLatGls27aNVatWsWTJEu644w5effVV7rjjjsRL8tNPP+WZZ57hjTfewOl0MnPmzDrHTUtLo7q6msmTJ3PPPfcc9j7+LZk2bRq33357nftC0PA4bguvjsTOnTvZtWsXW7ZsYebMmezevZvRo0dzxhln8M4775CVlcW1114LQFJSEgsXLiQ1NZV+/fpRXl7OBx98QGFhIaeeeipOp5OFCxeyb98+ZFlmx44dbN26ldLSUi644II6+bY3bdrE/Pnzufzyy8nMzGzQgxUIBNi0aRP79u3jo48+YtmyZXTv3p0777wTv9/PpEmT+P3vf0+PHj0AsFqtfPLJJwwYMICuXbvywQcf8OWXX9KsWTNycnL4/vvvWbRoEdnZ2RQUFLBlyxa+/fZbunbtWsvSXLNmDdu2bWP48OH8+c9/ZtmyZWzcuJEhQ4Y0mgdhy5Yt7Nmzhw0bNjBjxgzKysoYPXo0nTt35oMPPmD//v3ceuutuFwumjdvztq1a6mqquKSSy5h//79zJ49m9LSUrp3747VamXOnDmEQiFUVWXLli1s3rwZXddrJfWC+JL+wYMHM2zYMEzTZNq0aZx11lkkJyefFP3y5ptv8uGHH3LffffRpUsXoZgNm9hP8suEQiF27NhB+/btf1KEhmEYVFdXc8UVV9C+fXs8Hg9JSUlAvJzYoaXEVFVNRCwA3HXXXWzcuJH//Oc/3Hnnnbz44oskJSWhqip2ux3TNBk8eDAZGRkYhlEnpen27dux2+11fPwNldLSUnr27MmFF14IkCgIEQ6HkSSpVl/GYjF0XScUCtGzZ0/++c9/MnfuXJ555hlGjhxJ586d0XUdSZJQFIWcnBzGjBlTZ/FccnIyVquV0047DYvFQteuXVm4cCGBQOCERlr9mmiahqqqjBw5kubNm5OSkpK4NtM00TQtMeFfc8/W5Hx56KGHWL16NbNmzeLvf/87zz33HB6Pp9Y9etFFF9VbvOPQ/mvVqhUWiyXhRvqt2bp1K0uXLmX8+PG15iwETcSls3r1ah5//HEmTpzI5MmTWbVqFZqmHZPga5pG69atycnJSYg9QP/+/fn222+ZO3cuEC9v5vF4OP/889m7dy9LliyhW7duXH/99RQUFPDtt99y9tlnEwwGadGiBZdffjn9+/enY8eO9YayHThwgIyMDHw+X6MYsHA4jM/nIycnp5aANGvWjH79+vHxxx+zc+dOSkpK+M9//kOfPn1o3bo1n3/+OT6fjxtuuIFmzZqxYMGCxD7Ky8u5+OKLGTFiBD179qwTtjpkyBAqKioSkTkbNmygXbt2jUbsa0TcMAzatm1LixYtal3bgAEDUFWV//znPwB88sknFBQUMHz4cL7//nuWL1/OmWeeyRVXXEF+fj579+5l4MCBlJSU0LZtW6644grOOuusOkEO33zzDc888wzFxcWEw2GWLFmC1+ulRYsWJ0Wf5Obm8txzzwmxb0Qcs4W/bds25syZQ8uWLdmxYwehUIiFCxdy6qmncscddxzRpx8KhRIJlX7MeeedRzgcZsqUKUyePJnk5GTuvfdenE4nxcXFvPvuu4l81pdddhn9+/cHYPTo0bzxxhv8+9//RlVV/vznP3P22WfX2f/27duRZblRFDPQdZ2SkhKqqqrqbb/xxhsJhUI88MADmKZJ+/btGTt2LBaLhc8++4w333wTu91OcnIyt956KwC33347zzzzTMKl1q9fP2655ZZa+23WrBljxozh9ddfZ/r06eTk5DS6bKiBQICioqJ6C1efcsop3Hfffbz66qtceeWV2O12brvtNjp37szSpUt54403ePvtt9F1nT/96U906NABgOuuu47nn38ep9OJruvcdttt9OrVK7HfzMxMiouLGTt2LNFoNHHv/1bGSVVVVSKrJcQjl9xut1DJRsQxpVaIRqOMGzcOv99PVlZWwg1gmiZ79+7l7LPPZtSoUYfd/sCBA+zatYuuXbvWChM8lD179rBv3z46duxYK/dGZWUl+fn5eDweOnbsWCs6aP/+/ezevZsWLVoc1irasGEDdrudjh07NvjBikQibNiwgezs7MNGJBmGwaZNm1BVla5duya+egzDYPPmzQQCATp27FjLRxyJRPjuu++w2Wx06tTpsIt+CgoK2LNnD926dWs0X0yH3n9FRUV069btsO7K0tJStm7dSqtWrWrNcZSXl5Ofn09aWhrt27evNY9UWFjI3r17adu2LdnZ2fW+xLds2UIgEKB9+/ZHLQ5yotiwYQPPPfccmZmZPPvss0IZGyfHlktn3bp1vPbaa/XesDWrZ++///562wUCwcnNG2+8wcyZMxkwYAA33ngjWVlZolMaqeAfk0unqKgIwzASVn0tn9APFnd1dbUQfIGggWGaJsFgkHvvvZfBgweLDmnkHJPg+3y+OuGOh7oQTNNsVHlVBILGyu7duyksLEzMhUmSxF//+lfRMULw/0fnzp1xOBxUVlaSnJyMYRgAyLJMZWUleXl5J01kQWNi/fr1bNu2DdM0admyJb169Wr0K1x/TTRNY926dezatQvTNGnbti1nnHFGo8z+mJ+fz/Lly5k/fz4dO3ZMCL5ACH4dUlJSuOKKK3jzzTcJh8OkpKSgqirhcJjk5GSuvvpq0ZPH+TP7zTffZPny5ciyjCzLRCIRevbsyS233CK+po4DhmEwceJEVqxYgdPpxDRNYrEY/fv3Z/To0Y3qWqPRKK+//jqxWIzRo0cnckwJmh4/qQDK1q1b+fjjjzlw4AApKSnIsszOnTsZNmwYF198caMIfTwZePfdd/n444855ZRTavXpzp076dGjR4NKtHWy8sILL7B27Vpat26dsOgNw2DXrl0MGjSoQYadRiIR1q5dy5o1a7j44osTkVyapuH3+0lLSxO565s21T9ppW2HDh1o3749fr8fl8uF3W7nk08+Ydq0aZxxxhmivOFxoKCggFWrViVWXR46SZ6bm8uuXbvYs2fPYcMyBUdn7969bNu2LRFaWdPHsizTsmVL1qxZw+DBg8nLyzvpzj0cDlNUVERKSkqtBYyzZ89m2rRpuN1uMjMza825KYpCenq6GHgBv7jEoWma+P1+vF5vImKnpKSEV155BUmS0DQNn8/HmDFjEq6InTt3MnHiRDweD4FAgNatW3PzzTcn9rl582beeustUlJSqK6uplWrVrUWA+3atYuJEyficrkIBoN12tevX8+UKVNIT0+nrKyMHj16cM011yTav/jiC2bNmkVWVhalpaX07duXyy67LNH+zTffMGXKFFJTUykvL+f0009PLEwCWLp0KbNnzyY7O5uioiKGDBlSK8PkRx99xJIlSxILay6++GIGDRqUaP/888/58MMPE+2Hbv/VV18xadIkmjdvXiciSpIkotEoVqsVr9eLpmmUl5dz3XXX0b1798TvZs2axcqVK0lPT6eoqIjLLrusVjrb9957j3Xr1pGWlkZpaSnXXXcdp59+eqL9tddeY8+ePbjdbkKhEDfddFOtlNU17V6vl4qKCv70pz/RuXPnRPuECRPYvXt3YnxvuummxCrTSCTCyy+/TFVVFYqiYJomt912WyKLZFVVFS+//DKxWAxZluu0h8NhXn75Zaqrq+vd/mj3XkFBAc899xw2mw2Hw1FvH8diMRRFwWKx4PV6uf322xNzJz++d49075WXl3Paaadx3XXX1dvu9/vp0KFDrTUsP753a+699evX8/rrr+N2u9m7dy/nnHNOrWdm9erVFBYWMnjw4N8sll/QyCz8et8YklTnBrPZbAnrSNd1XC5XLYvD6XQm8vFEIpE6cb8ej4cOHTrg8XgIh8N1kp7VbG+324lEInXafT4fHTt2xOfzkZGRUSePTkpKCh07diQlJYX09PQ6x/f5fHTo0OGI23fq1Ink5GRSUlLqHD8zMzOxuCk1NbVOEefU1NRa7Yduf7RKR7qu07JlS3JycgiHw/WmjcjKykpcf0pKSp3xad68OaFQCK/XS3p6ep3tW7Rogd1ux+FwEI1G61R9qml3Op315tNp0aJFQlAjkUit7WVZpnXr1oRCoYS76tCJaEVRaNu2LZqmJdwPh7bLskybNm0Ih8OJe+rQ9qPde16vl5YtW7Jv377D5vSPxWLk5uYmLOVD3SA/vnd/6r13aHsgEKiTDvnQ9vT09MT2ycnJ9OzZk7y8PDIzM+ssYDzzzDMbRUZYwUlu4QuO/yf7+PHjCQaDpKam1omIkiSJ8ePHN7qVrr8m5eXlPPzwwyiKQlJSUq0+9vv9GIbBo48+KixlQaOz8GXRBycXTqeTK6+8Er/fn8jtomkapaWliRTQQux/GampqVx22WWUl5dTXFyMqqqoqkpRURF+v5/rr79eiL1AWPiCX4/Vq1ezcOFC/H4/pmnicrkYMmRIrbkAwS9jxYoVLFy4kGAwiGmaeL1ezjvvvERxGIGgsVn4QvBPYkzTpLi4GF3XycrKEmGvJ6iPDx48CMTnPg63olwgEIIvEAgEggYj+MKcEQgEgiaCEHyBQCAQgi8QCAQCIfgCgUAgEIIvEAgEAiH4AoFAIBCCLxAIBAIh+AKBQCAQgi8QCAQCIfgCgUAgEIIvEAgEQvAFAoFA0IhRgGriydOqRXcIBAJBo8QLVP//AQCtABhFQRIF5gAAAABJRU5ErkJggg==

    :param pos_list: list[posj] - List of target joints positions [deg].
    :param vel: float or float[6] - velocity (same to all axes) or velocity (to each axis) [deg/s].
    :param acc: float or float[6] - acceleration (same to all axes) or acceleration (to each axis) [deg/s^2].
    :param time: float - Reach time [sec].
    :param mod: int - Movement basis (DR_MV_MOD_ABS: Absolute, DR_MV_MOD_REL: Relative). Default: DR_MV_MOD_ABS.
    :param v: float or float[6] - velocity (same to all axes) or velocity (to each axis) [deg/s].
    :param a: float or float[6] - acceleration (same to all axes) or acceleration (to each axis) [deg/s^2].
    :param t: float - Reach time [sec].
    :param r: float - Radius for Blending [mm].

    :return: int - (0 -> Success, Negative value -> Error)
    """

    return 0


def movesx(pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None) -> int:
    """
    The robot moves along a spline curve path that connects the current position to the target position
    (the last waypoint in pos_list) via the waypoints of the task space input in pos_list.
    The input velocity/acceleration means the maximum velocity/acceleration in the path and the constant velocity motion
    is performed with the input velocity according to the condition if the option for the constant speed motion is selected.

    .. image:: data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAXwAAACqCAYAAACu9/RMAAAACXBIWXMAAAsTAAALEwEAmpwYAAA4IGlUWHRYTUw6Y29tLmFkb2JlLnhtcAAAAAAAPD94cGFja2V0IGJlZ2luPSLvu78iIGlkPSJXNU0wTXBDZWhpSHpyZVN6TlRjemtjOWQiPz4KPHg6eG1wbWV0YSB4bWxuczp4PSJhZG9iZTpuczptZXRhLyIgeDp4bXB0az0iQWRvYmUgWE1QIENvcmUgNS41LWMwMjEgNzkuMTU0OTExLCAyMDEzLzEwLzI5LTExOjQ3OjE2ICAgICAgICAiPgogICA8cmRmOlJERiB4bWxuczpyZGY9Imh0dHA6Ly93d3cudzMub3JnLzE5OTkvMDIvMjItcmRmLXN5bnRheC1ucyMiPgogICAgICA8cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0iIgogICAgICAgICAgICB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmRjPSJodHRwOi8vcHVybC5vcmcvZGMvZWxlbWVudHMvMS4xLyIKICAgICAgICAgICAgeG1sbnM6cGhvdG9zaG9wPSJodHRwOi8vbnMuYWRvYmUuY29tL3Bob3Rvc2hvcC8xLjAvIgogICAgICAgICAgICB4bWxuczp4bXBNTT0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wL21tLyIKICAgICAgICAgICAgeG1sbnM6c3RFdnQ9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9zVHlwZS9SZXNvdXJjZUV2ZW50IyIKICAgICAgICAgICAgeG1sbnM6dGlmZj0iaHR0cDovL25zLmFkb2JlLmNvbS90aWZmLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmV4aWY9Imh0dHA6Ly9ucy5hZG9iZS5jb20vZXhpZi8xLjAvIj4KICAgICAgICAgPHhtcDpDcmVhdG9yVG9vbD5BZG9iZSBQaG90b3Nob3AgQ0MgKE1hY2ludG9zaCk8L3htcDpDcmVhdG9yVG9vbD4KICAgICAgICAgPHhtcDpDcmVhdGVEYXRlPjIwMTctMTEtMjBUMTY6NDU6NDgrMDk6MDA8L3htcDpDcmVhdGVEYXRlPgogICAgICAgICA8eG1wOk1vZGlmeURhdGU+MjAxNy0xMi0xM1QxNzoxNToxMSswOTowMDwveG1wOk1vZGlmeURhdGU+CiAgICAgICAgIDx4bXA6TWV0YWRhdGFEYXRlPjIwMTctMTItMTNUMTc6MTU6MTErMDk6MDA8L3htcDpNZXRhZGF0YURhdGU+CiAgICAgICAgIDxkYzpmb3JtYXQ+aW1hZ2UvcG5nPC9kYzpmb3JtYXQ+CiAgICAgICAgIDxwaG90b3Nob3A6Q29sb3JNb2RlPjM8L3Bob3Rvc2hvcDpDb2xvck1vZGU+CiAgICAgICAgIDx4bXBNTTpJbnN0YW5jZUlEPnhtcC5paWQ6N2RlODk0NzAtZTAwYS00MTA1LWJiMmYtZjljYjhlM2NhNTAxPC94bXBNTTpJbnN0YW5jZUlEPgogICAgICAgICA8eG1wTU06RG9jdW1lbnRJRD54bXAuZGlkOjdkZTg5NDcwLWUwMGEtNDEwNS1iYjJmLWY5Y2I4ZTNjYTUwMTwveG1wTU06RG9jdW1lbnRJRD4KICAgICAgICAgPHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD54bXAuZGlkOjdkZTg5NDcwLWUwMGEtNDEwNS1iYjJmLWY5Y2I4ZTNjYTUwMTwveG1wTU06T3JpZ2luYWxEb2N1bWVudElEPgogICAgICAgICA8eG1wTU06SGlzdG9yeT4KICAgICAgICAgICAgPHJkZjpTZXE+CiAgICAgICAgICAgICAgIDxyZGY6bGkgcmRmOnBhcnNlVHlwZT0iUmVzb3VyY2UiPgogICAgICAgICAgICAgICAgICA8c3RFdnQ6YWN0aW9uPmNyZWF0ZWQ8L3N0RXZ0OmFjdGlvbj4KICAgICAgICAgICAgICAgICAgPHN0RXZ0Omluc3RhbmNlSUQ+eG1wLmlpZDo3ZGU4OTQ3MC1lMDBhLTQxMDUtYmIyZi1mOWNiOGUzY2E1MDE8L3N0RXZ0Omluc3RhbmNlSUQ+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDp3aGVuPjIwMTctMTEtMjBUMTY6NDU6NDgrMDk6MDA8L3N0RXZ0OndoZW4+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDpzb2Z0d2FyZUFnZW50PkFkb2JlIFBob3Rvc2hvcCBDQyAoTWFjaW50b3NoKTwvc3RFdnQ6c29mdHdhcmVBZ2VudD4KICAgICAgICAgICAgICAgPC9yZGY6bGk+CiAgICAgICAgICAgIDwvcmRmOlNlcT4KICAgICAgICAgPC94bXBNTTpIaXN0b3J5PgogICAgICAgICA8dGlmZjpPcmllbnRhdGlvbj4xPC90aWZmOk9yaWVudGF0aW9uPgogICAgICAgICA8dGlmZjpYUmVzb2x1dGlvbj43MjAwMDAvMTAwMDA8L3RpZmY6WFJlc29sdXRpb24+CiAgICAgICAgIDx0aWZmOllSZXNvbHV0aW9uPjcyMDAwMC8xMDAwMDwvdGlmZjpZUmVzb2x1dGlvbj4KICAgICAgICAgPHRpZmY6UmVzb2x1dGlvblVuaXQ+MjwvdGlmZjpSZXNvbHV0aW9uVW5pdD4KICAgICAgICAgPGV4aWY6Q29sb3JTcGFjZT42NTUzNTwvZXhpZjpDb2xvclNwYWNlPgogICAgICAgICA8ZXhpZjpQaXhlbFhEaW1lbnNpb24+MzgwPC9leGlmOlBpeGVsWERpbWVuc2lvbj4KICAgICAgICAgPGV4aWY6UGl4ZWxZRGltZW5zaW9uPjE3MDwvZXhpZjpQaXhlbFlEaW1lbnNpb24+CiAgICAgIDwvcmRmOkRlc2NyaXB0aW9uPgogICA8L3JkZjpSREY+CjwveDp4bXBtZXRhPgogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAg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Z5JIJIjFYhQXFzNlypSvrO9XrFjBc889x8SJEznppJPEwi90stksr732GgsWLEDTNBobG/nlL3/JSy+99KVet6uri/vuu4/HHnuMd999l2QyWTBttmHDBp577jmamprQNI05c+Zwww038Omnn35p11y8eDHXXXcdHR0dNDc3c8stt/DRRx8V3HibP38+s2bNAry0HA8++CD33Xffl5qwb+3atfzmN78hFovR2dnJL3/5S5YtW1YQ7RWNRnnhhRf4+OOP0TSNTz75hJ/97Gf885///Equ/9RTT/Hqq6+yZMmS/apdZOPV5xD8t956iyOPPJKrr76aKVOmEI/H+cc//sGUKVMIBAIALFq0iHXr1lFaWtonJUNDQwMffvhhPjFX77Z0gAULFrBu3Tpqa2vzaRfyd2hVZcSIEZimSWNjI6ecckrBxGY3NzezYMECfvzjH3PWWWcxdepUXnvtNT799NO8tbpmzRpWrFiBoijbpU5etGgR9fX1BAKBfHrqXmFasmQJgUCgT4I1gEQiQW1tLf/+7//OySefzHvvvceKFSuYOnVqQY23Dz74AFVVue2225g8eTLV1dX8z//8DyNHjsynpFixYgWrV6/GMIw+7dDR0cG8efOIRqOEw+F8kjTwUjE3NTUxYMCA7af/us6YMWO44IILmDJlCi+//DKdnZ0ce+yxBTEDf/PNNzn11FO54oorOOWUU6ivr2f27NmcfvrpaJpGS0sLS5YsIR6P90mbAbBq1SqWLl2KoiiEQqH897OtrY0FCxaQy+V2OjN99tlnaW1t5YgjjsDn8zFhwoT9RrbEpfM5p7xbC/LYsWN59dVX8fl8WJbFLbfcQnNzM6qqoqoqN910E0OHDuWNN97g2WefpbS0lIaGBs477zwuuOACXNflnnvu4cMPP0TXdWbOnMnPfvazPjlQTNOktraW8vLyfZKO+Ytss2AwSF1dHd3d3QD89a9/5bnnnqOkpITu7m4uvvhizjrrLGzb5v7772fFihX4fD6i0Si33HILdXV1zJo1i+nTp+fzxt922219pu3Dhg1j2LBhff6vr68vyHbbmjFjxmCaZj4+/95772XBggUEAgESiQTf+973OPHEE1m+fDl//OMfiUQiNDY2ctRRR/GTn/yEdDrNHXfcwdq1a7Esi6OOOor//M//7HONSCTC6NGjAW/fiWVZDBw4sGDa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

    :param pos_list: list[posx] - List of Task space positions.
    :param vel: float or float[2] - linear velocity or [linear velocity, angular velocity].
    :param acc: float or float[2] - linear acceleration or [linear acceleration, angular acceleration].
    :param time: float - Reach time [sec].
    :param ref: reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference). Default: DR_BASE.
    :param mod: int - Movement basis (DR_MV_MOD_ABS: Absolute, DR_MV_MOD_REL: Relative). Default: DR_MV_MOD_ABS.
    :param v: float or float[2] - linear velocity or [linear velocity, angular velocity].
    :param a: float or float[2] - linear acceleration or [linear acceleration, angular acceleration].
    :param t: float - Reach time [sec].
    :param vel_opt: int - Velocity option (DR_MVS_VEL_NONE -> None, DR_MVS_VEL_CONST -> Constant velocity).

    :return: int - (0 -> Success, Negative value -> Error)
    """
    for p in pos_list:
        movel(p, vel=vel, acc=acc, time=time, radius=None, ref=ref, mod=mod, ra=DR_MV_RA_DUPLICATE, v=v, a=a, t=t, r=None)
    return 0


def amovesx(pos_list, vel=None, acc=None, time=None, ref=None, mod= DR_MV_MOD_ABS, vel_opt=DR_MVS_VEL_NONE, v=None, a=None, t=None) -> int:
    """
    The asynchronous movesx motion operates in the same way as movesx except for the asynchronous processing.
    Generating a new command for the motion before the amovesj motion results in an error for safety reasons.
    Therefore, the termination of the amovesx motion  must  be  confirmed  using  mwait()  or check_motion()
    between amovesx() and the following motion command.

    .. image:: data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAXwAAACqCAYAAACu9/RMAAAACXBIWXMAAAsTAAALEwEAmpwYAAA4IGlUWHRYTUw6Y29tLmFkb2JlLnhtcAAAAAAAPD94cGFja2V0IGJlZ2luPSLvu78iIGlkPSJXNU0wTXBDZWhpSHpyZVN6TlRjemtjOWQiPz4KPHg6eG1wbWV0YSB4bWxuczp4PSJhZG9iZTpuczptZXRhLyIgeDp4bXB0az0iQWRvYmUgWE1QIENvcmUgNS41LWMwMjEgNzkuMTU0OTExLCAyMDEzLzEwLzI5LTExOjQ3OjE2ICAgICAgICAiPgogICA8cmRmOlJERiB4bWxuczpyZGY9Imh0dHA6Ly93d3cudzMub3JnLzE5OTkvMDIvMjItcmRmLXN5bnRheC1ucyMiPgogICAgICA8cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0iIgogICAgICAgICAgICB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmRjPSJodHRwOi8vcHVybC5vcmcvZGMvZWxlbWVudHMvMS4xLyIKICAgICAgICAgICAgeG1sbnM6cGhvdG9zaG9wPSJodHRwOi8vbnMuYWRvYmUuY29tL3Bob3Rvc2hvcC8xLjAvIgogICAgICAgICAgICB4bWxuczp4bXBNTT0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wL21tLyIKICAgICAgICAgICAgeG1sbnM6c3RFdnQ9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9zVHlwZS9SZXNvdXJjZUV2ZW50IyIKICAgICAgICAgICAgeG1sbnM6dGlmZj0iaHR0cDovL25zLmFkb2JlLmNvbS90aWZmLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmV4aWY9Imh0dHA6Ly9ucy5hZG9iZS5jb20vZXhpZi8xLjAvIj4KICAgICAgICAgPHhtcDpDcmVhdG9yVG9vbD5BZG9iZSBQaG90b3Nob3AgQ0MgKE1hY2ludG9zaCk8L3htcDpDcmVhdG9yVG9vbD4KICAgICAgICAgPHhtcDpDcmVhdGVEYXRlPjIwMTctMTEtMjBUMTY6NDU6NDgrMDk6MDA8L3htcDpDcmVhdGVEYXRlPgogICAgICAgICA8eG1wOk1vZGlmeURhdGU+MjAxNy0xMi0xM1QxNzoxNToxMSswOTowMDwveG1wOk1vZGlmeURhdGU+CiAgICAgICAgIDx4bXA6TWV0YWRhdGFEYXRlPjIwMTctMTItMTNUMTc6MTU6MTErMDk6MDA8L3htcDpNZXRhZGF0YURhdGU+CiAgICAgICAgIDxkYzpmb3JtYXQ+aW1hZ2UvcG5nPC9kYzpmb3JtYXQ+CiAgICAgICAgIDxwaG90b3Nob3A6Q29sb3JNb2RlPjM8L3Bob3Rvc2hvcDpDb2xvck1vZGU+CiAgICAgICAgIDx4bXBNTTpJbnN0YW5jZUlEPnhtcC5paWQ6N2RlODk0NzAtZTAwYS00MTA1LWJiMmYtZjljYjhlM2NhNTAxPC94bXBNTTpJbnN0YW5jZUlEPgogICAgICAgICA8eG1wTU06RG9jdW1lbnRJRD54bXAuZGlkOjdkZTg5NDcwLWUwMGEtNDEwNS1iYjJmLWY5Y2I4ZTNjYTUwMTwveG1wTU06RG9jdW1lbnRJRD4KICAgICAgICAgPHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD54bXAuZGlkOjdkZTg5NDcwLWUwMGEtNDEwNS1iYjJmLWY5Y2I4ZTNjYTUwMTwveG1wTU06T3JpZ2luYWxEb2N1bWVudElEPgogICAgICAgICA8eG1wTU06SGlzdG9yeT4KICAgICAgICAgICAgPHJkZjpTZXE+CiAgICAgICAgICAgICAgIDxyZGY6bGkgcmRmOnBhcnNlVHlwZT0iUmVzb3VyY2UiPgogICAgICAgICAgICAgICAgICA8c3RFdnQ6YWN0aW9uPmNyZWF0ZWQ8L3N0RXZ0OmFjdGlvbj4KICAgICAgICAgICAgICAgICAgPHN0RXZ0Omluc3RhbmNlSUQ+eG1wLmlpZDo3ZGU4OTQ3MC1lMDBhLTQxMDUtYmIyZi1mOWNiOGUzY2E1MDE8L3N0RXZ0Omluc3RhbmNlSUQ+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDp3aGVuPjIwMTctMTEtMjBUMTY6NDU6NDgrMDk6MDA8L3N0RXZ0OndoZW4+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDpzb2Z0d2FyZUFnZW50PkFkb2JlIFBob3Rvc2hvcCBDQyAoTWFjaW50b3NoKTwvc3RFdnQ6c29mdHdhcmVBZ2VudD4KICAgICAgICAgICAgICAgPC9yZGY6bGk+CiAgICAgICAgICAgIDwvcmRmOlNlcT4KICAgICAgICAgPC94bXBNTTpIaXN0b3J5PgogICAgICAgICA8dGlmZjpPcmllbnRhdGlvbj4xPC90aWZmOk9yaWVudGF0aW9uPgogICAgICAgICA8dGlmZjpYUmVzb2x1dGlvbj43MjAwMDAvMTAwMDA8L3RpZmY6WFJlc29sdXRpb24+CiAgICAgICAgIDx0aWZmOllSZXNvbHV0aW9uPjcyMDAwMC8xMDAwMDwvdGlmZjpZUmVzb2x1dGlvbj4KICAgICAgICAgPHRpZmY6UmVzb2x1dGlvblVuaXQ+MjwvdGlmZjpSZXNvbHV0aW9uVW5pdD4KICAgICAgICAgPGV4aWY6Q29sb3JTcGFjZT42NTUzNTwvZXhpZjpDb2xvclNwYWNlPgogICAgICAgICA8ZXhpZjpQaXhlbFhEaW1lbnNpb24+MzgwPC9leGlmOlBpeGVsWERpbWVuc2lvbj4KICAgICAgICAgPGV4aWY6UGl4ZWxZRGltZW5zaW9uPjE3MDwvZXhpZjpQaXhlbFlEaW1lbnNpb24+CiAgICAgIDwvcmRmOkRlc2NyaXB0aW9uPgogICA8L3JkZjpSREY+CjwveDp4bXBtZXRhPgogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAg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Z5JIJIjFYhQXFzNlypSvrO9XrFjBc889x8SJEznppJPEwi90stksr732GgsWLEDTNBobG/nlL3/JSy+99KVet6uri/vuu4/HHnuMd999l2QyWTBttmHDBp577jmamprQNI05c+Zwww038Omnn35p11y8eDHXXXcdHR0dNDc3c8stt/DRRx8V3HibP38+s2bNAry0HA8++CD33Xffl5qwb+3atfzmN78hFovR2dnJL3/5S5YtW1YQ7RWNRnnhhRf4+OOP0TSNTz75hJ/97Gf885///Equ/9RTT/Hqq6+yZMmS/apdZOPV5xD8t956iyOPPJKrr76aKVOmEI/H+cc//sGUKVMIBAIALFq0iHXr1lFaWtonJUNDQwMffvhhPjFX77Z0gAULFrBu3Tpqa2vzaRfyd2hVZcSIEZimSWNjI6ecckrBxGY3NzezYMECfvzjH3PWWWcxdepUXnvtNT799NO8tbpmzRpWrFiBoijbpU5etGgR9fX1BAKBfHrqXmFasmQJgUCgT4I1gEQiQW1tLf/+7//OySefzHvvvceKFSuYOnVqQY23Dz74AFVVue2225g8eTLV1dX8z//8DyNHjsynpFixYgWrV6/GMIw+7dDR0cG8efOIRqOEw+F8kjTwUjE3NTUxYMCA7af/us6YMWO44IILmDJlCi+//DKdnZ0ce+yxBTEDf/PNNzn11FO54oorOOWUU6ivr2f27NmcfvrpaJpGS0sLS5YsIR6P90mbAbBq1SqWLl2KoiiEQqH897OtrY0FCxaQy+V2OjN99tlnaW1t5YgjjsDn8zFhwoT9RrbEpfM5p7xbC/LYsWN59dVX8fl8WJbFLbfcQnNzM6qqoqoqN910E0OHDuWNN97g2WefpbS0lIaGBs477zwuuOACXNflnnvu4cMPP0TXdWbOnMnPfvazPjlQTNOktraW8vLyfZKO+Ytss2AwSF1dHd3d3QD89a9/5bnnnqOkpITu7m4uvvhizjrrLGzb5v7772fFihX4fD6i0Si33HILdXV1zJo1i+nTp+fzxt922219pu3Dhg1j2LBhff6vr68vyHbbmjFjxmCaZj4+/95772XBggUEAgESiQTf+973OPHEE1m+fDl//OMfiUQiNDY2ctRRR/GTn/yEdDrNHXfcwdq1a7Esi6OOOor//M//7HONSCTC6NGjAW/fiWVZDBw4sGDa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

    :param pos_list: list[posx] - List of Task space positions.
    :param vel: float or float[2] - linear velocity or [linear velocity, angular velocity].
    :param acc: float or float[2] - linear acceleration or [linear acceleration, angular acceleration].
    :param time: float - Reach time [sec].
    :param ref: reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference). Default: DR_BASE.
    :param mod: int - Movement basis (DR_MV_MOD_ABS: Absolute, DR_MV_MOD_REL: Relative). Default: DR_MV_MOD_ABS.
    :param v: float or float[2] - linear velocity or [linear velocity, angular velocity].
    :param a: float or float[2] - linear acceleration or [linear acceleration, angular acceleration].
    :param t: float - Reach time [sec].
    :param vel_opt: int - Velocity option (DR_MVS_VEL_NONE -> None, DR_MVS_VEL_CONST -> Constant velocity).

    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def moveb(seg_list, vel=None, acc=None, ref=None, time=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None) -> int:
    """
    This function takes a list that has one or more path segments (line or circle) as arguments
    and moves at a constant velocity by blending each segment into the specified radius.
    Here, the radius can be set through posb.

    .. image:: data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAXwAAAC0CAYAAACXK5enAAAACXBIWXMAAAsTAAALEwEAmpwYAAA4IGlUWHRYTUw6Y29tLmFkb2JlLnhtcAAAAAAAPD94cGFja2V0IGJlZ2luPSLvu78iIGlkPSJXNU0wTXBDZWhpSHpyZVN6TlRjemtjOWQiPz4KPHg6eG1wbWV0YSB4bWxuczp4PSJhZG9iZTpuczptZXRhLyIgeDp4bXB0az0iQWRvYmUgWE1QIENvcmUgNS41LWMwMjEgNzkuMTU0OTExLCAyMDEzLzEwLzI5LTExOjQ3OjE2ICAgICAgICAiPgogICA8cmRmOlJERiB4bWxuczpyZGY9Imh0dHA6Ly93d3cudzMub3JnLzE5OTkvMDIvMjItcmRmLXN5bnRheC1ucyMiPgogICAgICA8cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0iIgogICAgICAgICAgICB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmRjPSJodHRwOi8vcHVybC5vcmcvZGMvZWxlbWVudHMvMS4xLyIKICAgICAgICAgICAgeG1sbnM6cGhvdG9zaG9wPSJodHRwOi8vbnMuYWRvYmUuY29tL3Bob3Rvc2hvcC8xLjAvIgogICAgICAgICAgICB4bWxuczp4bXBNTT0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wL21tLyIKICAgICAgICAgICAgeG1sbnM6c3RFdnQ9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9zVHlwZS9SZXNvdXJjZUV2ZW50IyIKICAgICAgICAgICAgeG1sbnM6dGlmZj0iaHR0cDovL25zLmFkb2JlLmNvbS90aWZmLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmV4aWY9Imh0dHA6Ly9ucy5hZG9iZS5jb20vZXhpZi8xLjAvIj4KICAgICAgICAgPHhtcDpDcmVhdG9yVG9vbD5BZG9iZSBQaG90b3Nob3AgQ0MgKE1hY2ludG9zaCk8L3htcDpDcmVhdG9yVG9vbD4KICAgICAgICAgPHhtcDpDcmVhdGVEYXRlPjIwMTctMTEtMjBUMTY6NDU6NTArMDk6MDA8L3htcDpDcmVhdGVEYXRlPgogICAgICAgICA8eG1wOk1vZGlmeURhdGU+MjAxNy0xMi0xM1QxNzo1MDo0NSswOTowMDwveG1wOk1vZGlmeURhdGU+CiAgICAgICAgIDx4bXA6TWV0YWRhdGFEYXRlPjIwMTctMTItMTNUMTc6NTA6NDUrMDk6MDA8L3htcDpNZXRhZGF0YURhdGU+CiAgICAgICAgIDxkYzpmb3JtYXQ+aW1hZ2UvcG5nPC9kYzpmb3JtYXQ+CiAgICAgICAgIDxwaG90b3Nob3A6Q29sb3JNb2RlPjM8L3Bob3Rvc2hvcDpDb2xvck1vZGU+CiAgICAgICAgIDx4bXBNTTpJbnN0YW5jZUlEPnhtcC5paWQ6NjhkZGFlMTQtODJjNi00MDZhLTkyMTEtNDFiN2IwOGVjYmFlPC94bXBNTTpJbnN0YW5jZUlEPgogICAgICAgICA8eG1wTU06RG9jdW1lbnRJRD54bXAuZGlkOjY4ZGRhZTE0LTgyYzYtNDA2YS05MjExLTQxYjdiMDhlY2JhZTwveG1wTU06RG9jdW1lbnRJRD4KICAgICAgICAgPHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD54bXAuZGlkOjY4ZGRhZTE0LTgyYzYtNDA2YS05MjExLTQxYjdiMDhlY2JhZTwveG1wTU06T3JpZ2luYWxEb2N1bWVudElEPgogICAgICAgICA8eG1wTU06SGlzdG9yeT4KICAgICAgICAgICAgPHJkZjpTZXE+CiAgICAgICAgICAgICAgIDxyZGY6bGkgcmRmOnBhcnNlVHlwZT0iUmVzb3VyY2UiPgogICAgICAgICAgICAgICAgICA8c3RFdnQ6YWN0aW9uPmNyZWF0ZWQ8L3N0RXZ0OmFjdGlvbj4KICAgICAgICAgICAgICAgICAgPHN0RXZ0Omluc3RhbmNlSUQ+eG1wLmlpZDo2OGRkYWUxNC04MmM2LTQwNmEtOTIxMS00MWI3YjA4ZWNiYWU8L3N0RXZ0Omluc3RhbmNlSUQ+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDp3aGVuPjIwMTctMTEtMjBUMTY6NDU6NTArMDk6MDA8L3N0RXZ0OndoZW4+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDpzb2Z0d2FyZUFnZW50PkFkb2JlIFBob3Rvc2hvcCBDQyAoTWFjaW50b3NoKTwvc3RFdnQ6c29mdHdhcmVBZ2VudD4KICAgICAgICAgICAgICAgPC9yZGY6bGk+CiAgICAgICAgICAgIDwvcmRmOlNlcT4KICAgICAgICAgPC94bXBNTTpIaXN0b3J5PgogICAgICAgICA8dGlmZjpPcmllbnRhdGlvbj4xPC90aWZmOk9yaWVudGF0aW9uPgogICAgICAgICA8dGlmZjpYUmVzb2x1dGlvbj43MjAwMDAvMTAwMDA8L3RpZmY6WFJlc29sdXRpb24+CiAgICAgICAgIDx0aWZmOllSZXNvbHV0aW9uPjcyMDAwMC8xMDAwMDwvdGlmZjpZUmVzb2x1dGlvbj4KICAgICAgICAgPHRpZmY6UmVzb2x1dGlvblVuaXQ+MjwvdGlmZjpSZXNvbHV0aW9uVW5pdD4KICAgICAgICAgPGV4aWY6Q29sb3JTcGFjZT42NTUzNTwvZXhpZjpDb2xvclNwYWNlPgogICAgICAgICA8ZXhpZjpQaXhlbFhEaW1lbnNpb24+MzgwPC9leGlmOlBpeGVsWERpbWVuc2lvbj4KICAgICAgICAgPGV4aWY6UGl4ZWxZRGltZW5zaW9uPjE4MDwvZXhpZjpQaXhlbFlEaW1lbnNpb24+CiAgICAgIDwvcmRmOkRlc2NyaXB0aW9uPgogICA8L3JkZjpSREY+CjwveDp4bXBtZXRhPgogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAg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AF599VVefvll7HY7NTU13WbNixYtYsWKFWZKaZ8Pi8Vc8l62bBkvvPAC4XCYUCh0QL+ceeaZXHrppcyePZvKykqeeOIJLrzwwgP6UDgkRbHwD4FhGN1EvHMK6Xa7+etf/8qNN97Itm3bePnll/nWt77Fnj17AHjsscdYuHAhDoeDp556ijfeeKNLxG644QZ27drFSy+9xD333HOAlaKqKhaLBbvdfkxnEoPGItkvCIFAAEVRcDqdAPz2t7/lJz/5CXv37mXhwoXccccdADQ3N3PTTTcRiUTYs2cPjz/+eFc73n777Tz77LM0NTXx/e9/n23btnU7l9frZcKECV1/d7lcWK3Wrms4GDaLis+usjWWx20dHL58FSiUjzxDpmEYXZXA7HY7FosFn88HwI9+9CN+/etfs2fPHu68804eeeQRADo6Orj55pvZvn07HR0dPPzwwxQKZsmvX/7ylzz66KO0tLTwwx/+kA0bNhxwzs4U1alUilWrVjFr1iyCwaCI1GEiPvxD4PP5aG5uZuXKleRyOX7/+99zzjnnoCgKCxcuZObMmfzrv/4rmUyGf/mXf2HRokXccMMNrFq1CofDwTe+8Q0ACoUChUKB+++/n9mzZ/PNb36TXC7Hl770JebNm8fpp59+wLlLpRLKEF5Qs1gsWK1WXnvtNTweD8uWLQPg4osvZvv27SxatIhvfOMbXHrppbz99tvcfPPNXHLJJeTzedatW8e1117LtGnTur5v6dKlrFu3jkceeQS73c7NN9/M888/z3e+852Dnj8Wi/HnP/+ZT37yk12zs54Y4bOzqjVLTnNhUcwY9wFtyGCmVbAfgeArioLL5WLFihWEw2FWrVpFJBLh0ksv5aWXXmLVqlXcddddTJw4kT/84Q88/PDDfOITnyCVSvHmm29y++23c84553R937p163j11Ve59957qa+v5+c//zlPPfUUM2bMOODcTz31FI8//jgWi4V77723134RDv6yF3ogEAiwe/dufv3rX/Ob3/yGqVOn8rWvfY3GxkZaWlq6prAej4fzzz+f7du3YxgG11xzDalUiq9+9as8++yzOBwOSqUS5XIZVVV56KGHePjhh1EUhWg0Kg19MMvZZsNqtfL000/z4IMP0tLSwne/+10CgQCvvfYaFRUVXHTRRQDMmjWLCRMm8MorrzB9+nSuuOIKbrvtNm677bau9t20aRN1dXU8+eSTPPDAAySTyUOuj/ziF7/AYrHw6U9/utdrrXRZqXRaaEyVcA4CE6psgKIYuI8gJFNVVbxeLy+99BIPPfQQ7733Hv/2b/9GfX09b7zxBrNnz2bixIkAzJ07F5vNxltvvcWoUaP46le/yi9/+Ut+8IMfsG/fPgDeffddqqqqWLZsGQ8++CCtra0kEomDzm6nT5/OVVddxRlnnMHjjz9OJpORB0Us/P6jvb2dqVOn8sUvfhGgawpZLBZRFKXbgOvo6EBRFAzDYPr06TzwwAOsWrWK++67j0QiwRe+8AU0TcNisXD66adTLpc5//zzqaqqOui53W43xWJxyE5bi8UixWKR66+/noqKCkKhUNeMx+FwkEqlSCQSVFVVoWka0WiUhoYGnE4n1113HZ/+9Kd57LHHuO6667j77rupqalh48aNjBw5Ervdzty5cw/wFXfyq1/9il27dvGzn/2sz0XMG7w2NkcLjA4O7KB8BSjv19Ij2XRVLpeJx+PccMMNjBo1Cp/P19VGqqrS3t7e9X/T6TSFQgFj/8LGtddey+WXX86TTz7Jd77zHe68807q6+tJp9PU1tYSDAY566yzutx3f8/EiROZOHEi5513Hp/97Gf5yEc+wllnnSVCJRZ+/9BZyi4YDHYT3rFjxzJt2jQWLVrEmjVrWLx4McuWLeOiiy5CVVUefvhh3nrrLaZNm0ZVVRXbt29HVVWmTZvGhg0bCAQCDB8+nGQySUVFRbdzxuNx1q1bx7Zt27rWCnbs2DHk2r5QKJDJZBg2bBgVFRXdBOD8888nGAxy7733sm7dOu6//37S6TSf//znWbt2LQsXLsTv9zNjxgxaWloolUqcd955pFIpotEokydPJpPJHCDm2WyWhx56iBdeeIF58+YRi8VYvnw5kUik1+ut89opaGV2JzRCjgEcsbO/vKFFUY4ocZqmaWYxmKoqQqFQtzb82Mc+xu7du3nwwQd55513WLhwITU1NcyfP5+NGzfyP//zPzgcDmbPnk00GiUWi3HmmWcC0NTUxMSJEymVSthstm79vXPnTm666SZefvllNmzYwCOPPILX66W6ulpE6nBdpRKl07Ml89prr1FdXc1pp512wLR2+vTpbNu2jVdeeYXNmzfzsY99jM997nMA/OUvf+Gll17q8j9/5StfIRwOM2vWLLZs2cKf/vQnXn/9ddrb2zn11FO7xfVv3ryZu+66i0wmQ21tLStWrGDfvn3Mnz9/SLX/5s2b2bNnDwsWLDgg3trn8zFp0iRWrFjBW2+9RTwe5+qrr2bGjBls376dJUuW8Oqrr/L+++/zhS98gdmzZ+PxeAgEAjz33HO8+uqrrFmzhjFjxnTb49Dp4582bRqJRIJXXnmF5cuXM3XqVOrq6nq9ZruqsDGSI+C0UelSKZRNf/lAWolRFSjqBumiRr3PjuUw14l27NjB5s2bmTdv3gHRYw0NDVRXV7N06VJWrVqF3W7nm9/8JvX19ezevZuXX36ZZcuW8c477/DJT36SCy64AIfDQW1tLUuWLOGVV15hxYoVNDQ0dIu6KpfLvPfee7zxxhu8/fbbtLe3c+WVVx7wXAq9T5wVwzCSgE/aojuGYRCLxbDZbF0RCD25fRwOB36//wBrMZlMMmzYsAM+E41GURTloC6FYrFIPB7vig4plUpYLJYe3Q8nK5lMhlwuRygUOmSUTGtrK5WVlQdY6y0tLXg8ngP6rlQqEY1Gu4ULfrjPstksVqsVXdcxDDP9gN/v7/Nmu72pIqtasowNOZle5SRbMuvHDpT1d6sC8aJOLFfglLDrsD+fy+VIp9MEg8EeQ1WLxSKxWOygbdzW1obdbj/AVWkYBm1tbVRVVfXY3+l0mlwuR0VFxSHHhNCz00IEXxD6mURBY1VLFlVVOa3Gi9sGqcLAEH27BZrSGvlSiRlHIPjC4BZ88eELQj8TcFi5cKQflwXebEzSnNYIOEzXzol27VsUKJZlf8dQRQRfEI4RZ9d5mVbpYHVzii3RAl472NUTt6BrAC7LB8IviOALgtCPjPQ7+NhoP02pAn/Zm0ZRwGs/fqLfuWjssYHPDnvTGu+2Z3BI4ZMhifjwBeE48dfmNLGCztQqN3UeK+nS/k1Qx0jobSo4rZDXYE+yQKKgoRuQ1wyGeSxMrxIf/hBDFm2PFE3T0HX9mKZKFnqmWCweEK89GPhbosD69hzjQi4mhBwUy/urT/Wj0NtVcNkgltdpShWJ5jWKZZ16n4PRQQceCyzdnWZalYNhnsPfKFYqlQB6jNIRRPBPGtavX8+KFSuIRCIYhkEgEGDmzJnMmzdPGucYk8vlWLx4Mdu2bSOfz2Oz2Rg+fDgf/ehHB9UmnIKms2xvCkVRmTfch6pApnjkUTydbhuX1axTG8mV2RHPE89rhJxWGnwOqt1WVAWyJfDYYW1LDt0o85G6vmebXLNmDStXruxKVxEIBJgzZw5z5syRwSmCf/KJzW9/+1tWr16Nqqp4PB4URSGXy5HL5ZgyZQpf/vKX+7RBRzh8Vq9ezZNPPklrayt+vx+bzUa5XCaRSBAMBrnkkku48MILB9U9rW3N0JTROCXsod5nJVk0E6/1VfcNzMVXtxV0YFeiSHO6gKqoOK0K9V4HNR4LJd1063S+GCwKlAyD9yJZTq124rEdOqY9lUrxyCOPsHbt2q76BGDuldA0jcmTJ3PNNddI9koR/JOHX/7yl7z++uuMHTu2a2MO0OVS2LVrFw0NDdxyyy3i5ulnNm/ezH/913/hdDoJh8NdG6LAzKqZSCRobW3lG9/4RrdMjIOBHfE8G9rzjKtwMqHCSVHrm4vHqpoWfboETekCHTmNTKlM2GVjbNCJ36GQ06BQPvC7DAMqnbCmLY+FMqdWH7py1L333stbb73F+PHjsVgs3ca+YRjs3r2b8ePHc9NNN8mGqAEu+BKl0wdWrlzJ6tWrGTduHKqqdsvk1yk+o0ePpqmpiSeffFIarB8pl8s88cQTOBwOwuFw1w7YDx/3+/0MGzaMRYsWEYvFBtX9jQ06uWiUn8ZkgTf3pVH3W+wHi+IxMBdhvXazYtW77TlWNiVpy5So9dg5p8HPjGoXiqIQL+zf4fuhz7Lf7eOzw75MmeZ0EXcv1v2SJUtYu3YtkyZNQlGUA8Y+wOjRo9m6dSsvvviiDNgBjgh+HwRn6dKleL3ebsVQ/h5d1wmHw6xZs4ZyuSwN10+8/fbbNDc3U1VV1WNBGF3X8Xq95HI53n777UF3j26bysWjA4QcKi/titOWLRNymmKtG6ZvvzOssjWj8freFOvb0pQNg8mVbs6q9zLcb6NUhtTfuYUMzPw5fju4babbZ9meJFsiWaZWOpgYcvbsMjIM1qxZg9/v7/aSPdj/C4fDvPnmm6RSKRm0AxhJj9wLzc3NtLe34/P5Djnowaz+o2kaK1asoL6+nnK5TH19PS7XB+Fv7e3tXTl6NE2jrq6uW3KwWCxGe3t71/GGhoaDfr4zd8zw4cO7RUs0NzeTSqW6jjc0NHRzMXXmG+/peEtLC8lksqsAyfDhw7u96Pbt20c+n++63+HDh3eLlOk8rihKr593Op00NDR0E+69e/d2RUCFQiH27NmDYRhd7oNDiY7dbmfnzp2DdqydWu3Gb7fwbnuajryDqZVOLKrpf98eKxDLaxR1HY/Nwpigk5BTpVg2F2I7/fPK37l9nFbIlODd9hx5TQcMhnttTKlyovTiOEomk2SzWdxud69t7/F4aG9vZ8eOHV0lKAUR/EFHOp3uKlzSG50LWk888QTlcplYLMbNN9/crfLSCy+8wNNPP019fT3t7e3ceOON3bL+LV++nP/93/+ltraWRCLBv//7vzNlypRun3/mmWeoqqpCURRuu+02amtru44/8cQTvP7661RUVGAYBrfddlu3heQ//vGPLF68mMrKSnRd50c/+lG344sWLWL58uV4PB4qKyv5yU9+0u2F88ADD7B161asVisNDQ38+Mc/7ua37Txut9sJhULcfvvtuN3uruOdRTMAJkyYwK233tp1LJ/Pc8899xCLxcjn83ziE5/A5/PhdDp7fdkCWK3WrpfJYGVs0EGtx8Zf9qXIlnSG+2xsjebRDJ0Rficj/Q6cFshqkCx+IPAflm6HxcyZEy8YbI/lacuWUBUFXTe4eLS/z9eSz+fRdb1Poa+qqlIqlUin0yIaAxhZtO2F9vZ27rjjDmw2W69xx7quk81m+dznPkd9fT2apjFs2LBugtnR0UEsFuuaDQwbNqybIMbjcSKRCHa7nXK5TG1tbVcd187Pd2bTBKirq+t2Xa2traTT6R6Pt7e3k0wmezze1tZGOp1GVVWsVit1dXXdXnZNTU1dtUjtdnu39MIfPq4oChaLhfr6+h4/73A4ur1sdF2nsbGRcrmMrusEg0HeeOMN/vSnP1FXV3dI0VcUhba2NubMmcNVV111Uoy9NxvTtOfKzKrxUOmyYrOY4Zv63wl8p9vGvd98a85o7IznKek6IaeVaredkT4rbzRmcVkNTqvx9NnYueOOOygUCt3GYE9WfiQS4brrrutmoAgDipRY+L0QDodpaGhgy5Yt1NbW9uhH7qyAZbfbu4o6HIzKysoD8oh/mL8vtnK4n6+pqTloWtoP3084HO7xeHV19SFj2nsLOz2a46qqMnz48G7/NnnyZF588cU+zbKKxeJJJTZn1Xl4ZU8ar92Kgemf/7DbpnM3rctqRuO8F83TkSthUxV8dgv1PjdVLtPtk9Ggwe9gazTT5xz9Xq+XQCDA9u3bcbvdhxz78Xic+vp6xo0bJ6IxgJFF2z5wySWXoGlal+Xak4UTi8U4//zzpcH6kfHjxzNu3DhaW1t7DPlTVZVoNEp1dfVBi18P2odTUfDbVd6P5nBZu1v0NhWCDjPz5eZIntXNKZrTRcIuG6fWeJlZ48JjU0kWzJdBvgx1Xgteu5W9yUKfr+Hcc8+lWCxSKpV6HPu6rhOLxbjwwgslJFkEf/AzYcIEPvGJT7Bz504ymUw3S7PTd7lt2zZOO+20rsLaQv/x5S9/GY/Hw+7du7ssys7fiqLQ0tJCLpfj6quv7uY+OxmYXuVE08ukix9ssvLbIVnU+WtThrUtaVLFMmOCLhYM9zOhwkmpDIkCaPoHu3c7a9lWOu1sjfdd8E8//XQ++tGPsnXrVrJZM8d/Z/urqkqxWGT79u3MnTv3kDNbYWAgPvzDYMmSJSxZsoRoNNrl99Y0DafTyfz58/nUpz6FwyHVIo8F+/bt48knn+Tdd98FoLa2lmKxSFtbGxMmTOCKK65g0qRJJ+W9L9+Xwm61MWeYk80dRVoyBfwOC+2ZIhUuB2fWukhrZtz9IR92zMXc1/YkOaXaSYO3b9a4YRi8+OKLLF26tGvsK4pCsVjE5/Nxzjnn8JnPfGbQ5TUagshO28Olo6OD1atXE4/HAfB4PEybNo3Ro0dL4xwHVq9ezd69e1m9ejWhUIizzz6buXPnntT3HMlpvNOWw2dXKZYNqlwWplSaM5k3GzNMrHJjVRT0XgKZDMxY/vc7iuS1EmcM8xzWdbS1tbFmzRqSyWTX2J85c+YB6y59oTkNtV4ZzyL4gtAHbr31VtxuN9dff/2QuN9i2SBe0Kh2d48Ue6spjaJaOL3GRarYhyk9YLXAmuY0p4adBJ3HP25j8d/gNxvht5eaaxHC8RN8aW5hUDJnzpyTzl9/KOwW5QCxBxgbcJAtltH6WLVQx6x6ZVFVtsaObM9CNpsll8sd0WeTBfjNu7A3AS/vlHF8vBHBFwYlF110Ed/85jeHfDvUeGx4bAp7khoua+81cxXMTVtjg05yZdCPoPTWT3/6U5577rkjut6frTYjhhr88OY+Gcci+IIgHBZhl5WdiRyK0rcHuqRDjUfFoqq8Hz08Kz8SibBr1y5GjRp12Nf51HuwfC/Ue82Q0q1R2NIh/SeCLwhCnxkTcKAYBs3pMo4+uOQVzIiearedncniYZ3rlVdewWq1cvrppx/W59a1mq6cMUEzE6hVNV88f9wq/Xc8kZ22h8m2KLzXYWYeFA5Ex1yI89kh7IYR/mN3rqamJtrb2znllFOGdJsrCowO2mnNFmnwucjR+07avAYj/Fb2JC00porU+/oWomm1Wpk7d26fckt10pGDe1aZOX4cFjObp2GY42NDG7RnzT8LIvgDjue2w+0rYFRA2uJgdFpvbps5bR9bAfMaYE49VPfzQ71u3ToeffRRHnjgga4qTEPZyt+9N00kp+Oxqb0u4nZ67ht8Dval+y74V1xxxWGPh4XroTENtR6I5U1xj+chp0FrBp7fDlfNkGdHBH8AMrcBbjrbFDPhQFTFdBdEsuZDvqkdXtsNo4PwmUnmT39x2mmn8fTTT7Nz506mT58+tB9kVdm/eFvktGFOEoVDW/kKkCtBvc/GzniO9qxG2H0M5ECBz0yGz042d/+ua4MXd8DIAFQ4YXcC3tgHHxsDwyQuXwR/oDG7zvwReievQVMa3tgLv9tszow2tcMNZ5oFPY6WmpoadF1n9erVQ17wAaZWulgfyZMrmYtzvcXfGJgulpDLxr508ZgIvkL32bDDAr9YA1+aCpeOM907rRlwiYtUBF8Y3Dit5iLdmCDMGgYPvwvPbDXrsP5kvrnN/2i59NJLpXj2fgIOCyoGO+IFJlc6yJR6/0yuBOOCTja1ZyiUdRyWnn3z69evJxAIHFGETicbI+bLyO/4YEYoO26P4wxcmkA4HkwLwx0L4GNjYcnfzEW8/uAf/uEfOO+886SB91PnsdKYLtDX6HrNgJBTQUNhU6TnzVS6rnPnnXeycuXKo7q+XQnzRe+VpJoi+MJJPp1U4YcfgXOGw6It8Jc90ib9zZigE79NPawQzbwGowNOOvI9Z1/rTFp3NOm/8xr8LQ41HvNHEMEXTnI8Nrh+DgSd8Ngm+pT/RTg8Gnw29qby2FT6ZOnny9DgteCyWdl7kA5JJBLcddddzJo1i6qqqiO+rn0p2BGD8SFz85Uggi8MCSsULh4DK5vMBd2jRdd17rvvPtatWyeNC9R77aQKGi0ZHUcf1kkUzP0TVS4b2+P5g7bvggUL+NKXvnRU19WcNqO3JlVJH4ngC0OKi0eDwwrbYkf/XYqi0NTUxO9+9ztpWMxEa8M8Vvam8ritfbPycxrUe20UygqRnNbtWCgU4pprrmHYsGFHdV27k6Zbb0ZY+kgEXxhSTKuGU6phS4Q+Z3o8lOBfe+21bNq0ieXLl0vjAlMqXZTLBvGCWSmr11mSYS6m1njs7Ev3v5/NAFY3wZgQTKqU/hHBF4YUFsUU/HVtsLkfEmiNGTOGBQsWdJVBHOq4rCpW1WBbLN+nNCDKfit/pN9Ba0YjVTLnBYlEosfi5YfDhlZY0wKTKyUtiQi+MCSp90GmBGub++f7rr/+eq688kpp2P2M8ttJF0u9VsLqpKyb8fFuq4WoTaF19w6+973v0tFx9G/kvzabC/RTxX8vgi8MTaaHoc4L69vosygJfafOa8dtU81c+baeffmdHh+H1dyJqbjcdERy/OpXv2LmzFOpqKg46mvZ1G5G58wbIf0igi8MScaF4Ixa2BSBvUlpj2NBldPCzkQOVemeW0ffn+SuwgEhh/nba4V3YznCXoUdf3mR9Zvf56qrrsJiObot0e+2m7O4OfVQ5ZI+EcEXhixn1UNbBlY19d93plIpbr75ZrZt2zbk23dMwIGu6zSlyl0hmrph+tH9Nnhrd4m7Xs1y4wspvrc4RiRVZqIDhodDfO5LV+JwHH2WwBe2m1W2ZtXIeD/RSC4d4YQyucrMpbJ0N1w6Hlz9MCIdDgcul4t77rmH+++/f0i3r0VVGBNw0JotMtrvwgCcFrAqcM/yDLe/lMa23/w3DHh9i8GjV5Y5c965nNkP5+/ImdkwZw2DjzTIeBcLXxjSjA6YKafXtsDG9v75Trvdzve+9z3i8Th33333oGmLcg/BMLuiZVpS3Q/Gczrrm0psatHY3KqxoUljW7vWo5UfzZbY1FEkmtPYl8nx3ytT/NfLGRoCFsaGLYyttDA+bGFnTOP/LEoQzfXPPS3dZW64uqyfXuaCWPjCIOeScWZhmTf3mT79fhnYVivf/va3Wb9+PYZhoCjKCb/PF7cUuPvVDC6bgtuu4HcqRLM6kYzO7miZy6Y7ufdTH5QI++O7ee5/M8u6Jg27BT4+2cG1Z7k5fbiNVXtKXPlYHKdFoS2jo5XB51S4fLqTmy7wMq7qA7+7zaIwpdLB1ngej02l2quyYkcJVVXwu5SuF41hwIRqC6v3FHl+Y46vnHF0Dvdo3qxjO9IPC2SxVgRfEABOrTF9+X/aZmbTnFjRP987Z84c5syZM2Duc19c55WtRaYMs6JjkCkYuG0KVV4Vm0Xplgbh7tcyXP9UkoqAyvnjHWRKBg+8nKZUhoVfCOCwKhRKcOZIGx+f7KBswGvbi/z6zSw7ImUWfy2Ey/bBS25UwMGowAf+eAsl7FbtgFmFboDHrtKcPPrY+xd3mHssvn3GB+mQBRF8QWDBCHhll1nubuLsk/Mev3KGk0/NcOCwKrywOc8/Pp7gMzNd/PaLAT48AXn23TzX/zHJzFE2Hv1ykKnDzMf0lQUeptSYfy6UIJHTOaXextfONmtH/t+PuLnKDg+vzPHG34pcOLFnlbVaIFsy89F/OCTWokCmaBByHd2MKJY3X+CTK+Hzk2V8DxTEhy8MCC4eC3PqzJTJbZn+//62tjbuvPNOmpqaTtg9OqwKVR4Vn0PB51AplMBmgb/3Nj32dg5KBted4+4Se4Dzx9up9ZuPrMuu4LQpdKS7W+KjKyxQMEgVDr2x4aKJDpJ5g7xmoOwP2VQV2BktM7nGwiVTj84kf+o9Mxzz4jFmQXtBBF8QunBaTHHYHoNHN/X/91ssFt577z1+8IMfkMlkTvj9OqzgcUC22F2Yi2VoTek0VFuZP65n0XVazSRpxbLR5Yr5664Sv1mdp67GyuwRh1bZz8x08m8LPGxs1tgRKbM3XmZTi+niue8KP/WBI4+9b8/CH7earrr+rGEsiOALJxEXjYHZtWbcdmOqf7+7srKS+++/n8rKSv7pn/6Jtra2E/uCsynYLAdG5mSLBnkNfA4Fj71nt4qmwzC/ypZWjbm/6OD0uyOcc18Hu6Ia/3GRl4Zg74/2bR/38vRVIb5wqpNzxzn4znwPb1xXyZkjj84kv28t7E3BV6dDQHz3Awrx4QsDBp8drpkJ31kKD66HW+b2s1XtcPCf//mfPProoyc8aqdUNsX+70vIeuwKbpvCnqhBLKtT4zu4cFv3Fzcp6zCq0kLAqfC5WU4WjLUzpxfBLhQKLFy4kCsuu4SLJ4/g4sn9p8qv7jZ99xeNhvNGypgWwReEQzC3wSyB+Mz7MG94/4uGx+Pha1/72ol/8FRzU1Tp76oK2ixQ41V5bYvGq9uLTKqx9jhDiOcMpteq/O7KYJ/Pm0gkuPHGG9E0jU9+8pP9ek/tWXhovWnVf/1U8x6FgYV0iTDg+NqpUOGC375rpuw9lrS1tR11Ye4jwe9QcVgUitqBi6ufPdUJFoX/fjPLusZS178/vynP7Usz6Aa4bMoBETa9sX79er74xS9SVVXFAw88QH19fb/e0y/fhnWt8JVpMDYk41gEXxD6wMQK+IeJsLoZ/t/aY3uuQqHAPffcw3333YemacftHotlaI2XaUsfGO/+DzOc/MfHfGzcWeTCX0W5+ncJvvRInE/8KsZtL6XIlwycVmht09jZUe6z6Pv9fj7/+c9z66239vv9vLDDXHs5byR8dYaM4YGK5ZZbbrkRkKUVYUAxpQq2xmDZLjPfzgj/sTlPIBBg0qRJPP7447z44ouccsophELH3jxNFQw2tGjMHmE7qA99wTg7dZUWckVYuadIW1rn0mlO/vvTAUZVWMgUwbDCvLF2zhhh77GqVblcRlVNuy4UCnHKKaf0+73sTMBNr4HHDrfOk4yYA5iiYhhGEvBJWwgDjQ3t8PUXodoD//txqDyGQpLNZvn5z3/O5ZdfzpQpUwZUO3RkdBxWBa/j8Baaf/azn6EoCt/+9reP2bWli/DPL8HGCPz0XDhfFmoHMikRfGFA89gmuHulmXLhx/OP77l1Xe+yjgcTS5cu5eGHH6ayspIrr7yS008//Zid6z9eNxfYr5kJ150u43WgC75E6QgDmi9NNXds/nGrmVnzmpnH79zLly/nueee47LLLmP27Nk4nc5B0WbRaJQzzzyTf/7nfz6m5/nf9fD0VtNv/83TZKwOBsTCFwY8kRx862X4WxxuPQcuHH18ztvS0sJjjz3GypUrcTqd/PCHP2T8+PEDq20iEdra2o67G+qPW+GnK8xonP86H4Z5ZJwOBgtfBF8YFGyKmL5iw4B7L4RTqo/fuZubm1m8eDEXXnghDQ0nvoqHYRi0tbWxePFinnnmGWbMmHFMIm96YslO+I/lEHTC/RfB6KCMTxF8Qehn3tgL33/NjNG/Y4EZyXOi2L17Nw8++CBjx45l0qRJTJo06bhE9wA0NjZyyy23EAqFmD9/PrNnzyYcDh+Xc7+0E+5aacb/33kunDZMxqUIviAcI36/Be5cCWOD8JP5J26DTzKZ5KmnnmLDhg3s2rWLc889l29961tdxzVNI5VKEQgEjmjht729nebmZtatW8c777zD17/+dSZOnAhAOp1m+/btzJw587je8/K98MPlZgrlm86CC0bLeBTBF4RjzMIN8D/vQJ0X7joPxp3gXZ27d+9G13VGj/5AAX//+9/zhz/8gdGjR5PP5/nHf/zHbgK9c+dONm7ciNfrZcuWLcydO5cZMz7YsbRw4UKef/55xowZw9ixY7niiiuorq4+Yff48k74yQrTpfYfc+FcCb8clIIvUTrCoOPqGWY65btWwneXwW3zYeoJdO+MHHmg+s2aNYv6+noSiQRbt27F4ei+uer999/nd7/7HXV1dZTLZRYsWNDt+GWXXcYFF1zAiBEnvjbg7zabO56dVvjJArNugTA4EQtfGLT8diPc/zbUeuG7Z8KZg0iIDMNA1820ChaLZcBe50PrzMyl1W74wdlwZr2Mu8Fs4YvgC4OaRVvg56tNi//7Z5k59YWjJ6fBXX814+wnVsDNH4HpYWkXEXxBOMG8vMuMCc+WTHfP8dycdTKyqR1+sRbe2mf66n9wNtRInL0IviAMFDa2w0//aqbnvXQc/NOp5s5c4fD441ZzUbwxZe5y/r+zTN+9IIIvCAOK1gzcvQqW7oRxFfAvp5nFVITeiebhN+/CYxuhym2WJ/zCFGkXEXxBGOD8egM8uA5UBT41Ea6dCX67tEtPLN9jtte6NjijFm44EyZXSruI4AvCIOHNfabF+sY+OLsBPj0RLpYF3W7sTsIf3jOzXQJ8fgp8cSqEnNI2IviCMMiI5uHxTfDMVkgVzCpaF4+BmTVDu12SRXMj1e+3mGsf54403TcfaZAxI4IvCIOcd1rNVL5vNkKFA66YBFdMHJqRJ3/ZA49uMstH1vvgsnHwf04BixQ7FcEXhJOFkg5L/mZate+0monXzhkOHx8LY4In//0v+ZuZ+GxFIxjApyaYLz0pNi6CLwgnLW0ZeG47vLYH1rfCyCBcNh5m1cCskyzzYywPq5rMe31lF9gscMEoWDASzh0hY0EEXxCGCJEc/Gmbubi7vhW8dpg73BTESZWDu6DHjhisaTEt+rUtEHabaScuGgNzxU8vgi+CLwxVchq8uhsW/80USU03XTyn1phJwubUgWsQbDyK5s2X1xt7zeLvzWmo9ZjVwS4ac2KTywki+IIwoCiW4e1WWN9mphTYHDEXMk+pNsVyhB8mVprWvzIArtcANrfD+1HYHoMNbbClA9y2D15UM6rNPDiCIIIvCD3QkYO3Gk0R3bZfUDMlqPPBpApzwXfifrdPyGn+qMf4LZAsmFZ8W9YU+g3t5kupJWOef0KFeW1z6syMlqoi/SiI4AvC4QltEd7vMCN7NkdgVwLas1DWIeAw0xDUes0Qz5F+8+9Vbqh0mq4gp9VcLLX1EvZY1qFQhnwZ0kVTyDuysCcJTWnYmzT/LZYHFQh7TNfT1DDMrjUF32GR/hJE8AWhX9B0U3RbM6YQb4nA3hTsS0E8b2brBPA7wGPb/2MHl8UUf0Uxhd+imi4ZDCjqZn3YgmbOIlJF83eyYP52WMBnN2PmRwVNK35chfmSqfWIJS+I4AvCcSOnQUsaGtOmuyW3X6yzJXN2kMhDWoNM0SwPWNKhbJifVQC7xRRtt80U9gon+Bxm7h+fHYJOaPCZ5RyDkvJAEMEXhIFLWTfdNAXNtOqLZXOmoCim4DuspovGsd/9YxGLXTiGgi+ZrgXhGGJRwaOa7h1BONFIBg1BEAQRfEEQBEEEXxAEQRDBFwRBEETwBUEQBBF8QRAEQQRfEARB6BcUwzAMaQZBEISTHyvQhOy0FQRBONlJ/f8BABudSjmpQgUZAAAAAElFTkSuQmCC

    :param seg_list: list[posb] - List of posb segments.
    :param vel: float or float[2] - linear velocity or [linear velocity, angular velocity].
    :param acc: float or float[2] - linear acceleration or [linear acceleration, angular acceleration].
    :param time: float - Reach time [sec].
    :param ref: reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference). Default: DR_BASE.
    :param mod: int - Movement basis (DR_MV_MOD_ABS: Absolute, DR_MV_MOD_REL: Relative). Default: DR_MV_MOD_ABS.
    :param v: float or float[2] - linear velocity or [linear velocity, angular velocity].
    :param a: float or float[2] - linear acceleration or [linear acceleration, angular acceleration].
    :param t: float - Reach time [sec].

    :return: int - (0 -> Success, Negative value -> Error)
    """

    if _robodk_plugin_RDK is not None:
        for p in seg_list:
            p_list = p.to_list()
            seg_type = p_list[0]
            x1 = p_list[1]
            x2 = p_list[2]
            radius = p_list[3]
            if seg_type == DR_LINE:
                movel(x1, vel=vel, acc=acc, radius=radius, ref=ref, v=v, a=a, t=t)
            if seg_type == DR_CIRCLE:
                movec(x1,x2, vel=vel, acc=acc, radius=radius, ref=ref, v=v, a=a, t=t)
    return 0


def amoveb(seg_list, vel=None, acc=None, time=None, ref=None, mod=DR_MV_MOD_ABS, v=None, a=None, t=None) -> int:
    """
    The asynchronous moveb motion operates in the same way as moveb() except for the asynchronous processing and executes
    the next line after the command is executed. Generating a  new  command  for  the  motion  before  the  amoveb()
    motion  results  in  an  error  for safety reasons.
    Therefore,  the  termination  of  the  amoveb()  motion  must  be  confirmed  using  mwait()  or check_motion()
    between amoveb() and the following motion command.

    .. image:: data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAXwAAAC0CAYAAACXK5enAAAACXBIWXMAAAsTAAALEwEAmpwYAAA4IGlUWHRYTUw6Y29tLmFkb2JlLnhtcAAAAAAAPD94cGFja2V0IGJlZ2luPSLvu78iIGlkPSJXNU0wTXBDZWhpSHpyZVN6TlRjemtjOWQiPz4KPHg6eG1wbWV0YSB4bWxuczp4PSJhZG9iZTpuczptZXRhLyIgeDp4bXB0az0iQWRvYmUgWE1QIENvcmUgNS41LWMwMjEgNzkuMTU0OTExLCAyMDEzLzEwLzI5LTExOjQ3OjE2ICAgICAgICAiPgogICA8cmRmOlJERiB4bWxuczpyZGY9Imh0dHA6Ly93d3cudzMub3JnLzE5OTkvMDIvMjItcmRmLXN5bnRheC1ucyMiPgogICAgICA8cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0iIgogICAgICAgICAgICB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmRjPSJodHRwOi8vcHVybC5vcmcvZGMvZWxlbWVudHMvMS4xLyIKICAgICAgICAgICAgeG1sbnM6cGhvdG9zaG9wPSJodHRwOi8vbnMuYWRvYmUuY29tL3Bob3Rvc2hvcC8xLjAvIgogICAgICAgICAgICB4bWxuczp4bXBNTT0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wL21tLyIKICAgICAgICAgICAgeG1sbnM6c3RFdnQ9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9zVHlwZS9SZXNvdXJjZUV2ZW50IyIKICAgICAgICAgICAgeG1sbnM6dGlmZj0iaHR0cDovL25zLmFkb2JlLmNvbS90aWZmLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmV4aWY9Imh0dHA6Ly9ucy5hZG9iZS5jb20vZXhpZi8xLjAvIj4KICAgICAgICAgPHhtcDpDcmVhdG9yVG9vbD5BZG9iZSBQaG90b3Nob3AgQ0MgKE1hY2ludG9zaCk8L3htcDpDcmVhdG9yVG9vbD4KICAgICAgICAgPHhtcDpDcmVhdGVEYXRlPjIwMTctMTEtMjBUMTY6NDU6NTArMDk6MDA8L3htcDpDcmVhdGVEYXRlPgogICAgICAgICA8eG1wOk1vZGlmeURhdGU+MjAxNy0xMi0xM1QxNzo1MDo0NSswOTowMDwveG1wOk1vZGlmeURhdGU+CiAgICAgICAgIDx4bXA6TWV0YWRhdGFEYXRlPjIwMTctMTItMTNUMTc6NTA6NDUrMDk6MDA8L3htcDpNZXRhZGF0YURhdGU+CiAgICAgICAgIDxkYzpmb3JtYXQ+aW1hZ2UvcG5nPC9kYzpmb3JtYXQ+CiAgICAgICAgIDxwaG90b3Nob3A6Q29sb3JNb2RlPjM8L3Bob3Rvc2hvcDpDb2xvck1vZGU+CiAgICAgICAgIDx4bXBNTTpJbnN0YW5jZUlEPnhtcC5paWQ6NjhkZGFlMTQtODJjNi00MDZhLTkyMTEtNDFiN2IwOGVjYmFlPC94bXBNTTpJbnN0YW5jZUlEPgogICAgICAgICA8eG1wTU06RG9jdW1lbnRJRD54bXAuZGlkOjY4ZGRhZTE0LTgyYzYtNDA2YS05MjExLTQxYjdiMDhlY2JhZTwveG1wTU06RG9jdW1lbnRJRD4KICAgICAgICAgPHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD54bXAuZGlkOjY4ZGRhZTE0LTgyYzYtNDA2YS05MjExLTQxYjdiMDhlY2JhZTwveG1wTU06T3JpZ2luYWxEb2N1bWVudElEPgogICAgICAgICA8eG1wTU06SGlzdG9yeT4KICAgICAgICAgICAgPHJkZjpTZXE+CiAgICAgICAgICAgICAgIDxyZGY6bGkgcmRmOnBhcnNlVHlwZT0iUmVzb3VyY2UiPgogICAgICAgICAgICAgICAgICA8c3RFdnQ6YWN0aW9uPmNyZWF0ZWQ8L3N0RXZ0OmFjdGlvbj4KICAgICAgICAgICAgICAgICAgPHN0RXZ0Omluc3RhbmNlSUQ+eG1wLmlpZDo2OGRkYWUxNC04MmM2LTQwNmEtOTIxMS00MWI3YjA4ZWNiYWU8L3N0RXZ0Omluc3RhbmNlSUQ+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDp3aGVuPjIwMTctMTEtMjBUMTY6NDU6NTArMDk6MDA8L3N0RXZ0OndoZW4+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDpzb2Z0d2FyZUFnZW50PkFkb2JlIFBob3Rvc2hvcCBDQyAoTWFjaW50b3NoKTwvc3RFdnQ6c29mdHdhcmVBZ2VudD4KICAgICAgICAgICAgICAgPC9yZGY6bGk+CiAgICAgICAgICAgIDwvcmRmOlNlcT4KICAgICAgICAgPC94bXBNTTpIaXN0b3J5PgogICAgICAgICA8dGlmZjpPcmllbnRhdGlvbj4xPC90aWZmOk9yaWVudGF0aW9uPgogICAgICAgICA8dGlmZjpYUmVzb2x1dGlvbj43MjAwMDAvMTAwMDA8L3RpZmY6WFJlc29sdXRpb24+CiAgICAgICAgIDx0aWZmOllSZXNvbHV0aW9uPjcyMDAwMC8xMDAwMDwvdGlmZjpZUmVzb2x1dGlvbj4KICAgICAgICAgPHRpZmY6UmVzb2x1dGlvblVuaXQ+MjwvdGlmZjpSZXNvbHV0aW9uVW5pdD4KICAgICAgICAgPGV4aWY6Q29sb3JTcGFjZT42NTUzNTwvZXhpZjpDb2xvclNwYWNlPgogICAgICAgICA8ZXhpZjpQaXhlbFhEaW1lbnNpb24+MzgwPC9leGlmOlBpeGVsWERpbWVuc2lvbj4KICAgICAgICAgPGV4aWY6UGl4ZWxZRGltZW5zaW9uPjE4MDwvZXhpZjpQaXhlbFlEaW1lbnNpb24+CiAgICAgIDwvcmRmOkRlc2NyaXB0aW9uPgogICA8L3JkZjpSREY+CjwveDp4bXBtZXRhPgogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAg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AF599VVefvll7HY7NTU13WbNixYtYsWKFWZKaZ8Pi8Vc8l62bBkvvPAC4XCYUCh0QL+ceeaZXHrppcyePZvKykqeeOIJLrzwwgP6UDgkRbHwD4FhGN1EvHMK6Xa7+etf/8qNN97Itm3bePnll/nWt77Fnj17AHjsscdYuHAhDoeDp556ijfeeKNLxG644QZ27drFSy+9xD333HOAlaKqKhaLBbvdfkxnEoPGItkvCIFAAEVRcDqdAPz2t7/lJz/5CXv37mXhwoXccccdADQ3N3PTTTcRiUTYs2cPjz/+eFc73n777Tz77LM0NTXx/e9/n23btnU7l9frZcKECV1/d7lcWK3Wrms4GDaLis+usjWWx20dHL58FSiUjzxDpmEYXZXA7HY7FosFn88HwI9+9CN+/etfs2fPHu68804eeeQRADo6Orj55pvZvn07HR0dPPzwwxQKZsmvX/7ylzz66KO0tLTwwx/+kA0bNhxwzs4U1alUilWrVjFr1iyCwaCI1GEiPvxD4PP5aG5uZuXKleRyOX7/+99zzjnnoCgKCxcuZObMmfzrv/4rmUyGf/mXf2HRokXccMMNrFq1CofDwTe+8Q0ACoUChUKB+++/n9mzZ/PNb36TXC7Hl770JebNm8fpp59+wLlLpRLKEF5Qs1gsWK1WXnvtNTweD8uWLQPg4osvZvv27SxatIhvfOMbXHrppbz99tvcfPPNXHLJJeTzedatW8e1117LtGnTur5v6dKlrFu3jkceeQS73c7NN9/M888/z3e+852Dnj8Wi/HnP/+ZT37yk12zs54Y4bOzqjVLTnNhUcwY9wFtyGCmVbAfgeArioLL5WLFihWEw2FWrVpFJBLh0ksv5aWXXmLVqlXcddddTJw4kT/84Q88/PDDfOITnyCVSvHmm29y++23c84553R937p163j11Ve59957qa+v5+c//zlPPfUUM2bMOODcTz31FI8//jgWi4V77723134RDv6yF3ogEAiwe/dufv3rX/Ob3/yGqVOn8rWvfY3GxkZaWlq6prAej4fzzz+f7du3YxgG11xzDalUiq9+9as8++yzOBwOSqUS5XIZVVV56KGHePjhh1EUhWg0Kg19MMvZZsNqtfL000/z4IMP0tLSwne/+10CgQCvvfYaFRUVXHTRRQDMmjWLCRMm8MorrzB9+nSuuOIKbrvtNm677bau9t20aRN1dXU8+eSTPPDAAySTyUOuj/ziF7/AYrHw6U9/utdrrXRZqXRaaEyVcA4CE6psgKIYuI8gJFNVVbxeLy+99BIPPfQQ7733Hv/2b/9GfX09b7zxBrNnz2bixIkAzJ07F5vNxltvvcWoUaP46le/yi9/+Ut+8IMfsG/fPgDeffddqqqqWLZsGQ8++CCtra0kEomDzm6nT5/OVVddxRlnnMHjjz9OJpORB0Us/P6jvb2dqVOn8sUvfhGgawpZLBZRFKXbgOvo6EBRFAzDYPr06TzwwAOsWrWK++67j0QiwRe+8AU0TcNisXD66adTLpc5//zzqaqqOui53W43xWJxyE5bi8UixWKR66+/noqKCkKhUNeMx+FwkEqlSCQSVFVVoWka0WiUhoYGnE4n1113HZ/+9Kd57LHHuO6667j77rupqalh48aNjBw5Ervdzty5cw/wFXfyq1/9il27dvGzn/2sz0XMG7w2NkcLjA4O7KB8BSjv19Ij2XRVLpeJx+PccMMNjBo1Cp/P19VGqqrS3t7e9X/T6TSFQgFj/8LGtddey+WXX86TTz7Jd77zHe68807q6+tJp9PU1tYSDAY566yzutx3f8/EiROZOHEi5513Hp/97Gf5yEc+wllnnSVCJRZ+/9BZyi4YDHYT3rFjxzJt2jQWLVrEmjVrWLx4McuWLeOiiy5CVVUefvhh3nrrLaZNm0ZVVRXbt29HVVWmTZvGhg0bCAQCDB8+nGQySUVFRbdzxuNx1q1bx7Zt27rWCnbs2DHk2r5QKJDJZBg2bBgVFRXdBOD8888nGAxy7733sm7dOu6//37S6TSf//znWbt2LQsXLsTv9zNjxgxaWloolUqcd955pFIpotEokydPJpPJHCDm2WyWhx56iBdeeIF58+YRi8VYvnw5kUik1+ut89opaGV2JzRCjgEcsbO/vKFFUY4ocZqmaWYxmKoqQqFQtzb82Mc+xu7du3nwwQd55513WLhwITU1NcyfP5+NGzfyP//zPzgcDmbPnk00GiUWi3HmmWcC0NTUxMSJEymVSthstm79vXPnTm666SZefvllNmzYwCOPPILX66W6ulpE6nBdpRKl07Ml89prr1FdXc1pp512wLR2+vTpbNu2jVdeeYXNmzfzsY99jM997nMA/OUvf+Gll17q8j9/5StfIRwOM2vWLLZs2cKf/vQnXn/9ddrb2zn11FO7xfVv3ryZu+66i0wmQ21tLStWrGDfvn3Mnz9/SLX/5s2b2bNnDwsWLDgg3trn8zFp0iRWrFjBW2+9RTwe5+qrr2bGjBls376dJUuW8Oqrr/L+++/zhS98gdmzZ+PxeAgEAjz33HO8+uqrrFmzhjFjxnTb49Dp4582bRqJRIJXXnmF5cuXM3XqVOrq6nq9ZruqsDGSI+C0UelSKZRNf/lAWolRFSjqBumiRr3PjuUw14l27NjB5s2bmTdv3gHRYw0NDVRXV7N06VJWrVqF3W7nm9/8JvX19ezevZuXX36ZZcuW8c477/DJT36SCy64AIfDQW1tLUuWLOGVV15hxYoVNDQ0dIu6KpfLvPfee7zxxhu8/fbbtLe3c+WVVx7wXAq9T5wVwzCSgE/aojuGYRCLxbDZbF0RCD25fRwOB36//wBrMZlMMmzYsAM+E41GURTloC6FYrFIPB7vig4plUpYLJYe3Q8nK5lMhlwuRygUOmSUTGtrK5WVlQdY6y0tLXg8ngP6rlQqEY1Gu4ULfrjPstksVqsVXdcxDDP9gN/v7/Nmu72pIqtasowNOZle5SRbMuvHDpT1d6sC8aJOLFfglLDrsD+fy+VIp9MEg8EeQ1WLxSKxWOygbdzW1obdbj/AVWkYBm1tbVRVVfXY3+l0mlwuR0VFxSHHhNCz00IEXxD6mURBY1VLFlVVOa3Gi9sGqcLAEH27BZrSGvlSiRlHIPjC4BZ88eELQj8TcFi5cKQflwXebEzSnNYIOEzXzol27VsUKJZlf8dQRQRfEI4RZ9d5mVbpYHVzii3RAl472NUTt6BrAC7LB8IviOALgtCPjPQ7+NhoP02pAn/Zm0ZRwGs/fqLfuWjssYHPDnvTGu+2Z3BI4ZMhifjwBeE48dfmNLGCztQqN3UeK+nS/k1Qx0jobSo4rZDXYE+yQKKgoRuQ1wyGeSxMrxIf/hBDFm2PFE3T0HX9mKZKFnqmWCweEK89GPhbosD69hzjQi4mhBwUy/urT/Wj0NtVcNkgltdpShWJ5jWKZZ16n4PRQQceCyzdnWZalYNhnsPfKFYqlQB6jNIRRPBPGtavX8+KFSuIRCIYhkEgEGDmzJnMmzdPGucYk8vlWLx4Mdu2bSOfz2Oz2Rg+fDgf/ehHB9UmnIKms2xvCkVRmTfch6pApnjkUTydbhuX1axTG8mV2RHPE89rhJxWGnwOqt1WVAWyJfDYYW1LDt0o85G6vmebXLNmDStXruxKVxEIBJgzZw5z5syRwSmCf/KJzW9/+1tWr16Nqqp4PB4URSGXy5HL5ZgyZQpf/vKX+7RBRzh8Vq9ezZNPPklrayt+vx+bzUa5XCaRSBAMBrnkkku48MILB9U9rW3N0JTROCXsod5nJVk0E6/1VfcNzMVXtxV0YFeiSHO6gKqoOK0K9V4HNR4LJd1063S+GCwKlAyD9yJZTq124rEdOqY9lUrxyCOPsHbt2q76BGDuldA0jcmTJ3PNNddI9koR/JOHX/7yl7z++uuMHTu2a2MO0OVS2LVrFw0NDdxyyy3i5ulnNm/ezH/913/hdDoJh8NdG6LAzKqZSCRobW3lG9/4RrdMjIOBHfE8G9rzjKtwMqHCSVHrm4vHqpoWfboETekCHTmNTKlM2GVjbNCJ36GQ06BQPvC7DAMqnbCmLY+FMqdWH7py1L333stbb73F+PHjsVgs3ca+YRjs3r2b8ePHc9NNN8mGqAEu+BKl0wdWrlzJ6tWrGTduHKqqdsvk1yk+o0ePpqmpiSeffFIarB8pl8s88cQTOBwOwuFw1w7YDx/3+/0MGzaMRYsWEYvFBtX9jQ06uWiUn8ZkgTf3pVH3W+wHi+IxMBdhvXazYtW77TlWNiVpy5So9dg5p8HPjGoXiqIQL+zf4fuhz7Lf7eOzw75MmeZ0EXcv1v2SJUtYu3YtkyZNQlGUA8Y+wOjRo9m6dSsvvviiDNgBjgh+HwRn6dKleL3ebsVQ/h5d1wmHw6xZs4ZyuSwN10+8/fbbNDc3U1VV1WNBGF3X8Xq95HI53n777UF3j26bysWjA4QcKi/titOWLRNymmKtG6ZvvzOssjWj8freFOvb0pQNg8mVbs6q9zLcb6NUhtTfuYUMzPw5fju4babbZ9meJFsiWaZWOpgYcvbsMjIM1qxZg9/v7/aSPdj/C4fDvPnmm6RSKRm0AxhJj9wLzc3NtLe34/P5Djnowaz+o2kaK1asoL6+nnK5TH19PS7XB+Fv7e3tXTl6NE2jrq6uW3KwWCxGe3t71/GGhoaDfr4zd8zw4cO7RUs0NzeTSqW6jjc0NHRzMXXmG+/peEtLC8lksqsAyfDhw7u96Pbt20c+n++63+HDh3eLlOk8rihKr593Op00NDR0E+69e/d2RUCFQiH27NmDYRhd7oNDiY7dbmfnzp2DdqydWu3Gb7fwbnuajryDqZVOLKrpf98eKxDLaxR1HY/Nwpigk5BTpVg2F2I7/fPK37l9nFbIlODd9hx5TQcMhnttTKlyovTiOEomk2SzWdxud69t7/F4aG9vZ8eOHV0lKAUR/EFHOp3uKlzSG50LWk888QTlcplYLMbNN9/crfLSCy+8wNNPP019fT3t7e3ceOON3bL+LV++nP/93/+ltraWRCLBv//7vzNlypRun3/mmWeoqqpCURRuu+02amtru44/8cQTvP7661RUVGAYBrfddlu3heQ//vGPLF68mMrKSnRd50c/+lG344sWLWL58uV4PB4qKyv5yU9+0u2F88ADD7B161asVisNDQ38+Mc/7ua37Txut9sJhULcfvvtuN3uruOdRTMAJkyYwK233tp1LJ/Pc8899xCLxcjn83ziE5/A5/PhdDp7fdkCWK3WrpfJYGVs0EGtx8Zf9qXIlnSG+2xsjebRDJ0Rficj/Q6cFshqkCx+IPAflm6HxcyZEy8YbI/lacuWUBUFXTe4eLS/z9eSz+fRdb1Poa+qqlIqlUin0yIaAxhZtO2F9vZ27rjjDmw2W69xx7quk81m+dznPkd9fT2apjFs2LBugtnR0UEsFuuaDQwbNqybIMbjcSKRCHa7nXK5TG1tbVcd187Pd2bTBKirq+t2Xa2traTT6R6Pt7e3k0wmezze1tZGOp1GVVWsVit1dXXdXnZNTU1dtUjtdnu39MIfPq4oChaLhfr6+h4/73A4ur1sdF2nsbGRcrmMrusEg0HeeOMN/vSnP1FXV3dI0VcUhba2NubMmcNVV111Uoy9NxvTtOfKzKrxUOmyYrOY4Zv63wl8p9vGvd98a85o7IznKek6IaeVaredkT4rbzRmcVkNTqvx9NnYueOOOygUCt3GYE9WfiQS4brrrutmoAgDipRY+L0QDodpaGhgy5Yt1NbW9uhH7qyAZbfbu4o6HIzKysoD8oh/mL8vtnK4n6+pqTloWtoP3084HO7xeHV19SFj2nsLOz2a46qqMnz48G7/NnnyZF588cU+zbKKxeJJJTZn1Xl4ZU8ar92Kgemf/7DbpnM3rctqRuO8F83TkSthUxV8dgv1PjdVLtPtk9Ggwe9gazTT5xz9Xq+XQCDA9u3bcbvdhxz78Xic+vp6xo0bJ6IxgJFF2z5wySWXoGlal+Xak4UTi8U4//zzpcH6kfHjxzNu3DhaW1t7DPlTVZVoNEp1dfVBi18P2odTUfDbVd6P5nBZu1v0NhWCDjPz5eZIntXNKZrTRcIuG6fWeJlZ48JjU0kWzJdBvgx1Xgteu5W9yUKfr+Hcc8+lWCxSKpV6HPu6rhOLxbjwwgslJFkEf/AzYcIEPvGJT7Bz504ymUw3S7PTd7lt2zZOO+20rsLaQv/x5S9/GY/Hw+7du7ssys7fiqLQ0tJCLpfj6quv7uY+OxmYXuVE08ukix9ssvLbIVnU+WtThrUtaVLFMmOCLhYM9zOhwkmpDIkCaPoHu3c7a9lWOu1sjfdd8E8//XQ++tGPsnXrVrJZM8d/Z/urqkqxWGT79u3MnTv3kDNbYWAgPvzDYMmSJSxZsoRoNNrl99Y0DafTyfz58/nUpz6FwyHVIo8F+/bt48knn+Tdd98FoLa2lmKxSFtbGxMmTOCKK65g0qRJJ+W9L9+Xwm61MWeYk80dRVoyBfwOC+2ZIhUuB2fWukhrZtz9IR92zMXc1/YkOaXaSYO3b9a4YRi8+OKLLF26tGvsK4pCsVjE5/Nxzjnn8JnPfGbQ5TUagshO28Olo6OD1atXE4/HAfB4PEybNo3Ro0dL4xwHVq9ezd69e1m9ejWhUIizzz6buXPnntT3HMlpvNOWw2dXKZYNqlwWplSaM5k3GzNMrHJjVRT0XgKZDMxY/vc7iuS1EmcM8xzWdbS1tbFmzRqSyWTX2J85c+YB6y59oTkNtV4ZzyL4gtAHbr31VtxuN9dff/2QuN9i2SBe0Kh2d48Ue6spjaJaOL3GRarYhyk9YLXAmuY0p4adBJ3HP25j8d/gNxvht5eaaxHC8RN8aW5hUDJnzpyTzl9/KOwW5QCxBxgbcJAtltH6WLVQx6x6ZVFVtsaObM9CNpsll8sd0WeTBfjNu7A3AS/vlHF8vBHBFwYlF110Ed/85jeHfDvUeGx4bAp7khoua+81cxXMTVtjg05yZdCPoPTWT3/6U5577rkjut6frTYjhhr88OY+Gcci+IIgHBZhl5WdiRyK0rcHuqRDjUfFoqq8Hz08Kz8SibBr1y5GjRp12Nf51HuwfC/Ue82Q0q1R2NIh/SeCLwhCnxkTcKAYBs3pMo4+uOQVzIiearedncniYZ3rlVdewWq1cvrppx/W59a1mq6cMUEzE6hVNV88f9wq/Xc8kZ22h8m2KLzXYWYeFA5Ex1yI89kh7IYR/mN3rqamJtrb2znllFOGdJsrCowO2mnNFmnwucjR+07avAYj/Fb2JC00porU+/oWomm1Wpk7d26fckt10pGDe1aZOX4cFjObp2GY42NDG7RnzT8LIvgDjue2w+0rYFRA2uJgdFpvbps5bR9bAfMaYE49VPfzQ71u3ToeffRRHnjgga4qTEPZyt+9N00kp+Oxqb0u4nZ67ht8Dval+y74V1xxxWGPh4XroTENtR6I5U1xj+chp0FrBp7fDlfNkGdHBH8AMrcBbjrbFDPhQFTFdBdEsuZDvqkdXtsNo4PwmUnmT39x2mmn8fTTT7Nz506mT58+tB9kVdm/eFvktGFOEoVDW/kKkCtBvc/GzniO9qxG2H0M5ECBz0yGz042d/+ua4MXd8DIAFQ4YXcC3tgHHxsDwyQuXwR/oDG7zvwReievQVMa3tgLv9tszow2tcMNZ5oFPY6WmpoadF1n9erVQ17wAaZWulgfyZMrmYtzvcXfGJgulpDLxr508ZgIvkL32bDDAr9YA1+aCpeOM907rRlwiYtUBF8Y3Dit5iLdmCDMGgYPvwvPbDXrsP5kvrnN/2i59NJLpXj2fgIOCyoGO+IFJlc6yJR6/0yuBOOCTja1ZyiUdRyWnn3z69evJxAIHFGETicbI+bLyO/4YEYoO26P4wxcmkA4HkwLwx0L4GNjYcnfzEW8/uAf/uEfOO+886SB91PnsdKYLtDX6HrNgJBTQUNhU6TnzVS6rnPnnXeycuXKo7q+XQnzRe+VpJoi+MJJPp1U4YcfgXOGw6It8Jc90ib9zZigE79NPawQzbwGowNOOvI9Z1/rTFp3NOm/8xr8LQ41HvNHEMEXTnI8Nrh+DgSd8Ngm+pT/RTg8Gnw29qby2FT6ZOnny9DgteCyWdl7kA5JJBLcddddzJo1i6qqqiO+rn0p2BGD8SFz85Uggi8MCSsULh4DK5vMBd2jRdd17rvvPtatWyeNC9R77aQKGi0ZHUcf1kkUzP0TVS4b2+P5g7bvggUL+NKXvnRU19WcNqO3JlVJH4ngC0OKi0eDwwrbYkf/XYqi0NTUxO9+9ztpWMxEa8M8Vvam8ritfbPycxrUe20UygqRnNbtWCgU4pprrmHYsGFHdV27k6Zbb0ZY+kgEXxhSTKuGU6phS4Q+Z3o8lOBfe+21bNq0ieXLl0vjAlMqXZTLBvGCWSmr11mSYS6m1njs7Ev3v5/NAFY3wZgQTKqU/hHBF4YUFsUU/HVtsLkfEmiNGTOGBQsWdJVBHOq4rCpW1WBbLN+nNCDKfit/pN9Ba0YjVTLnBYlEosfi5YfDhlZY0wKTKyUtiQi+MCSp90GmBGub++f7rr/+eq688kpp2P2M8ttJF0u9VsLqpKyb8fFuq4WoTaF19w6+973v0tFx9G/kvzabC/RTxX8vgi8MTaaHoc4L69vosygJfafOa8dtU81c+baeffmdHh+H1dyJqbjcdERy/OpXv2LmzFOpqKg46mvZ1G5G58wbIf0igi8MScaF4Ixa2BSBvUlpj2NBldPCzkQOVemeW0ffn+SuwgEhh/nba4V3YznCXoUdf3mR9Zvf56qrrsJiObot0e+2m7O4OfVQ5ZI+EcEXhixn1UNbBlY19d93plIpbr75ZrZt2zbk23dMwIGu6zSlyl0hmrph+tH9Nnhrd4m7Xs1y4wspvrc4RiRVZqIDhodDfO5LV+JwHH2WwBe2m1W2ZtXIeD/RSC4d4YQyucrMpbJ0N1w6Hlz9MCIdDgcul4t77rmH+++/f0i3r0VVGBNw0JotMtrvwgCcFrAqcM/yDLe/lMa23/w3DHh9i8GjV5Y5c965nNkP5+/ImdkwZw2DjzTIeBcLXxjSjA6YKafXtsDG9v75Trvdzve+9z3i8Th33333oGmLcg/BMLuiZVpS3Q/Gczrrm0psatHY3KqxoUljW7vWo5UfzZbY1FEkmtPYl8nx3ytT/NfLGRoCFsaGLYyttDA+bGFnTOP/LEoQzfXPPS3dZW64uqyfXuaCWPjCIOeScWZhmTf3mT79fhnYVivf/va3Wb9+PYZhoCjKCb/PF7cUuPvVDC6bgtuu4HcqRLM6kYzO7miZy6Y7ufdTH5QI++O7ee5/M8u6Jg27BT4+2cG1Z7k5fbiNVXtKXPlYHKdFoS2jo5XB51S4fLqTmy7wMq7qA7+7zaIwpdLB1ngej02l2quyYkcJVVXwu5SuF41hwIRqC6v3FHl+Y46vnHF0Dvdo3qxjO9IPC2SxVgRfEABOrTF9+X/aZmbTnFjRP987Z84c5syZM2Duc19c55WtRaYMs6JjkCkYuG0KVV4Vm0Xplgbh7tcyXP9UkoqAyvnjHWRKBg+8nKZUhoVfCOCwKhRKcOZIGx+f7KBswGvbi/z6zSw7ImUWfy2Ey/bBS25UwMGowAf+eAsl7FbtgFmFboDHrtKcPPrY+xd3mHssvn3GB+mQBRF8QWDBCHhll1nubuLsk/Mev3KGk0/NcOCwKrywOc8/Pp7gMzNd/PaLAT48AXn23TzX/zHJzFE2Hv1ykKnDzMf0lQUeptSYfy6UIJHTOaXextfONmtH/t+PuLnKDg+vzPHG34pcOLFnlbVaIFsy89F/OCTWokCmaBByHd2MKJY3X+CTK+Hzk2V8DxTEhy8MCC4eC3PqzJTJbZn+//62tjbuvPNOmpqaTtg9OqwKVR4Vn0PB51AplMBmgb/3Nj32dg5KBted4+4Se4Dzx9up9ZuPrMuu4LQpdKS7W+KjKyxQMEgVDr2x4aKJDpJ5g7xmoOwP2VQV2BktM7nGwiVTj84kf+o9Mxzz4jFmQXtBBF8QunBaTHHYHoNHN/X/91ssFt577z1+8IMfkMlkTvj9OqzgcUC22F2Yi2VoTek0VFuZP65n0XVazSRpxbLR5Yr5664Sv1mdp67GyuwRh1bZz8x08m8LPGxs1tgRKbM3XmZTi+niue8KP/WBI4+9b8/CH7earrr+rGEsiOALJxEXjYHZtWbcdmOqf7+7srKS+++/n8rKSv7pn/6Jtra2E/uCsynYLAdG5mSLBnkNfA4Fj71nt4qmwzC/ypZWjbm/6OD0uyOcc18Hu6Ia/3GRl4Zg74/2bR/38vRVIb5wqpNzxzn4znwPb1xXyZkjj84kv28t7E3BV6dDQHz3Awrx4QsDBp8drpkJ31kKD66HW+b2s1XtcPCf//mfPProoyc8aqdUNsX+70vIeuwKbpvCnqhBLKtT4zu4cFv3Fzcp6zCq0kLAqfC5WU4WjLUzpxfBLhQKLFy4kCsuu4SLJ4/g4sn9p8qv7jZ99xeNhvNGypgWwReEQzC3wSyB+Mz7MG94/4uGx+Pha1/72ol/8FRzU1Tp76oK2ixQ41V5bYvGq9uLTKqx9jhDiOcMpteq/O7KYJ/Pm0gkuPHGG9E0jU9+8pP9ek/tWXhovWnVf/1U8x6FgYV0iTDg+NqpUOGC375rpuw9lrS1tR11Ye4jwe9QcVgUitqBi6ufPdUJFoX/fjPLusZS178/vynP7Usz6Aa4bMoBETa9sX79er74xS9SVVXFAw88QH19fb/e0y/fhnWt8JVpMDYk41gEXxD6wMQK+IeJsLoZ/t/aY3uuQqHAPffcw3333YemacftHotlaI2XaUsfGO/+DzOc/MfHfGzcWeTCX0W5+ncJvvRInE/8KsZtL6XIlwycVmht09jZUe6z6Pv9fj7/+c9z66239vv9vLDDXHs5byR8dYaM4YGK5ZZbbrkRkKUVYUAxpQq2xmDZLjPfzgj/sTlPIBBg0qRJPP7447z44ouccsophELH3jxNFQw2tGjMHmE7qA99wTg7dZUWckVYuadIW1rn0mlO/vvTAUZVWMgUwbDCvLF2zhhh77GqVblcRlVNuy4UCnHKKaf0+73sTMBNr4HHDrfOk4yYA5iiYhhGEvBJWwgDjQ3t8PUXodoD//txqDyGQpLNZvn5z3/O5ZdfzpQpUwZUO3RkdBxWBa/j8Baaf/azn6EoCt/+9reP2bWli/DPL8HGCPz0XDhfFmoHMikRfGFA89gmuHulmXLhx/OP77l1Xe+yjgcTS5cu5eGHH6ayspIrr7yS008//Zid6z9eNxfYr5kJ150u43WgC75E6QgDmi9NNXds/nGrmVnzmpnH79zLly/nueee47LLLmP27Nk4nc5B0WbRaJQzzzyTf/7nfz6m5/nf9fD0VtNv/83TZKwOBsTCFwY8kRx862X4WxxuPQcuHH18ztvS0sJjjz3GypUrcTqd/PCHP2T8+PEDq20iEdra2o67G+qPW+GnK8xonP86H4Z5ZJwOBgtfBF8YFGyKmL5iw4B7L4RTqo/fuZubm1m8eDEXXnghDQ0nvoqHYRi0tbWxePFinnnmGWbMmHFMIm96YslO+I/lEHTC/RfB6KCMTxF8Qehn3tgL33/NjNG/Y4EZyXOi2L17Nw8++CBjx45l0qRJTJo06bhE9wA0NjZyyy23EAqFmD9/PrNnzyYcDh+Xc7+0E+5aacb/33kunDZMxqUIviAcI36/Be5cCWOD8JP5J26DTzKZ5KmnnmLDhg3s2rWLc889l29961tdxzVNI5VKEQgEjmjht729nebmZtatW8c777zD17/+dSZOnAhAOp1m+/btzJw587je8/K98MPlZgrlm86CC0bLeBTBF4RjzMIN8D/vQJ0X7joPxp3gXZ27d+9G13VGj/5AAX//+9/zhz/8gdGjR5PP5/nHf/zHbgK9c+dONm7ciNfrZcuWLcydO5cZMz7YsbRw4UKef/55xowZw9ixY7niiiuorq4+Yff48k74yQrTpfYfc+FcCb8clIIvUTrCoOPqGWY65btWwneXwW3zYeoJdO+MHHmg+s2aNYv6+noSiQRbt27F4ei+uer999/nd7/7HXV1dZTLZRYsWNDt+GWXXcYFF1zAiBEnvjbg7zabO56dVvjJArNugTA4EQtfGLT8diPc/zbUeuG7Z8KZg0iIDMNA1820ChaLZcBe50PrzMyl1W74wdlwZr2Mu8Fs4YvgC4OaRVvg56tNi//7Z5k59YWjJ6fBXX814+wnVsDNH4HpYWkXEXxBOMG8vMuMCc+WTHfP8dycdTKyqR1+sRbe2mf66n9wNtRInL0IviAMFDa2w0//aqbnvXQc/NOp5s5c4fD441ZzUbwxZe5y/r+zTN+9IIIvCAOK1gzcvQqW7oRxFfAvp5nFVITeiebhN+/CYxuhym2WJ/zCFGkXEXxBGOD8egM8uA5UBT41Ea6dCX67tEtPLN9jtte6NjijFm44EyZXSruI4AvCIOHNfabF+sY+OLsBPj0RLpYF3W7sTsIf3jOzXQJ8fgp8cSqEnNI2IviCMMiI5uHxTfDMVkgVzCpaF4+BmTVDu12SRXMj1e+3mGsf54403TcfaZAxI4IvCIOcd1rNVL5vNkKFA66YBFdMHJqRJ3/ZA49uMstH1vvgsnHwf04BixQ7FcEXhJOFkg5L/mZate+0monXzhkOHx8LY4In//0v+ZuZ+GxFIxjApyaYLz0pNi6CLwgnLW0ZeG47vLYH1rfCyCBcNh5m1cCskyzzYywPq5rMe31lF9gscMEoWDASzh0hY0EEXxCGCJEc/Gmbubi7vhW8dpg73BTESZWDu6DHjhisaTEt+rUtEHabaScuGgNzxU8vgi+CLwxVchq8uhsW/80USU03XTyn1phJwubUgWsQbDyK5s2X1xt7zeLvzWmo9ZjVwS4ac2KTywki+IIwoCiW4e1WWN9mphTYHDEXMk+pNsVyhB8mVprWvzIArtcANrfD+1HYHoMNbbClA9y2D15UM6rNPDiCIIIvCD3QkYO3Gk0R3bZfUDMlqPPBpApzwXfifrdPyGn+qMf4LZAsmFZ8W9YU+g3t5kupJWOef0KFeW1z6syMlqoi/SiI4AvC4QltEd7vMCN7NkdgVwLas1DWIeAw0xDUes0Qz5F+8+9Vbqh0mq4gp9VcLLX1EvZY1qFQhnwZ0kVTyDuysCcJTWnYmzT/LZYHFQh7TNfT1DDMrjUF32GR/hJE8AWhX9B0U3RbM6YQb4nA3hTsS0E8b2brBPA7wGPb/2MHl8UUf0Uxhd+imi4ZDCjqZn3YgmbOIlJF83eyYP52WMBnN2PmRwVNK35chfmSqfWIJS+I4AvCcSOnQUsaGtOmuyW3X6yzJXN2kMhDWoNM0SwPWNKhbJifVQC7xRRtt80U9gon+Bxm7h+fHYJOaPCZ5RyDkvJAEMEXhIFLWTfdNAXNtOqLZXOmoCim4DuspovGsd/9YxGLXTiGgi+ZrgXhGGJRwaOa7h1BONFIBg1BEAQRfEEQBEEEXxAEQRDBFwRBEETwBUEQBBF8QRAEQQRfEARB6BcUwzAMaQZBEISTHyvQhOy0FQRBONlJ/f8BABudSjmpQgUZAAAAAElFTkSuQmCC

    :param seg_list: list[posb] - List of posb segments.
    :param vel: float or float[2] - linear velocity or [linear velocity, angular velocity].
    :param acc: float or float[2] - linear acceleration or [linear acceleration, angular acceleration].
    :param time: float - Reach time [sec].
    :param ref: reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference). Default: DR_BASE.
    :param mod: int - Movement basis (DR_MV_MOD_ABS: Absolute, DR_MV_MOD_REL: Relative). Default: DR_MV_MOD_ABS.
    :param v: float or float[2] - linear velocity or [linear velocity, angular velocity].
    :param a: float or float[2] - linear acceleration or [linear acceleration, angular acceleration].
    :param t: float - Reach time [sec].

    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def move_spiral(rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None) -> int:
    """
    Motion along a spiral trajectory on a plane which is perpendicular to the input 'axis' is performed on the specified coordinate system 'ref'.
    Additional input, travel distance 'lmax' can cause the robot to move around a cone, starting from the apex of it.

    .. image:: data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAXwAAAELCAYAAADawD2zAAAACXBIWXMAAAsTAAALEwEAmpwYAAA4IGlUWHRYTUw6Y29tLmFkb2JlLnhtcAAAAAAAPD94cGFja2V0IGJlZ2luPSLvu78iIGlkPSJXNU0wTXBDZWhpSHpyZVN6TlRjemtjOWQiPz4KPHg6eG1wbWV0YSB4bWxuczp4PSJhZG9iZTpuczptZXRhLyIgeDp4bXB0az0iQWRvYmUgWE1QIENvcmUgNS41LWMwMjEgNzkuMTU0OTExLCAyMDEzLzEwLzI5LTExOjQ3OjE2ICAgICAgICAiPgogICA8cmRmOlJERiB4bWxuczpyZGY9Imh0dHA6Ly93d3cudzMub3JnLzE5OTkvMDIvMjItcmRmLXN5bnRheC1ucyMiPgogICAgICA8cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0iIgogICAgICAgICAgICB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmRjPSJodHRwOi8vcHVybC5vcmcvZGMvZWxlbWVudHMvMS4xLyIKICAgICAgICAgICAgeG1sbnM6cGhvdG9zaG9wPSJodHRwOi8vbnMuYWRvYmUuY29tL3Bob3Rvc2hvcC8xLjAvIgogICAgICAgICAgICB4bWxuczp4bXBNTT0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wL21tLyIKICAgICAgICAgICAgeG1sbnM6c3RFdnQ9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9zVHlwZS9SZXNvdXJjZUV2ZW50IyIKICAgICAgICAgICAgeG1sbnM6dGlmZj0iaHR0cDovL25zLmFkb2JlLmNvbS90aWZmLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmV4aWY9Imh0dHA6Ly9ucy5hZG9iZS5jb20vZXhpZi8xLjAvIj4KICAgICAgICAgPHhtcDpDcmVhdG9yVG9vbD5BZG9iZSBQaG90b3Nob3AgQ0MgKE1hY2ludG9zaCk8L3htcDpDcmVhdG9yVG9vbD4KICAgICAgICAgPHhtcDpDcmVhdGVEYXRlPjIwMTctMTEtMjBUMTY6NDU6NTArMDk6MDA8L3htcDpDcmVhdGVEYXRlPgogICAgICAgICA8eG1wOk1vZGlmeURhdGU+MjAxNy0xMi0xM1QxNzoxNjowNiswOTowMDwveG1wOk1vZGlmeURhdGU+CiAgICAgICAgIDx4bXA6TWV0YWRhdGFEYXRlPjIwMTctMTItMTNUMTc6MTY6MDYrMDk6MDA8L3htcDpNZXRhZGF0YURhdGU+CiAgICAgICAgIDxkYzpmb3JtYXQ+aW1hZ2UvcG5nPC9kYzpmb3JtYXQ+CiAgICAgICAgIDxwaG90b3Nob3A6Q29sb3JNb2RlPjM8L3Bob3Rvc2hvcDpDb2xvck1vZGU+CiAgICAgICAgIDx4bXBNTTpJbnN0YW5jZUlEPnhtcC5paWQ6ZmY3MzhjNzEtZjQyOS00YTcyLTlhYTgtNDg5MjE1OWYxY2ZhPC94bXBNTTpJbnN0YW5jZUlEPgogICAgICAgICA8eG1wTU06RG9jdW1lbnRJRD54bXAuZGlkOmZmNzM4YzcxLWY0MjktNGE3Mi05YWE4LTQ4OTIxNTlmMWNmYTwveG1wTU06RG9jdW1lbnRJRD4KICAgICAgICAgPHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD54bXAuZGlkOmZmNzM4YzcxLWY0MjktNGE3Mi05YWE4LTQ4OTIxNTlmMWNmYTwveG1wTU06T3JpZ2luYWxEb2N1bWVudElEPgogICAgICAgICA8eG1wTU06SGlzdG9yeT4KICAgICAgICAgICAgPHJkZjpTZXE+CiAgICAgICAgICAgICAgIDxyZGY6bGkgcmRmOnBhcnNlVHlwZT0iUmVzb3VyY2UiPgogICAgICAgICAgICAgICAgICA8c3RFdnQ6YWN0aW9uPmNyZWF0ZWQ8L3N0RXZ0OmFjdGlvbj4KICAgICAgICAgICAgICAgICAgPHN0RXZ0Omluc3RhbmNlSUQ+eG1wLmlpZDpmZjczOGM3MS1mNDI5LTRhNzItOWFhOC00ODkyMTU5ZjFjZmE8L3N0RXZ0Omluc3RhbmNlSUQ+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDp3aGVuPjIwMTctMTEtMjBUMTY6NDU6NTArMDk6MDA8L3N0RXZ0OndoZW4+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDpzb2Z0d2FyZUFnZW50PkFkb2JlIFBob3Rvc2hvcCBDQyAoTWFjaW50b3NoKTwvc3RFdnQ6c29mdHdhcmVBZ2VudD4KICAgICAgICAgICAgICAgPC9yZGY6bGk+CiAgICAgICAgICAgIDwvcmRmOlNlcT4KICAgICAgICAgPC94bXBNTTpIaXN0b3J5PgogICAgICAgICA8dGlmZjpPcmllbnRhdGlvbj4xPC90aWZmOk9yaWVudGF0aW9uPgogICAgICAgICA8dGlmZjpYUmVzb2x1dGlvbj43MjAwMDAvMTAwMDA8L3RpZmY6WFJlc29sdXRpb24+CiAgICAgICAgIDx0aWZmOllSZXNvbHV0aW9uPjcyMDAwMC8xMDAwMDwvdGlmZjpZUmVzb2x1dGlvbj4KICAgICAgICAgPHRpZmY6UmVzb2x1dGlvblVuaXQ+MjwvdGlmZjpSZXNvbHV0aW9uVW5pdD4KICAgICAgICAgPGV4aWY6Q29sb3JTcGFjZT42NTUzNTwvZXhpZjpDb2xvclNwYWNlPgogICAgICAgICA8ZXhpZjpQaXhlbFhEaW1lbnNpb24+MzgwPC9leGlmOlBpeGVsWERpbWVuc2lvbj4KICAgICAgICAgPGV4aWY6UGl4ZWxZRGltZW5zaW9uPjI2NzwvZXhpZjpQaXhlbFlEaW1lbnNpb24+CiAgICAgIDwvcmRmOkRlc2NyaXB0aW9uPgogICA8L3JkZjpSREY+CjwveDp4bXBtZXRhPgogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAg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

    :param rev: float - Total number of revolutions. Default: 10.
    :param rmax: float - Final spiral radius [mm]. Default: 10.
    :param lmax: float - Distance moved in the axis direction [mm]. Default: 0.
    :param vel: float - velocity.
    :param acc: float - acceleration.
    :param time: float - Total execution time [sec].
    :param axis: int - axis - (DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z). Default: DR_AXIS_Z.
    :param ref: reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference). Default: DR_BASE.
    :param v: float - velocity.
    :param a: float - acceleration.
    :param t: float - Total execution time [sec].

    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def amove_spiral(rev=10, rmax=10, lmax=0, vel=None, acc=None, time=None, axis=DR_AXIS_Z, ref=DR_TOOL, v=None, a=None, t=None):
    """
    The  asynchronous  move_spiral  motion  operates  in  the  same  way  as  move_spiral()  except  for  the asynchronous
    processing and executes the next line after the command is executed. Generating a new command for the motion before
    the amove_spiral() motion results in an error for safety reasons. Therefore, the termination of the amove_spiral()
    motion must be confirmed using mwait()  or check_motion() between amove_spiral() and the following motion command.
    Motion along a spiral trajectory on a plane which is perpendicular to the input 'axis' is performed on the specified coordinate system 'ref'.
    Additional input, travel distance 'lmax' can cause the robot to move around a cone, starting from the apex of it.

    .. image:: data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAXwAAAELCAYAAADawD2zAAAACXBIWXMAAAsTAAALEwEAmpwYAAA4IGlUWHRYTUw6Y29tLmFkb2JlLnhtcAAAAAAAPD94cGFja2V0IGJlZ2luPSLvu78iIGlkPSJXNU0wTXBDZWhpSHpyZVN6TlRjemtjOWQiPz4KPHg6eG1wbWV0YSB4bWxuczp4PSJhZG9iZTpuczptZXRhLyIgeDp4bXB0az0iQWRvYmUgWE1QIENvcmUgNS41LWMwMjEgNzkuMTU0OTExLCAyMDEzLzEwLzI5LTExOjQ3OjE2ICAgICAgICAiPgogICA8cmRmOlJERiB4bWxuczpyZGY9Imh0dHA6Ly93d3cudzMub3JnLzE5OTkvMDIvMjItcmRmLXN5bnRheC1ucyMiPgogICAgICA8cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0iIgogICAgICAgICAgICB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmRjPSJodHRwOi8vcHVybC5vcmcvZGMvZWxlbWVudHMvMS4xLyIKICAgICAgICAgICAgeG1sbnM6cGhvdG9zaG9wPSJodHRwOi8vbnMuYWRvYmUuY29tL3Bob3Rvc2hvcC8xLjAvIgogICAgICAgICAgICB4bWxuczp4bXBNTT0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wL21tLyIKICAgICAgICAgICAgeG1sbnM6c3RFdnQ9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9zVHlwZS9SZXNvdXJjZUV2ZW50IyIKICAgICAgICAgICAgeG1sbnM6dGlmZj0iaHR0cDovL25zLmFkb2JlLmNvbS90aWZmLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmV4aWY9Imh0dHA6Ly9ucy5hZG9iZS5jb20vZXhpZi8xLjAvIj4KICAgICAgICAgPHhtcDpDcmVhdG9yVG9vbD5BZG9iZSBQaG90b3Nob3AgQ0MgKE1hY2ludG9zaCk8L3htcDpDcmVhdG9yVG9vbD4KICAgICAgICAgPHhtcDpDcmVhdGVEYXRlPjIwMTctMTEtMjBUMTY6NDU6NTArMDk6MDA8L3htcDpDcmVhdGVEYXRlPgogICAgICAgICA8eG1wOk1vZGlmeURhdGU+MjAxNy0xMi0xM1QxNzoxNjowNiswOTowMDwveG1wOk1vZGlmeURhdGU+CiAgICAgICAgIDx4bXA6TWV0YWRhdGFEYXRlPjIwMTctMTItMTNUMTc6MTY6MDYrMDk6MDA8L3htcDpNZXRhZGF0YURhdGU+CiAgICAgICAgIDxkYzpmb3JtYXQ+aW1hZ2UvcG5nPC9kYzpmb3JtYXQ+CiAgICAgICAgIDxwaG90b3Nob3A6Q29sb3JNb2RlPjM8L3Bob3Rvc2hvcDpDb2xvck1vZGU+CiAgICAgICAgIDx4bXBNTTpJbnN0YW5jZUlEPnhtcC5paWQ6ZmY3MzhjNzEtZjQyOS00YTcyLTlhYTgtNDg5MjE1OWYxY2ZhPC94bXBNTTpJbnN0YW5jZUlEPgogICAgICAgICA8eG1wTU06RG9jdW1lbnRJRD54bXAuZGlkOmZmNzM4YzcxLWY0MjktNGE3Mi05YWE4LTQ4OTIxNTlmMWNmYTwveG1wTU06RG9jdW1lbnRJRD4KICAgICAgICAgPHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD54bXAuZGlkOmZmNzM4YzcxLWY0MjktNGE3Mi05YWE4LTQ4OTIxNTlmMWNmYTwveG1wTU06T3JpZ2luYWxEb2N1bWVudElEPgogICAgICAgICA8eG1wTU06SGlzdG9yeT4KICAgICAgICAgICAgPHJkZjpTZXE+CiAgICAgICAgICAgICAgIDxyZGY6bGkgcmRmOnBhcnNlVHlwZT0iUmVzb3VyY2UiPgogICAgICAgICAgICAgICAgICA8c3RFdnQ6YWN0aW9uPmNyZWF0ZWQ8L3N0RXZ0OmFjdGlvbj4KICAgICAgICAgICAgICAgICAgPHN0RXZ0Omluc3RhbmNlSUQ+eG1wLmlpZDpmZjczOGM3MS1mNDI5LTRhNzItOWFhOC00ODkyMTU5ZjFjZmE8L3N0RXZ0Omluc3RhbmNlSUQ+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDp3aGVuPjIwMTctMTEtMjBUMTY6NDU6NTArMDk6MDA8L3N0RXZ0OndoZW4+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDpzb2Z0d2FyZUFnZW50PkFkb2JlIFBob3Rvc2hvcCBDQyAoTWFjaW50b3NoKTwvc3RFdnQ6c29mdHdhcmVBZ2VudD4KICAgICAgICAgICAgICAgPC9yZGY6bGk+CiAgICAgICAgICAgIDwvcmRmOlNlcT4KICAgICAgICAgPC94bXBNTTpIaXN0b3J5PgogICAgICAgICA8dGlmZjpPcmllbnRhdGlvbj4xPC90aWZmOk9yaWVudGF0aW9uPgogICAgICAgICA8dGlmZjpYUmVzb2x1dGlvbj43MjAwMDAvMTAwMDA8L3RpZmY6WFJlc29sdXRpb24+CiAgICAgICAgIDx0aWZmOllSZXNvbHV0aW9uPjcyMDAwMC8xMDAwMDwvdGlmZjpZUmVzb2x1dGlvbj4KICAgICAgICAgPHRpZmY6UmVzb2x1dGlvblVuaXQ+MjwvdGlmZjpSZXNvbHV0aW9uVW5pdD4KICAgICAgICAgPGV4aWY6Q29sb3JTcGFjZT42NTUzNTwvZXhpZjpDb2xvclNwYWNlPgogICAgICAgICA8ZXhpZjpQaXhlbFhEaW1lbnNpb24+MzgwPC9leGlmOlBpeGVsWERpbWVuc2lvbj4KICAgICAgICAgPGV4aWY6UGl4ZWxZRGltZW5zaW9uPjI2NzwvZXhpZjpQaXhlbFlEaW1lbnNpb24+CiAgICAgIDwvcmRmOkRlc2NyaXB0aW9uPgogICA8L3JkZjpSREY+CjwveDp4bXBtZXRhPgogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAg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

    :param rev: float - Total number of revolutions. Default: 10.
    :param rmax: float - Final spiral radius [mm]. Default: 10.
    :param lmax: float - Distance moved in the axis direction [mm]. Default: 0.
    :param vel: float - velocity.
    :param acc: float - acceleration.
    :param time: float - Total execution time [sec].
    :param axis: int - axis - (DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z). Default: DR_AXIS_Z.
    :param ref: reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference). Default: DR_BASE.
    :param v: float - velocity.
    :param a: float - acceleration.
    :param t: float - Total execution time [sec].

    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def move_periodic(amp, period, atime=None, repeat=None, ref=DR_TOOL):
    """
    This function performs the cyclic motion based on the sine function of each axis (parallel and rotation) of  the
    reference  coordinate  (ref)  input  as  a  relative  motion  that  begins  at  the  current  position.
    The attributes  of  the  motion  on  each  axis  are  determined  by  the  amplitude  and  period,
    and  the acceleration/deceleration time and the total motion time are set by the interval and repetition count.

    .. image:: data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAXwAAADOCAYAAAA9krkAAAAACXBIWXMAAAsTAAALEwEAmpwYAAA4IGlUWHRYTUw6Y29tLmFkb2JlLnhtcAAAAAAAPD94cGFja2V0IGJlZ2luPSLvu78iIGlkPSJXNU0wTXBDZWhpSHpyZVN6TlRjemtjOWQiPz4KPHg6eG1wbWV0YSB4bWxuczp4PSJhZG9iZTpuczptZXRhLyIgeDp4bXB0az0iQWRvYmUgWE1QIENvcmUgNS41LWMwMjEgNzkuMTU0OTExLCAyMDEzLzEwLzI5LTExOjQ3OjE2ICAgICAgICAiPgogICA8cmRmOlJERiB4bWxuczpyZGY9Imh0dHA6Ly93d3cudzMub3JnLzE5OTkvMDIvMjItcmRmLXN5bnRheC1ucyMiPgogICAgICA8cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0iIgogICAgICAgICAgICB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmRjPSJodHRwOi8vcHVybC5vcmcvZGMvZWxlbWVudHMvMS4xLyIKICAgICAgICAgICAgeG1sbnM6cGhvdG9zaG9wPSJodHRwOi8vbnMuYWRvYmUuY29tL3Bob3Rvc2hvcC8xLjAvIgogICAgICAgICAgICB4bWxuczp4bXBNTT0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wL21tLyIKICAgICAgICAgICAgeG1sbnM6c3RFdnQ9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9zVHlwZS9SZXNvdXJjZUV2ZW50IyIKICAgICAgICAgICAgeG1sbnM6dGlmZj0iaHR0cDovL25zLmFkb2JlLmNvbS90aWZmLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmV4aWY9Imh0dHA6Ly9ucy5hZG9iZS5jb20vZXhpZi8xLjAvIj4KICAgICAgICAgPHhtcDpDcmVhdG9yVG9vbD5BZG9iZSBQaG90b3Nob3AgQ0MgKE1hY2ludG9zaCk8L3htcDpDcmVhdG9yVG9vbD4KICAgICAgICAgPHhtcDpDcmVhdGVEYXRlPjIwMTctMTEtMjBUMTY6NDU6NTArMDk6MDA8L3htcDpDcmVhdGVEYXRlPgogICAgICAgICA8eG1wOk1vZGlmeURhdGU+MjAxNy0xMi0xM1QxNzo1MToyMiswOTowMDwveG1wOk1vZGlmeURhdGU+CiAgICAgICAgIDx4bXA6TWV0YWRhdGFEYXRlPjIwMTctMTItMTNUMTc6NTE6MjIrMDk6MDA8L3htcDpNZXRhZGF0YURhdGU+CiAgICAgICAgIDxkYzpmb3JtYXQ+aW1hZ2UvcG5nPC9kYzpmb3JtYXQ+CiAgICAgICAgIDxwaG90b3Nob3A6Q29sb3JNb2RlPjM8L3Bob3Rvc2hvcDpDb2xvck1vZGU+CiAgICAgICAgIDx4bXBNTTpJbnN0YW5jZUlEPnhtcC5paWQ6MTVmMjJlMmQtODhiMi00ZDc1LWFiY2UtMTFhZWIzNWI3YWJhPC94bXBNTTpJbnN0YW5jZUlEPgogICAgICAgICA8eG1wTU06RG9jdW1lbnRJRD54bXAuZGlkOjE1ZjIyZTJkLTg4YjItNGQ3NS1hYmNlLTExYWViMzViN2FiYTwveG1wTU06RG9jdW1lbnRJRD4KICAgICAgICAgPHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD54bXAuZGlkOjE1ZjIyZTJkLTg4YjItNGQ3NS1hYmNlLTExYWViMzViN2FiYTwveG1wTU06T3JpZ2luYWxEb2N1bWVudElEPgogICAgICAgICA8eG1wTU06SGlzdG9yeT4KICAgICAgICAgICAgPHJkZjpTZXE+CiAgICAgICAgICAgICAgIDxyZGY6bGkgcmRmOnBhcnNlVHlwZT0iUmVzb3VyY2UiPgogICAgICAgICAgICAgICAgICA8c3RFdnQ6YWN0aW9uPmNyZWF0ZWQ8L3N0RXZ0OmFjdGlvbj4KICAgICAgICAgICAgICAgICAgPHN0RXZ0Omluc3RhbmNlSUQ+eG1wLmlpZDoxNWYyMmUyZC04OGIyLTRkNzUtYWJjZS0xMWFlYjM1YjdhYmE8L3N0RXZ0Omluc3RhbmNlSUQ+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDp3aGVuPjIwMTctMTEtMjBUMTY6NDU6NTArMDk6MDA8L3N0RXZ0OndoZW4+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDpzb2Z0d2FyZUFnZW50PkFkb2JlIFBob3Rvc2hvcCBDQyAoTWFjaW50b3NoKTwvc3RFdnQ6c29mdHdhcmVBZ2VudD4KICAgICAgICAgICAgICAgPC9yZGY6bGk+CiAgICAgICAgICAgIDwvcmRmOlNlcT4KICAgICAgICAgPC94bXBNTTpIaXN0b3J5PgogICAgICAgICA8dGlmZjpPcmllbnRhdGlvbj4xPC90aWZmOk9yaWVudGF0aW9uPgogICAgICAgICA8dGlmZjpYUmVzb2x1dGlvbj43MjAwMDAvMTAwMDA8L3RpZmY6WFJlc29sdXRpb24+CiAgICAgICAgIDx0aWZmOllSZXNvbHV0aW9uPjcyMDAwMC8xMDAwMDwvdGlmZjpZUmVzb2x1dGlvbj4KICAgICAgICAgPHRpZmY6UmVzb2x1dGlvblVuaXQ+MjwvdGlmZjpSZXNvbHV0aW9uVW5pdD4KICAgICAgICAgPGV4aWY6Q29sb3JTcGFjZT42NTUzNTwvZXhpZjpDb2xvclNwYWNlPgogICAgICAgICA8ZXhpZjpQaXhlbFhEaW1lbnNpb24+MzgwPC9leGlmOlBpeGVsWERpbWVuc2lvbj4KICAgICAgICAgPGV4aWY6UGl4ZWxZRGltZW5zaW9uPjIwNjwvZXhpZjpQaXhlbFlEaW1lbnNpb24+CiAgICAgIDwvcmRmOkRlc2NyaXB0aW9uPgogICA8L3JkZjpSREY+CjwveDp4bXBtZXRhPgogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAg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WcMjjzySkGEvgRB8QSvhnnvuYfLkyUjS5a96ePToUX77299y/PjxhLaRzWbjRz/6EYsWLboiIZU9e/YQjUYZMGCAuGGF4AsS1StsDciyTHJy8hU594EDB0hPTyc/Pz/h7XTddddRVVXFihUrLvu5e/fuzZQpU9DrRWVKIfiChGPhwoX84Q9/EIZohHA4zLx58ygqKkKr1SZ8ey0WC4899hi9e/e+7Oe2Wq0JO/EvaByRltkO+PjjjykqKmpVbd67dy/p6emkp6dflvMpisLtt9/OwIEDW42Nxo4dK25ugfDwBf/lvffew+Vy8f3vf79VtfuDDz7g/fffv2zn02q1TJgwIaEnta80sViML774gsrKSmEMIfiCRMThcHDTTTddsbj4xXLbbbexe/dunE6n6MTzEAwGL8sczc6dO5k+fToej0cYXQi+IBF57LHH+PGPf9zq2j106FAUReHNN99s8XOFQqFW278lJSU8+uijHDx4sMXPtXz5ckaNGkVBQYF4sITgCxKVK5Hi2Bw88sgjF1yc60Lx+/1MnjyZNWvWtEobpaWlIcsyn332WYuep6qqim3btjFmzBjxQAnBFwian2uuuYbhw4e36DlOnDjBkSNHsNlsrdJGVquVe++9l6VLl1JWVtZi54lGo0yYMCFhqnQKhOALzqC0tJQPP/xQ7CLVCIqiMHfuXHJzc69IemNzMXDgQPr169eigp+VlcW3v/1tZFkWN44QfEGiMXv2bObPn98mrqWlJiQVRSESiVzxjdwvFYvFwosvvpiQ5ZwFiYXIw2+DHD58mHXr1vGLX/yiTVR8/OMf/8jw4cObPZ6vUqmYPHlym+n31jpXIxAevuASWL58OUlJSS0e/75cKIpCcXGx6NgrQFVVFU899RR79+4VxhCCL0hExo0bx7PPPttmrueee+5h9+7d7N69W3RuIxw5coR//OMfzVo2eeXKlezfvz+htnoUCMEXnEGHDh1aRQGwppKXl0ePHj3YunVrsx1z0aJFbN68uU31u8/n44MPPmDjxo3Ncjy/38+8efO477776Nixo3iwhOALBJeHp556im9/+9vNcqxQKMSbb77Jvn372pSNevXqxejRo1m8eHGzHK+yspKcnByuvfZacQMKwRckGuFwuM3WOdHpdBgMhmY51s6dO+ObrbQ1hg0bxo4dOzh69OglHys3N5eXXnqJ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

    :param amp: float[6] - Amplitude (motion between -amp and +amp) [mm] or [deg].
    :param period: float or float[6] - Period (time for 1 cycle) [sec].
    :param atime: float - Acceleration time.
    :param repeat: int - Repetition time.
    :param ref: reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference). Default: DR_BASE.
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def amove_periodic(amp, period, atime=None, repeat=None, ref=DR_TOOL) -> int:
    """
    The asynchronous move_periodic motion operates in the same way as move_periodic() except for the asynchronous
    processing and executes the next line after the command is executed. Generating a new command for the motion before
    the amove_periodic() motion results in an error for safety  reasons.  Therefore,  the  termination  of  the  amove_periodic()
    motion  must  be  confirmed  using mwait() or check_motion() between amove_periodic() and the following motion command.
    This command performs a cyclic motion based on thesine function of each axis (parallel and rotation) of  the  reference
    coordinate  (ref)  input  as  a  relative  motion  that  begins  at  the  current  position.  The attributes  of  the
    motion  on  each  axis  are  determined  by  amp  (amplitude)  and  period,  and  the acceleration/deceleration time and
    the total motion time are set by the interval and repetition count.

    .. image:: data:image/png;base64, iVBORw0KGgoAAAANSUhEUgAAAXwAAADOCAYAAAA9krkAAAAACXBIWXMAAAsTAAALEwEAmpwYAAA4IGlUWHRYTUw6Y29tLmFkb2JlLnhtcAAAAAAAPD94cGFja2V0IGJlZ2luPSLvu78iIGlkPSJXNU0wTXBDZWhpSHpyZVN6TlRjemtjOWQiPz4KPHg6eG1wbWV0YSB4bWxuczp4PSJhZG9iZTpuczptZXRhLyIgeDp4bXB0az0iQWRvYmUgWE1QIENvcmUgNS41LWMwMjEgNzkuMTU0OTExLCAyMDEzLzEwLzI5LTExOjQ3OjE2ICAgICAgICAiPgogICA8cmRmOlJERiB4bWxuczpyZGY9Imh0dHA6Ly93d3cudzMub3JnLzE5OTkvMDIvMjItcmRmLXN5bnRheC1ucyMiPgogICAgICA8cmRmOkRlc2NyaXB0aW9uIHJkZjphYm91dD0iIgogICAgICAgICAgICB4bWxuczp4bXA9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmRjPSJodHRwOi8vcHVybC5vcmcvZGMvZWxlbWVudHMvMS4xLyIKICAgICAgICAgICAgeG1sbnM6cGhvdG9zaG9wPSJodHRwOi8vbnMuYWRvYmUuY29tL3Bob3Rvc2hvcC8xLjAvIgogICAgICAgICAgICB4bWxuczp4bXBNTT0iaHR0cDovL25zLmFkb2JlLmNvbS94YXAvMS4wL21tLyIKICAgICAgICAgICAgeG1sbnM6c3RFdnQ9Imh0dHA6Ly9ucy5hZG9iZS5jb20veGFwLzEuMC9zVHlwZS9SZXNvdXJjZUV2ZW50IyIKICAgICAgICAgICAgeG1sbnM6dGlmZj0iaHR0cDovL25zLmFkb2JlLmNvbS90aWZmLzEuMC8iCiAgICAgICAgICAgIHhtbG5zOmV4aWY9Imh0dHA6Ly9ucy5hZG9iZS5jb20vZXhpZi8xLjAvIj4KICAgICAgICAgPHhtcDpDcmVhdG9yVG9vbD5BZG9iZSBQaG90b3Nob3AgQ0MgKE1hY2ludG9zaCk8L3htcDpDcmVhdG9yVG9vbD4KICAgICAgICAgPHhtcDpDcmVhdGVEYXRlPjIwMTctMTEtMjBUMTY6NDU6NTArMDk6MDA8L3htcDpDcmVhdGVEYXRlPgogICAgICAgICA8eG1wOk1vZGlmeURhdGU+MjAxNy0xMi0xM1QxNzo1MToyMiswOTowMDwveG1wOk1vZGlmeURhdGU+CiAgICAgICAgIDx4bXA6TWV0YWRhdGFEYXRlPjIwMTctMTItMTNUMTc6NTE6MjIrMDk6MDA8L3htcDpNZXRhZGF0YURhdGU+CiAgICAgICAgIDxkYzpmb3JtYXQ+aW1hZ2UvcG5nPC9kYzpmb3JtYXQ+CiAgICAgICAgIDxwaG90b3Nob3A6Q29sb3JNb2RlPjM8L3Bob3Rvc2hvcDpDb2xvck1vZGU+CiAgICAgICAgIDx4bXBNTTpJbnN0YW5jZUlEPnhtcC5paWQ6MTVmMjJlMmQtODhiMi00ZDc1LWFiY2UtMTFhZWIzNWI3YWJhPC94bXBNTTpJbnN0YW5jZUlEPgogICAgICAgICA8eG1wTU06RG9jdW1lbnRJRD54bXAuZGlkOjE1ZjIyZTJkLTg4YjItNGQ3NS1hYmNlLTExYWViMzViN2FiYTwveG1wTU06RG9jdW1lbnRJRD4KICAgICAgICAgPHhtcE1NOk9yaWdpbmFsRG9jdW1lbnRJRD54bXAuZGlkOjE1ZjIyZTJkLTg4YjItNGQ3NS1hYmNlLTExYWViMzViN2FiYTwveG1wTU06T3JpZ2luYWxEb2N1bWVudElEPgogICAgICAgICA8eG1wTU06SGlzdG9yeT4KICAgICAgICAgICAgPHJkZjpTZXE+CiAgICAgICAgICAgICAgIDxyZGY6bGkgcmRmOnBhcnNlVHlwZT0iUmVzb3VyY2UiPgogICAgICAgICAgICAgICAgICA8c3RFdnQ6YWN0aW9uPmNyZWF0ZWQ8L3N0RXZ0OmFjdGlvbj4KICAgICAgICAgICAgICAgICAgPHN0RXZ0Omluc3RhbmNlSUQ+eG1wLmlpZDoxNWYyMmUyZC04OGIyLTRkNzUtYWJjZS0xMWFlYjM1YjdhYmE8L3N0RXZ0Omluc3RhbmNlSUQ+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDp3aGVuPjIwMTctMTEtMjBUMTY6NDU6NTArMDk6MDA8L3N0RXZ0OndoZW4+CiAgICAgICAgICAgICAgICAgIDxzdEV2dDpzb2Z0d2FyZUFnZW50PkFkb2JlIFBob3Rvc2hvcCBDQyAoTWFjaW50b3NoKTwvc3RFdnQ6c29mdHdhcmVBZ2VudD4KICAgICAgICAgICAgICAgPC9yZGY6bGk+CiAgICAgICAgICAgIDwvcmRmOlNlcT4KICAgICAgICAgPC94bXBNTTpIaXN0b3J5PgogICAgICAgICA8dGlmZjpPcmllbnRhdGlvbj4xPC90aWZmOk9yaWVudGF0aW9uPgogICAgICAgICA8dGlmZjpYUmVzb2x1dGlvbj43MjAwMDAvMTAwMDA8L3RpZmY6WFJlc29sdXRpb24+CiAgICAgICAgIDx0aWZmOllSZXNvbHV0aW9uPjcyMDAwMC8xMDAwMDwvdGlmZjpZUmVzb2x1dGlvbj4KICAgICAgICAgPHRpZmY6UmVzb2x1dGlvblVuaXQ+MjwvdGlmZjpSZXNvbHV0aW9uVW5pdD4KICAgICAgICAgPGV4aWY6Q29sb3JTcGFjZT42NTUzNTwvZXhpZjpDb2xvclNwYWNlPgogICAgICAgICA8ZXhpZjpQaXhlbFhEaW1lbnNpb24+MzgwPC9leGlmOlBpeGVsWERpbWVuc2lvbj4KICAgICAgICAgPGV4aWY6UGl4ZWxZRGltZW5zaW9uPjIwNjwvZXhpZjpQaXhlbFlEaW1lbnNpb24+CiAgICAgIDwvcmRmOkRlc2NyaXB0aW9uPgogICA8L3JkZjpSREY+CjwveDp4bXBtZXRhPgogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIAogICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAKICAgICAgICAgICAgICAgICAg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WcMjjzySkGEvgRB8QSvhnnvuYfLkyUjS5a96ePToUX77299y/PjxhLaRzWbjRz/6EYsWLboiIZU9e/YQjUYZMGCAuGGF4AsS1StsDciyTHJy8hU594EDB0hPTyc/Pz/h7XTddddRVVXFihUrLvu5e/fuzZQpU9DrRWVKIfiChGPhwoX84Q9/EIZohHA4zLx58ygqKkKr1SZ8ey0WC4899hi9e/e+7Oe2Wq0JO/EvaByRltkO+PjjjykqKmpVbd67dy/p6emkp6dflvMpisLtt9/OwIEDW42Nxo4dK25ugfDwBf/lvffew+Vy8f3vf79VtfuDDz7g/fffv2zn02q1TJgwIaEnta80sViML774gsrKSmEMIfiCRMThcHDTTTddsbj4xXLbbbexe/dunE6n6MTzEAwGL8sczc6dO5k+fToej0cYXQi+IBF57LHH+PGPf9zq2j106FAUReHNN99s8XOFQqFW278lJSU8+uijHDx4sMXPtXz5ckaNGkVBQYF4sITgCxKVK5Hi2Bw88sgjF1yc60Lx+/1MnjyZNWvWtEobpaWlIcsyn332WYuep6qqim3btjFmzBjxQAnBFwian2uuuYbhw4e36DlOnDjBkSNHsNlsrdJGVquVe++9l6VLl1JWVtZi54lGo0yYMCFhqnQKhOALzqC0tJQPP/xQ7CLVCIqiMHfuXHJzc69IemNzMXDgQPr169eigp+VlcW3v/1tZFkWN44QfEGiMXv2bObPn98mrqWlJiQVRSESiVzxjdwvFYvFwosvvpiQ5ZwFiYXIw2+DHD58mHXr1vGLX/yiTVR8/OMf/8jw4cObPZ6vUqmYPHlym+n31jpXIxAevuASWL58OUlJSS0e/75cKIpCcXGx6NgrQFVVFU899RR79+4VxhCCL0hExo0bx7PPPttmrueee+5h9+7d7N69W3RuIxw5coR//OMfzVo2eeXKlezfvz+htnoUCMEXnEGHDh1aRQGwppKXl0ePHj3YunVrsx1z0aJFbN68uU31u8/n44MPPmDjxo3Ncjy/38+8efO477776Nixo3iwhOALBJeHp556im9/+9vNcqxQKMSbb77Jvn372pSNevXqxejRo1m8eHGzHK+yspKcnByuvfZacQMKwRckGuFwuM3WOdHpdBgMhmY51s6dO+ObrbQ1hg0bxo4dOzh69OglHys3N5eXXnqJ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

    :param amp: float[6] - Amplitude (motion between -amp and +amp) [mm] or [deg].
    :param period: float or float[6] - Period (time for 1 cycle) [sec].
    :param atime: float - Acceleration time.
    :param repeat: int - Repetition time.
    :param ref: reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference). Default: DR_BASE.
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def mwait(time=None, state=None, progress=None) -> int:
    """
    This function sets the waiting time between the previous motion command and the motion command in the next line.
    The waiting time differs according to the time[sec] input.
    :param time: Waiting time after the motion ends [sec]

    :return: int - (0 -> Success, Negative value -> Error)
    """
    wait(time)
    return 0


def begin_blend(radius=0) -> int:
    """
    This  function  begins  the  blending  section.  The  following  sync  motion  commands  (movej, movel, movec, and movejx)
    with the blending section argument radius are blended using the radius  set  as  the  default  argument.
    There  is  no  actual  blending  effect  if  the  radius  is  0. Moreover, if a blending radius that  is different from
    the set radius is needed, the blending radius  can  be  changed  as  an  exception  by  specifying  the  blending
    radius  to  the  motion argument.

    :param radius: Radius for blending.

    :return: int - (0 -> Success, Negative value -> Error)
    """
    global _robodk_plugin_r
    _robodk_plugin_r = radius
    return 0


def end_blend() -> int:
    """
    This function ends the blending section.
    It means that the validity of the blending section that began with begin_blend() ends.

    :return: int - (0 -> Success, Negative value -> Error)
    """
    global _robodk_plugin_r
    _robodk_plugin_r = -1
    return 0


def check_motion() -> int:
    """
    This function checks the status of the currently active motion.

    :return: int - (DR_STATE_IDLE, DR_STATE_INIT, DR_STATE_BUSY)
    """
    if _robodk_plugin_RDK is not None:
        if _robodk_plugin_robot.Busy():
            return DR_STATE_BUSY
        else:
            return DR_STATE_IDLE
    return DR_STATE_IDLE


def get_motion_status():
    return None


def stop(st_mode) -> int:
    """
    This function stops the currently active motion. This function stops differently according to the st_mode received as an argument.
    All stop modes except Estop stop the motion in the currently active section.

    :param st_mode:  int - (DR_QSTOP_STOP, DR_QSTOP, DR_SSTO, DR_HOLD)
    :return: int - (0 -> Success, Negative value -> Error)
    """
    if _robodk_plugin_RDK_backup is not None:
        _robodk_plugin_robot_backup.Stop()
    return 0


def exit(mode=1):
    """
    This function terminates the currently running program-
    """
    sys.exit()


def change_operation_speed(speed):
    """
    This  function  adjusts  the  operation  velocity.  The  argument  is  the  relative  velocity  in  a percentage of
    the currently set velocity and has a value from 1 to 100. Therefore, a value of 50 means that the velocity is
    reduced to 50% of the currently set velocity.

    :param speed: int - operation speed(1~100).
    """
    if _robodk_plugin_RDK is not None:
        _robodk_plugin_RDK.setSimulationSpeed(speed/100)


def enable_virtual_wall():
    return None


def disable_virtual_wall():
    return None


def watch_collision_on(sensitivity=DR_COLSENS_DEFAULT):
    return None


def set_collision_sensitivity(sensitivity):
    return None


def watch_collision_off():
    return None


def get_workpiece_weight() -> float:
    """
    This function measures and returns the weight of the workpiece.

    :return: float - Measured weight.
    """
    return 0


def set_workpiece_weight(weight=0.0, cog=[0.0, 0.0, 0.0], cog_ref=DR_CUR_TCP, add_up=DR_REPLACE, start_time=None, transition_time=None):
    """
    In addition to the tool weight/center of gravity at the end of the robot, set the weight/center of gravity of the work piece and other information. The weight and center of gravity of the entire payload is reflected by combining the set tool weight/center of gravity and the work piece's weight/center of gravity. It can be used in applications where the type of workpiece is frequently varied or the weight needs to be dynamically changed.
    Workpiece weight change is allowed only when both Collision Detection and TCP SLF Violation check are mute or deactivated during Auto Mode.
    In the current version, Collision Detection considers the function mute when Collision Sensitivity is overridden to 0 and TCP SLF considers the function mute when the TCP SLF Limit is overridden to the maximum. This override can be set using Collision Sensitivity Reduction Zone and Custom Zone.
    Otherwise, trigger an SS1 protective stop unless the workpiece weight is set to zero.
    If the robot stops due to an error and needs to be manually restored, place the robot in the desired position in the Recovery Mode and unload the workpiece through Servo On and I/O operation while the corresponding zones are activated in Auto Mode.
    When changing the set tool weight, the workpiece weight is initialized to 0

    :param weight: float - Weight [kg]
    :param cog: [float, float, float] - Center of gravity of the workpiece (x, y, z) [mm]
    :param cog_ref: Reference coordinates of center of gravity position. DR_CUR_TCP : TCP coordinates, DR_FLANGE : FLANGE coordinates.
    :param add_up: DR_REPLACE(0): Replace workpiece, DR_ADD(1): Add workpiece, DR_REMOVE(2): Remove workpiece.
    :parm start_time: float - Starting time of changing workpiece weight [sec].
    :parm start_time: float - Transition time of changing workpiece weight [sec].
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return None


def reset_workpiece_weight() -> int:
    """
    This function initializes the weight data of the material to initialize the algorithm before measuring the weight of the material.

    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def estimate_load():
    return None


def get_normal(x1, x2, x3) -> List[float]:
    """
    This function returns the normal vector of a surface consisting of three points (posx) in the task space.
    This direction is clockwise.

    :param x1: posx - point 1 on plane
    :param x2: posx - point 2 on plane
    :param x3: posx - point 3 on plane
    :return: float[3] - normal vector
    """
    # TODO
    return None


def parallel_axis(*args, **kargs) -> int:
    """
    This  function  matches the  normal  vectorofthe  plane  consists  of  points(x1,  x2,  x3)  based  on
    the ref coordinate(refer  to  get_normal(x1,  x2,  x3))and  the  designated  axis  of  the  tool  frame.
    The  current position is maintained as the TCP position of the robot.

    :param x1: posx - point 1 on plane
    :param x2: posx - point 2 on plane
    :param x3: posx - point 3 on plane
    :param vect: float[3] - alternative to x1, x2, x3
    :param ref: int - reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference). Default: DR_BASE.
    :param axis: int - (DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z)
    :return: int - (0 -> Success, Negative value -> Error)
    """
    # TODO
    return None


def align_axis(*args, **kargs) -> int:
    """
    This  function  matches the  normal  vectorofthe  plane  consists  of  points(x1,  x2,  x3)  based  on
    the ref coordinate(refer to get_normal(x1,  x2,  x3)) and the designated axis of the tool frame.
    The robot TCP moves to the pos position.

    :param x1: posx - point 1 on plane
    :param x2: posx - point 2 on plane
    :param x3: posx - point 3 on plane
    :param vect: float[3] - alternative to x1, x2, x3
    :param pos: posx - target position
    :param ref: int - reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference). Default: DR_BASE.
    :param axis: int - (DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z)
    :return: int - (0 -> Success, Negative value -> Error)
    """
    # TODO
    return None


def is_done_bolt_tightening(m=0, timeout=0, axis=None) -> int:
    """
    This function monitors the tightening torque of the tool and returns True if the set torque (m) is reached within
    the given time and False if the given time has passed.

    :param m: float - target torque
    :param timeout: float - monitoring duration.
    :param axis: int - (DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z)
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def joint_compliance_ctrl(stj=[100, 100, 100, 100, 100, 100], time=0):
    global _robodk_control_mode
    _robodk_control_mode = _ROBODK_TORQUE_CONTROL


def set_stiffnessj(stj=[100, 100, 100, 100, 100, 100], time=0):
    return None


def release_compliance_ctrl() -> int:
    """
    This function terminates compliance control and begins position control at the current position.

    :return: int - (0 -> Success, Negative value -> Error)
    """
    global _robodk_control_mode
    _robodk_control_mode = _ROBODK_POSITION_CONTROL
    return 0


def task_compliance_ctrl(stx=[3000, 3000, 3000, 200, 200, 200], time=0) -> int:
    """
    This function begins task compliance control based on the preset reference coordinate system.

    :param stx: float[6] - Three translational stiffness, Three rotational stiffness. Default: [3000, 3000, 3000, 200, 200, 200].
    :param time: Stiffness varying time [sec].
    :return: int - (0 -> Success, Negative value -> Error)
    """
    global _robodk_control_mode
    _robodk_control_mode = _ROBODK_TORQUE_CONTROL
    return 0


def set_stiffnessx(stx=[500, 500, 500, 100, 100, 100], time=0) -> int:
    """
    This function sets the stiffness valuebased on the global coordinate(refer to set_ref_coord()).
    The linear transition  from  the  current  or default stiffness is performed during the time
    given as STX.
    The  user-defined ranges of the translational stiffness and rotational stiffness are 0-20000N/m and 0-400Nm/rad, respectively.

    :param stx: float[6] - Three translational stiffness, Three rotational stiffness. Default: [3000, 3000, 3000, 200, 200, 200].
    :param time: Stiffness varying time [sec].
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def set_zaxis_nt(*args, **kargs):
    return None


def set_desired_force(fd=[0, 0, 0, 0, 0, 0], dir=[0, 0, 0, 0, 0, 0], time=0, mod=DR_FC_MOD_ABS) -> int:
    """
    This function defines the target force, direction,translation time, and mode for force control based on the global coordinate.

    :param fd: float[6] - Three translational target forces, Three rotational target moment.
    :param dir: int[6] - Force control in the corresponding direction if 1, Compliance control in the corresponding direction if 0.
    :param time: float - Transition time of target force to take effect [sec].
    :param mod: int - (DR_FC_MOD_ABS: Force control with absolute value, DR_FC_MOD_REL: force control with relative value to initial state (the instance when this function is called)
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def release_force(time=0) -> int :
    """
    This  function  reduces  the  force  control  target  value  to  0  through  the  time  value  and  returns  the  task space to adaptive control.

    :param time: float - Time needed to reduce the force.
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def check_position_condition(axis, min=DR_COND_NONE, max=DR_COND_NONE, ref=None, mod= DR_MV_MOD_ABS, pos=None) -> bool:
    """
    This function checks the status of the given position. This condition can be repeated with the while or if statement.
    Axis and pos of input paramets are based on the ref coordinate.In case of ref=DR_TOOL, pos should be defined in BASE coordinate.

    :param axis: int - (DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z).
    :param min: float - Minimum value
    :param max: float - Maximum value
    :param ref: int - reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference). Default: DR_BASE.
    :param mod: int - (DR_MV_MOD_ABS: Absolulte, DR_MV_MOD_REL: Relative)
    :param pos: posx - Check position.
    :return: bool - True -> The condition is True, False -> The condition is False.
    """
    return True


def check_force_condition(axis, min=DR_COND_NONE, max=DR_COND_NONE, ref=None) -> bool:
    """
    This function checks the status of the given force. It disregards the force direction and only compares the  sizes.
    This condition can be  repeated  with  the  while  or if  statement.Measuring  the  force,
    axis is based on the ref coordinate and measuring the moment, axis is based on the tool coordinate.

    :param axis: int - axis (DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z, DR_AXIS_A, DR_AXIS_B, DR_AXIS_C)
    :param min: float - Minimum value
    :param max: float - Maximum value
    :param ref: int - reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference).
    :return:  bool - True -> The condition is True, False -> The condition is False.
    """
    return True


def check_orientation_condition(axis, min=None,         max=None,         ref=None, mod = None,          pos=None) -> bool:
    """
    This function checks the difference between the current pose and the specified pose of the robot end effector.
    It  returns  the  difference  between  the  current  pose  and  the  specified  pose  in  rad  with  the algorithm
    that  transforms  it  to  a  rotation  matrix  using the "AngleAxis" technique.
    It  returns True  if the difference is positive (+) and False if the difference is negative (-).
    It is used to check if the difference between the current pose and the rotating angle range is + or -.
    For example, the function can use the direct teaching position to check if the difference from the current position is + or -and then create the condition for the orientation limit.
    This condition can be repeated with the while or if statement.

    :param axis: int - axis (DR_AXIS_A, DR_AXIS_B, DR_AXIS_C)
    :param min: posx - minimum position
    :param max: posx - maximum position
    :param ref: int - reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference).
    :param mod: int - Movement basis (DR_MV_MOD_ABS: Absolute)
    :param pos:
    :return: bool - True -> The condition is True, False -> The condition is False.
    """
    return True


def get_orientation_error(xd, xc, axis) -> float:
    """
    This function returns the orientation error value between the arbitrary poses xd and xc of the axis.

    :param xd: posx
    :param xc: posx
    :param axis: int - axis (DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z)
    :return: Orientation error value.
    """
    return 0


DR_FIFO = 0
DR_LIFO = 1


def set_conveyor(name) -> int:
    """
    If conveyor information is configured in the UI, obtainID with the conveyor name to start the Conveyor Tracking
    Application from the program and execute the command for workpiece monitoring.
    Workpiece monitoring  is  performed  on  workpieces  triggered  in  the  conveyor,
    and  monitoring  continues  until  the program ends.

    :param name: string - Conveyor name.
    :return: Returns Conveyor ID if conveyor setting is successful
    """
    return None


def set_conveyor_ex(name=None, conv_type=0, encoder_channel=1, triggering_mute_time=0.0, count_per_dist=5000, conv_coord=posx(0,0,0,0,0,0), ref=DR_BASE, conv_speed=100.0, speed_filter_size=500, min_dist=0.0, max_dist=1000.0, watch_window=100.0, out_tracking_dist=10.0) -> int :
    """
    Configures  the  conveyor  and obtains  Conveyor  ID  to  allow  the  Conveyor  Tracking Application  to  start.
    After  the  command  is  executed,  it  monitors  workpieces  triggered  in  the  configured  conveyor  until  the program ends.
    It can be used when you need to set parameters manually if is unavailable to configure conveyor information through UI.

    :param name: string - Conveyor name.
    :param conv_type: int - Conveyor type (0: Linear, 1: Circular).
    :param encoder_channel: int - External encoder channel (1, 2).
    :param triggering_mute_time: float - It is the time (s) triggering (encoder reset, start workpiece tracking) is not performed when a triggering signal is received immediately after triggering.
    :param count_per_dist: int - Encoder count converted value per length (Linear: count/m, Circular: count/rad).
    :param conv_coord: posx - Fixed conveyor coordinates (based on Base/World coordinates, mm,  deg)
    :param ref: int - Reference coordinates of conveyor coordinates(DR_BASE: Base, DR_WORLD: World)
    :param conv_speed: float - Conveyor nominal velocity (Linear: mm/s, Circular:  deg/s).
    :param speed_filter_size: int - Moving Average FilterSize during conveyor velocity filtering.
    :param min_dist: float - Minimum conveyor work length (based on Triggering Switch, Linear: mm, Circular:  deg)
    :param max_dist: float - Maximum conveyor work length (based on Triggering Switch, Linear: mm, Circular:  deg)
    :param watch_window: float - Conveyor work standby monitoring length (based on minimum work length, Linear: mm, Circular:  deg)
    :param out_tracking_dist: float - Conveyor tracking release buffer section length (based on maximum work length, Linear: mm, Circular:  deg)
    :return: Returns Conveyor ID if conveyor setting is successful
    """
    return None


def get_conveyor_obj(conv_id, timeout=None, container_type=DR_FIFO, obj_offset_coord=None) -> int:
    """
    It returns the workpiece coordinate ID available for the job from the corresponding conveyor. When a function  is called,  it returns
    the workpiece  present  in  the  Watch  Zone  one  by  one  according  to  the container rule.

    :param conv_id: int - Conveyor ID.
    :param timeout: float - If no workpiece to return is present, it ends the standby and returns the function.
    :param container_type: int - Workpiece container type (DR_FIFO: first-in/first-out, DR_LIFO: last-in/last-out.
    :param obj_offset_coord: posx - Workpiece coordinates (mm,  deg) based on conveyor lock coordinate.
    :return: CONV_COORD. Conveyor user coordinate ID (121~150). Negative number: If no workpiece is present even after the timeout expires.
    """
    return None


def tracking_conveyor(conv_id) -> int:
    """
    The  robot  starts  Conveyor  Tracking.  It  returns  a  function  when  it reaches  conveyor  velocity  by accelerating from stop.

    :param conv_id: int - Conveyor ID
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return None


def untracking_conveyor(conv_id, time=None) -> int:
    """
    The robot ends Conveyor Tracking. Return is made when end motion is complete and velocity reaches 0.

    :param conv_id: int - Conveyor ID.
    :param time: float - Deceleration time (sec) to end Tracking.
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return None


def set_extenc_polarity(channel, polarity_A, polarity_B, polarity_Z, polarity_S):
    """
    It  configures  the  polarity  of  phase  A,  B  and  the  trigger  method  of  phase  S,  Zof  the  corresponding encoder channel.

    :param channel: int - Encoder channel (1, 2).
    :param polarity_A: int - Polarity of phase A (0: Phase A, 1: /Phase A
    :param polarity_B: int - Polarity of phase B (0: Phase B, 1: /Phase B)
    :param polarity_Z: int - Trigger method of Phase Z (0: Falling edge, 1: Rising edge.
    :param polarity_S: int - Trigger method of Phase S (0: Falling edge, 1: Rising edge.
    """
    pass

def set_extenc_mode(channel, mode_AB, pulse_AZ, mode_Z, mode_S, inverse_cnt):
    """
    It configures the operation mode of phase A, B, Z and S of the corresponding encoder channel.

    :param channel: int - Encoder channel (1, 2)
    :param mode_AB: int - Use of phase AB Mode (0 ~ 4).
    :param pulse_AZ: int - Pulse A Count per Pulse.
    :param mode_Z: int - Phase Z Use Mode (0 ~ 1).
    :param mode_S: int - Phase S Use Mode (0 ~ 1).
    :param inverse_cnt: - Encoder Count Direction.
    """
    pass


def get_extenc_count(channel) -> int:
    """
    Get the countvalueof the corresponding encoder channel.

    :param channel: Encoder channel (1, 2).
    :return: Current encoder count value of corresponding channel.
    """
    return 0


def clear_extenc_count(channel):
    """
    Reset counter valueof the corresponding encoder channel to 0.

    :param channel: Encoder channel (1, 2).
    """
    pass


def add_modbus_signal(ip, port, name, reg_type, index, value=0, slaveid=255) -> int:
    """
    This function registers the ModbusTCP signal. The Modbus I/O must be set in the Teach Pendant I/O set-up menu.
    Use this command only for testing if it is difficult to use the Teach Pendant.
    The Modbus menu is disabled in the Teach Pendant if it is set using this command.

    :param ip: str - IP address of the Modbus TCP module.
    :param port: int - Port number of the ModbusTCP module.
    :param name: str - Modbus signal name.
    :param reg_type: int - Modbus signal type (DR_MODBUS_DIG_INPUT, DR_MODBUS_DIG_OUTPUT, DR_MODBUS_REG_INPUT, DR_MODBUS_REG_OUTPU)
    :param index: int - Modbus signal index.
    :param value: int - Output when the type is DR_MODBUS_DIG_OUTPUT or DR_MODBUS_REG_OUTPUT (ignored otherwise).
    :param slaveid: Slave ID of the ModbusTCP module (0 - 255).
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def add_modbus_rtu_signal (name, reg_type, index, value=0, slaveid=1, port=None, baudrate=115200, bytesize=8, parity="N", stopbits=1):
    """
    This function registers the ModbusRTU signal. The Modbus I/O must be set in the Teach Pendant I/O set-up menu.
    Use this command only for testing if it is difficult to use the Teach Pendant. The Modbus menu is disabled in the
    Teach Pendant if it is set using this command.

    :param name: str - Modbus signal name.
    :param reg_type: int - Modbus signal type (DR_MODBUS_DIG_INPUT, DR_MODBUS_DIG_OUTPUT, DR_MODBUS_REG_INPUT, DR_MODBUS_REG_OUTPU)
    :param index: int - Modbus signal index.
    :param value: int - Output when the type is DR_MODBUS_DIG_OUTPUT or DR_MODBUS_REG_OUTPUT (ignored otherwise).
    :param slaveid: Slave ID of the ModbusTCP module (0 - 255).
    :param port: int - Port number of the ModbusTCP module.
    :param baudrate:
    :param bytesize:
    :param parity:
    :param stopbits:
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def add_modbus_signal_multi(ip, port, slaveid=255, name=None, reg_type=DR_HOLDING_REGISTER, start_address=0, cnt=1):
    """
    This function registers the ModbusTCP FC15 & FC16 multiple signal. The Modbus I/O must be set in the Teach
    Pendant I/O set-up menu. Use this command only for testing if it is difficult to use the Teach Pendant. The
    Modbus menu is disabled in the Teach Pendant if it is set using this command.

    :param ip: str - IP address of the Modbus TCP module.
    :param port: int - Port number of the ModbusTCP module.
    :param slaveid: Slave ID of the ModbusTCP module (0 - 255).
    :param name: str - Modbus signal name.
    :param reg_type: int - Modbus signal type (DR_MODBUS_DIG_INPUT, DR_MODBUS_DIG_OUTPUT, DR_MODBUS_REG_INPUT, DR_MODBUS_REG_OUTPU)
    :param start_address: int - Start address of Modbus multiple signal
    :param cnt: int - Count of Modbus multiple signal
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def add_modbus_rtu_signal_multi(slaveid=1, port=None, baudrate=115200, bytesize=8, parity="N", stopbits=1, name=None, reg_type=DR_HOLDING_REGISTER, start_address=0, cnt=1):
    """
    This function registers the ModbusRTU FC15 & FC16 multiple signal. The Modbus I/O must be set in the Teach
    Pendant I/O set-up menu. Use this command only for testing if it is difficult to use the Teach Pendant. The
    Modbus menu is disabled in the Teach Pendant if it is set using this command.

    :param name: str - Modbus signal name.
    :param reg_type: int - Modbus signal type (DR_MODBUS_DIG_INPUT, DR_MODBUS_DIG_OUTPUT, DR_MODBUS_REG_INPUT, DR_MODBUS_REG_OUTPU)
    :param index: int - Modbus signal index.
    :param value: int - Output when the type is DR_MODBUS_DIG_OUTPUT or DR_MODBUS_REG_OUTPUT (ignored otherwise).
    :param slaveid: Slave ID of the ModbusTCP module (0 - 255).
    :param port: int - Port number of the ModbusTCP module.
    :param baudrate:
    :param bytesize:
    :param parity:
    :param stopbits:
    :return: int - (0 -> Success, Negative value -> Error)
    :param start_address: int - Start address of Modbus multiple signal
    :param cnt: int - Count of Modbus multiple signal
    """
    return 0


def get_modbus_slave(address) -> int:
    """
    It is used to import values by approaching the General Purpose Register area of the Modbus TCP Slave.

    :param address: int - Address value of the GPR area to read (128~255)
    :return: Corresponding register value.
    """
    return 0


def set_modbus_slave(address, val) -> int:
    """
    It is used to export values to the General Purpose Register area of the Modbus TCP Slave.

    :param address: Address value of GPR area (128~255).
    :param val: 2byte value (0~65535
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def del_modbus_signal(name) -> int:
    """
    This function deletes the registered Modbus signal. The Modbus I/O must be set in the Teach Pendant I/O set-up menu.
    Use this command only for testing if it is difficult to use the Teach Pendant.
    The Modbus menu is disabled in the Teach Pendant if it is set using this command.

    :param name: str - Name of the registered Modbus signal.
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def del_modbus_signal_multi(name) -> int:
    """
    This function deletes the registered Modbus multiple signal. The Modbus I/O must be set in the Teach Pendant I/O
    set-up menu. Use this command only for testing if it is difficult to use the Teach Pendant. The Modbus menu is
    disabled in the Teach Pendant if it is set using this command.

    :param name: str - Name of the registered Modbus signal.
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def wait_digital_input(index, val, timeout=None) -> int:
    """
    This function waits until the signal value of the digital input register of the controller becomes val (ON or OFF).
    The waiting time can be changed with a timeout setting. The waiting time ends, and the result is returned if the waiting
    time has passed. This function waits indefinitely if the timeout is not set.

    :param index: int - A number 1 -16 which means the I/O index mounted on the controller.
    :param val: int - I/O value (ON : 1, OFF : 0)
    :param timeout: float - Waiting time (sec). This function waits indefinitely if the timeout is not set.
    :return: int - (0 -> Success, Negative value -> Error/Timeout)
    """
    return 0


def wait_tool_digital_input(index, val, timeout=None) -> int:
    """
    This function waits until the digital input signal value of the robot tool becomes val (ON or OFF).
    The waiting time can be changed with a timeout setting. The waiting time ends, and the result is returned if the
    waiting time has passed. This function waits indefinitely if the timeout is not set.

    :param index: int - A number in 1 - 6 which means the I/O indexmounted on the robot arm
    :param val: int - I/O value (ON : 1, OFF : 0)
    :param timeout: float - Waiting time (sec). This function waits indefinitely if the timeout is not set.
    :return: int - (0 -> Success, Negative value -> Error/Timeout)
    """
    return 0


def wait_modbus_input(iobus, val, timeout=None)  -> int:
    """
    This function waits until the specified signal value of the Modbus digital I/O becomes val (ON or OFF).
    The waiting time can be changed with a timeout setting.
    The waiting time ends, and the result is returned if the waiting time has passed.
    This function waits indefinitely if the timeout is not set.

    :param iobus:  str - Modbus name (set in the TP).
    :param val: int - I/O value (ON : 1, OFF : 0) or Value for Modbus analog I/O.
    :param timeout: float - Waiting time (sec). This function waits indefinitely if the timeout is not set
    :return: int - (0 -> Success, Negative value -> Error/Timeout)
    """
    return 0


def wait(second):
    """
    This function waits for the specified time.

    :param second: float - Time [sec].
    """
    time.sleep(second)


def get_digital_input(index) -> int:
    """
    This function  reads  the signals  from  digital contact  points  of  the controller and  reads  the digital  input contact value.

    :param index: A number 1 -16 which means the contact number of I/O mounted on the controller.
    :return: int (ON, OFF)
    """
    return ON


def get_digital_inputs(*args, **kargs) -> int:
    """
    This function reads the signals from multiple digital contact points of the controller.
    The digital signals of the contact points defined in bit_list are input at one.

    :param index: int[] - List of contact points to read
    :param bit_start: int - Beginning contact number for input signals (1~16) (Alternatively to index)
    :param bit_end: int - Ending contact number for input signals (1~16) (Alternatively to index)
    :return: int - Multiple contacts to be read at once(the value of the combination of the bit list where bit_start =LSB and bit_end=MSB
    """
    return 0


def get_analog_input(ch) -> float:
    """
    This function reads the channel value corresponding to the controller analog input.

    :param ch: int - (1 : channel 1, 2 : channel 2)
    :return: float - The analog input value of the specified channel (Current mode: 4.0~20.0 [mA], Voltage mode: 0~10.0 [V])
    """
    return 0


def get_tool_digital_input(index) -> int:
    """
    This function reads the signal of the robot tool from the digital contact point.

    :param index: int - I/O contact number (1-6) mounted on the robot tool.
    :return: int (ON, OFF)
    """
    return ON


def get_tool_digital_inputs(*args, **kargs) -> int:
    """
    This function reads the signal of the robot tool from the digital contact point. The digital signals of the contact points defined in bit_list are input at one.

    :param index: int[] - List of contact points to read
    :param bit_start: int - Beginning contact number for input signals (1~6) (Alternatively to index)
    :param bit_end: int - Ending contact number for input signals (1~6) (Alternatively to index)
    :return: int - Multiple contacts to be read at once(the value of the combination of the bit list where bit_start =LSB and bit_end=MSB
    """
    return 0


def get_modbus_input(iobus) -> int:
    """
    This function reads the signal from the Modbus Slave unit parameters.

    :param iobus: str - Modbus name (set in the TP).
    :return: ON or Off in the case of the Modbus digital I/O. The register value in the case of the Modbus analog module.
    """
    return 0


def get_modbus_inputs(*args, **kargs) -> int:
    """
    This function reads multiple signals from the Modbus Slave unit.

    :param iobus_list: str[] - Modbus input name list (set in the TP)signal type can be used only in the following cases: DR_MODBUS_DIG_INPUT, DR_MODBUS_DIG_OUTPUT.
    :return: Multiple signals to be read at once(the value of the combination of iobus_list where the first value is LSB and the last value is MSB)
    """
    return 0


def set_digital_output(index, val=None) -> int:
    """
    This  function  sends  a  signal  at  the  digital  contact  point  of  the controller.
    A  value  saved  in  the  digital output register is output as a digital signal.

    :param index: int - I/O contact number mounted on the controller. A positive number means ON while a negative number means OFF.
    :param val: int - (ON, OFF)
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def set_digital_outputs(*args, **kargs) -> int:
    """
    This function sends a signal to multiple digital output contact points of the controller.
    The digital signals of the contact points defined in bit_list are output at one.

    :param bit_list: int[] - List of multiple output contacts. The positive contact number outputs ON: 1~16. The negative contact number outputs OFF: -1~-16.
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def set_modbus_output_multi(iobus, val_list):
    """
    This function sends the signal to an external Modbus system.
        Function Code 15 Write Multiple Coil Register
        Function Code 16 Write Multiple Holding Register

    :param iobus: Modbus multiple signal name (set in the TP)
    :param val_list: Value list of modbus multiple signal
    """
    return 0


def get_modbus_inputs_list(iobus_list):
    """
    It is the command for reading multiple register type open signals from an external Modbus Slave unit.

    :param iobus_list: list(string)
    :return: Number values read, List of multiple signal values read simultaneously
    """
    return 0, []


def set_analog_output(ch, val) -> int:
    """
    This function outputs the channel value corresponding to the controller analog output.

    :param ch: int - (1 : channel 1, 2 : channel 2)
    :param val: float - analog output value. Current mode: 4.0~20.0 [mA], Voltage mode: 0~10.0 [V].
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def get_modbus_input_multi(iobus):
    """
    This function reads the signal from the Modbus Slave unit.

    :param iobus: Modbus multi signal name (set in the TP)
    :return: List of values corresponding to the number of signals
    """
    return []

def set_mode_analog_output(ch, mod) -> int:
    """
    This function sets the channel mode of the controller analog output.

    :param ch: int - (1 : channel 1, 2 : channel 2)
    :param mod: int - analog io mode. (DR_ANALOG_CURRENT: Current mode, DR_ANALOG_VOLTAGE: Voltage mode)
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def set_mode_analog_input(ch, mod) -> int:
    """
    This function sets the channel mode of the controller analog input.

    :param ch: int - (1 : channel 1, 2 : channel 2)
    :param mod: int - analog io mode. (DR_ANALOG_CURRENT: Current mode, DR_ANALOG_VOLTAGE: Voltage mode)
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def set_tool_digital_output(index, val=None) -> int:
    """
    This function sends the signal of the robot tool from the digital contact point.

    :param index: int - I/O contact number mounted on the robot arm. A positive number means ON while a negative number means OFF.
    :param val: int - I/O value: The value to output.
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def set_tool_digital_outputs(*args, **kargs) -> int:
    """
    This function sends the signal of the robot tool from the digital contact point.
    The digital signals of the contact points defined in bit_list are output at one.

    :param bit_list: List of multiple output contacts. The positive contact number outputs ON: 1~6. The negative contact number outputs OFF: -1~-6.
    :param bit_start: int - Beginning contact number for input signals (1~6) (Alternatively to index)
    :param bit_end: int - Ending contact number for input signals (1~6) (Alternatively to index)    :return:
    :param Val: int - Output value
    """
    return 0


def set_modbus_output(iobus, val) -> int:
    """
    This function sends the signal to an external Modbus system.

    :param iobus: str - Modbus name (set in the TP)
    :param val: int - (ON, OFF)
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def set_modbus_outputs(*args, **kargs) -> int:
    """
    This function sends multiple signals to the Modbus Slave unit.

    :param iobus: str[] - Modbus name (set in the TP).
    :param value: int[] - I/O output value list.
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def set_tcp(name) -> int:
    """
    This function calls the name of the TCP registered in the Teach Pendant and sets it as the current TCP.

    :param name: str - Name of the TCP registered in the TP.
    :return: int - (0 -> Success, Negative value -> Error)
    """
    if _robodk_plugin_RDK is not None:
        tool = _robodk_plugin_RDK.Item(name, ITEM_TYPE_TOOL)
        if tool.Valid():
            _robodk_plugin_robot.setPoseTool(tool)

            # update tool frame
            tool_frame = _robodk_plugin_RDK.Item('DR_TOOL', ITEM_TYPE_FRAME)
            if not tool_frame.Valid():
                tool_frame = _robodk_plugin_RDK.AddFrame("DR_TOOL")

            tool_frame.setVisible(False)
            pose_tool = _robodk_plugin_robot.PoseTool()
            tool_frame.setPose(pose_tool)
    return 0


def set_tool(name, start_time=DR_COND_NONE, transition_time=DR_COND_NONE) -> int:
    """
    This function activates the tool of the entered name among the tool information registered in the Teach Pendant.

    :param name: str - Tool name registered in the Teach Pendant.
    :param start_time: float - Tool weight is changed after setting time.
    :param transition_time: float - Tool weight is changed during setting time.
    :return: int - (0 -> Success, Negative value -> Error)
    """
    set_tcp(name)
    return 0


def tp_popup(message, pm_type=DR_PM_MESSAGE,type=0):
    """
    This  function  provides  a  message  to  users  through  the  Teach  Pendant.
    The  higher  level  controller receives the string and displays it in the popup window,
    and the window must be closed by a user’s confirmation.

    :param message: str - Message provided to the use.
    :param pm_type: int - Message type (DR_PM_MESSAGE, DR_PM_WARNING, DR_PM_ALARM)
    :param type: int - button type of TP pop message (0 : show Stop & Resume button, 1 : show Stop button)
    :return: int - (0 -> Success, Negative value -> Error)
    """
    mbox(message)


def tp_log(message) -> int:
    """
    This function records the user-written log to the Teach Pendant.

    :param message: Log message.
    """
    print(str(message))
    return 0


def tp_progress(cur_progress, total_progress) -> int:
    """
    This  function  provides  a  message  to  users  through  the  Teach  Pendant.
    The  higher  level  controller receives the runtime data when a patterned program is run and is displayed in the GUI.

    :param cur_progress: int - Value at the current step.
    :param total_progress: int - Value at the final step.
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def tp_get_user_input(message, input_type):
    """
    This function receives the user input data through the Teach Pendant.

    :param message: str - Character string message to be displayed on the TP user input window.
    :param input_type: int - TP user input message type (DR_VAR_INT: Integer type, DR_VAR_FLOAT: Real number type, DR_VAR_STR: Character string)
    :return: int - (0 -> Success, Negative value -> Error)
    """
    result = mbox(message,entry=True)
    if input_type == DR_VAR_FLOAT:
        return float(result)
    elif input_type == DR_VAR_INT:
        return int(result)
    elif input_type == DR_VAR_STR:
        return str(result)
    return None


def get_coordinate_info(ref):
    return None


def wait_manual_guide() -> int:
    """
    This function enables the user to perform hand guiding (changing the position of the robot by  pressing  the  Direct  Teach
    button  in  the  cockpit  or  the  TP)  during  the  execution  of  the program. The user executes the next command in
    one of the following two ways after hand guiding is completed (unless the program is terminated, it will wait at the command until
    one of the following is executed after the user performs hand guiding).
    1) The user presses the "OK"or "Finish"button on the "Hand Guiding Execution"popup window generated from the TP.
    2) A signal is applied to the digital input channel specified for "Manual guide release"in the safety I/O settings.
    The  current  TCP  position  and  the  TCP  position  of  the  hand guided  robot  must  be  in  the collaborative
    workspace in order to execute this command properly. Run the command after specifying the hand guiding area as the
    collaborative workspace and enabling it. An error is generated, and the program is terminated to ensure worker safety
    if the current position or hand guiding deviates from the collaborative workspace.

    :return: int - (0 -> Success, Negative value -> Error)
    """
    tp_popup("Waiting manual guide...")
    return 0


def motion_pause():
    return None


def motion_resume():
    return None


def get_motor_temperature():
    return None


def get_motor_input_current():
    return None


def set_singular_handling(mode = DR_AVOID) -> int:
    """
    In case of path deviation due to the effect of singularity in task motion, user can select the response policy.
    The mode can be set as follows.
    -Automatic avoidance mode(Default) : DR_AVOID
    -Path first mode : DR_TASK_STOP
    -Variable velocity mode : DR_VAR_VEL
    The  default  setting  is  automatic  avoidance  mode,  which  reduces  instability  caused  by singularity,
    but  reduces  path  tracking  accuracy.  In  case  of  path  first  setting,  if  there  is possibility of instability
    due to singularity, a warning message is output after deceleration and then the corresponding task is terminated.
    In case of  variable  velocity mode setting, TCP velocity would be changed in singular region to reduce instability
    and maintain path tracking accuracy.


    :param mode: int - (DR_AVOID : Automatic avoidance mode, DR_TASK_STOP : Deceleration/ Warning/ Task termination, DR_VAR_VEL : Variable velocity mode)
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def set_singularity_handling(mode = DR_AVOID) -> int:
    """
    In case of path deviation due to the effect of singularity in task motion, user can select the response policy.
    The mode can be set as follows.
    -Automatic avoidance mode(Default) : DR_AVOID
    -Path first mode : DR_TASK_STOP
    -Variable velocity mode : DR_VAR_VEL
    The  default  setting  is  automatic  avoidance  mode,  which  reduces  instability  caused  by singularity,
    but  reduces  path  tracking  accuracy.  In  case  of  path  first  setting,  if  there  is possibility of instability
    due to singularity, a warning message is output after deceleration and then the corresponding task is terminated.
    In case of  variable  velocity mode setting, TCP velocity would be changed in singular region to reduce instability
    and maintain path tracking accuracy.


    :param mode: int - (DR_AVOID : Automatic avoidance mode, DR_TASK_STOP : Deceleration/ Warning/ Task termination, DR_VAR_VEL : Variable velocity mode)
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def wait_nudge() -> int:
    """
    This function enables users to resume the execution of the program through the user’s nudge input  (applying
    external  force  to  the  robot)  when  the  execution  of  the  program  is  paused. When  the  external
    force greater  than  the  force  threshold,  it  will  proceed  to  the  following command after the resume time,
    where the force threshold and the resume time are set at the collaborative workspace setting menu. This command can
    be used as an interlock during the program.However,  if  the  robot’s  configuration  is  in  the  singularity  area,
    or  if  the  force  is  applied continuously after the nudge input, warning will be occurred for safety.For  this
    function  is  allowed  to  execute  within  the  collaborative  workspace,  please  set  the collaborative workspace,
    activate it, and assure the TCP position is in this workspace when this command is performed in advance.

    :return: int - (0 -> Success, Negative value -> Error)
    """
    tp_popup("Waiting nudge...")
    return 0


def enable_alter_motion(n, mode, ref, limit_dPOS, limit_dPOS_per) -> int:
    """
    enable_alter_motion() and alter_motion() functions enable to alter motion trajectory.This  function  sets  the
    configurations  for  altering  function  and  allows  the  input  quantity  of alter_motion()  to  be  applied  to
    motion  trajectory.  The  unit  cycle  time  of  generating  alter motion is 100msec. Cycle time(n*100msec) can be
    changed through input parameter n. This function   provide   2   modes(Accumulation   mode,   Increment   mode).
    Input   quantity   of alter_motion() can  be  applied  to  motion  trajectory  in  two  ways  as  accumulated
    value  or increment  value.  In  accumulation  mode,  the  input  quantity  means  absolute  altering amount(dX,dY,dZ,dRX,dRY,dRZ)
    from   current   motion   trajectory.   On   the   contrary   in increment  mode,  the  quantity  means  increment
    value  from  the  previous  absolute  altering amount. The reference coordinate can be changed through input parameter ref.
    Limitations of accumulation amout and increment amount can be set through input paramet limit_dPOS (accumulated limit)
    and limit_dPOS_per(increment input limit during 1 cycle). The actual alter amount is limited to these limits.

    :param n: int - Cycle time number.
    :param mode: int . Mode (DR_DPOS : accumulation amount, DR_VEL : increment amount)
    :param ref: int - reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference).
    :param limit_dPOS: float[2] - First value : limitation of position[mm]. Second value : limitation of orientation[deg].
    :param limit_dPOS_per: float[2] - Fist value : limitation of position[mm]. Second value : limitation of orientation[deg].
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def alter_motion(pos) -> int:
    """
    This function applies altering amount of motion trajectory when the alter function is activated.
    The meaning of the input values is defined from enable_alter_motion().

    :param pos: float[6] - position list.
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def disable_alter_motion() -> int:
    """
    This function deactivates alter motion.

    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def set_tool_shape(name) -> int:
    """
    This function activates the tool shape information of the entered name among the tool shape information registered
    in the Teach Pendant.

    :param name: str - Tool name registered in the Teach Pendant.
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0

def traceit(frame, event, arg):
    return None

def drl_report_line(option) -> int:
    """
    This  command  is  used  to  turn  ON /OFF  the  execution  line  display  function  when  the  DRL  script  is running.
    When  the  run  line  display  function  is  turned  OFF,  the  time  required  to  execute  the  run  line display
    function is reduced, which significantly speeds up the execution of the DRL.

    :param option: int - Whether to display the DRL execution line ON(1)OFF(0).
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def calc_coord(x1, x2, x3, x4, ref, mod):
    """
    This function returns anew user cartesiancoordinate system by using up to 4 input poses([x1]~[x4]),
    input  mode [mod]and  the  reference  coordinate  system [ref].The  input  mode  is  only  valid  when  the number of
    input robot poses is 2.In the case thatthe number of input poses is 1, the coordinate system is calculated using the
    position and orientation of x1.In the case that the number of input poses is 2 and the input mode is 0, X-axis is
    defined bythe direction from  x1  tox2,  and  Z-axis  is  defined bythe  projection  of  the  current  Tool-Zdirection
    onto  the  plane orthogonal to the x-axis. The origin is the position of x1.In the case that the number of input poses
    is 2 and the input mode is 1, X-axis is defined bythe direction from x1 to x2, and Z-axis is defined by the projection
    of the z direction of x1 onto the plane orthogonal to the X-axis. The origin is the position of x1.In  the case  that
    the  number  of  input  poses  is  3,  X-axis  is  defined  by the  direction  from  x1  to  x2.  If a vector  v  is
    the direction  from x1  to  x3,  Z-axis  is  defined  by the cross  product of  X-axis  and  v  (X-axis cross v).
    The origin is the position of x1.In the case that the number of input poses is 4, the definition of axes is identical
    to the case that the number of input poses is 3, but the origin is the position of x.

    :param x1:
    :param x2:
    :param x3:
    :param x4:
    :param ref: int - reference coordinate (DR_BASE, DR_WORLD, DR_TOOL, User Reference).
    :param mod: int - input mode(only valid when the number of input poses is 2).
    :return: posx - Successful coordinate calculationPosition information of the calculated coordinate.
    """
    if _robodk_plugin_RDK is not None:
        u1 = [x2[0] - x1[0], x2[1] - x1[1], x2[2] - x1[2]] + [0, 0, 0]
        ux = unit_pose(u1) + [0, 0, 0]
        u3 = [x3[0] - x1[0], x3[1] - x1[1], x3[2] - x1[2]] + [0, 0, 0]
        u3_unit = unit_pose(u3) + [0, 0, 0]
        uz = cross_pose(ux, u3_unit) + [0, 0, 0]
        uy = cross_pose(uz, ux)

        H = eye()
        H.setPos([x4[0], x4[1], x4[2]])
        H.setVX([ux[0], ux[1], ux[2]])
        H.setVY([uy[0], uy[1], uy[2]])
        H.setVZ([uz[0], uz[1], uz[2]])

        frame_pose = Pose_2_Comau(H)
        return frame_pose


def set_user_cart_coord(*args, **kwargs):
    """
    This  function set anew user cartesiancoordinate  system  using input pose[pos]and  reference coordinate system[ref].
    Up to 20 user coordinate systems can be set including the coordinate systems set within Workcell Item.
    Since the coordinate system set by this functionis removed when the program is terminated, setting new coordinate
    systems within Workcell Item is recommended for maintaining the coordinate information.

    :param x1:
    :param x2:
    :param x3:
    :param pos:
    :param ref: int - reference coordinate (DR_BASE, DR_WORLD).
    :return: Successful coordinate setting. Set coordinate ID (101 -200).
    """
    global _robodk_frame_count

    if _robodk_plugin_RDK is not None:
        args_length = len(args)
        kwargs_length = len(kwargs)

        if args_length + kwargs_length == 2:
            # single position case
            if args_length == 0:
                pos = kwargs['pos']
                ref = kwargs['ref']
            if args_length == 1:
                pos = args[0]
                ref = kwargs['ref']
            if args_length == 2:
                pos = args[0]
                ref = args[1]

            frame_pose = pos

        elif args_length + kwargs_length == 5:
            if args_length == 0:
                x1 = kwargs['x1']
                x2 = kwargs['x2']
                x3 = kwargs['x3']
                pos = kwargs['pos']
                ref = kwargs['ref']
            if args_length == 1:
                x1 = args[0]
                x2 = kwargs['x2']
                x3 = kwargs['x3']
                pos = kwargs['pos']
                ref = kwargs['ref']
            if args_length == 2:
                x1 = args[0]
                x2 = args[1]
                x3 = kwargs['x3']
                pos = kwargs['pos']
                ref = kwargs['ref']
            if args_length == 3:
                x1 = args[0]
                x2 = args[1]
                x3 = args[2]
                pos = kwargs['pos']
                ref = kwargs['ref']
            if args_length == 4:
                x1 = args[0]
                x2 = args[1]
                x3 = args[2]
                pos = args[3]
                ref = kwargs['ref']
            if args_length == 5:
                x1 = args[0]
                x2 = args[1]
                x3 = args[2]
                pos = args[3]
                ref = args[4]

            u1 = [x2[0] - x1[0], x2[1] - x1[1], x2[2] - x1[2]] + [0, 0, 0]
            ux = unit_pose(u1) + [0, 0, 0]
            u3 = [x3[0] - x1[0], x3[1] - x1[1], x3[2] - x1[2]] + [0, 0, 0]
            u3_unit = unit_pose(u3) + [0, 0, 0]
            uz = cross_pose(ux, u3_unit) + [0, 0, 0]
            uy = cross_pose(uz, ux)

            H = eye()
            H.setPos([pos[0], pos[1], pos[2]])
            H.setVX([ux[0], ux[1], ux[2]])
            H.setVY([uy[0], uy[1], uy[2]])
            H.setVZ([uz[0], uz[1], uz[2]])

            frame_pose = Pose_2_Comau(H)

        elif args_length + kwargs_length == 4:
            # versor case case
            if args_length == 0:
                u1 = kwargs['u1']
                v1 = kwargs['v1']
                pos = kwargs['pos']
                ref = kwargs['ref']
            if args_length == 1:
                u1 = args[0]
                v1 = kwargs['v1']
                pos = kwargs['pos']
                ref = kwargs['ref']
            if args_length == 2:
                u1 = args[0]
                v1 = args[1]
                pos = kwargs['pos']
                ref = kwargs['ref']
            if args_length == 3:
                u1 = args[0]
                v1 = args[1]
                pos = args[2]
                ref = kwargs['ref']
            if args_length == 4:
                u1 = args[0]
                v1 = args[1]
                pos = args[2]
                ref = args[3]

            ux = unit_pose(u1 + [0, 0, 0]) + [0, 0, 0]
            u3_unit = unit_pose(v1 + [0, 0, 0]) + [0, 0, 0]
            uz = cross_pose(ux, u3_unit) + [0, 0, 0]
            uy = cross_pose(uz, ux)

            H = eye()
            H.setPos([pos[0], pos[1], pos[2]])
            H.setVX([ux[0], ux[1], ux[2]])
            H.setVY([uy[0], uy[1], uy[2]])
            H.setVZ([uz[0], uz[1], uz[2]])

            frame_pose = Pose_2_Comau(H)

        elif args_length + kwargs_length == 3:
            # versor case case
            if args_length == 0:
                u1 = kwargs['u1']
                v1 = kwargs['v1']
                pos = kwargs['pos']
                ref = DR_BASE
            if args_length == 1:
                u1 = args[0]
                v1 = kwargs['v1']
                pos = kwargs['pos']
                ref = DR_BASE
            if args_length == 2:
                u1 = args[0]
                v1 = args[1]
                pos = kwargs['pos']
                ref = DR_BASE
            if args_length == 3:
                u1 = args[0]
                v1 = args[1]
                pos = args[2]
                ref = DR_BASE

            ux = unit_pose(u1+ [0, 0, 0]) + [0, 0, 0]
            u3_unit = unit_pose(v1+ [0, 0, 0]) + [0, 0, 0]
            uz = cross_pose(ux, u3_unit) + [0, 0, 0]
            uy = cross_pose(uz, ux)

            H = eye()
            H.setPos([pos[0], pos[1], pos[2]])
            H.setVX([ux[0], ux[1], ux[2]])
            H.setVY([uy[0], uy[1], uy[2]])
            H.setVZ([uz[0], uz[1], uz[2]])

            frame_pose = Pose_2_Comau(H)

        p = Comau_2_Pose(frame_pose)
        frame_id = _robodk_frame_count
        frame_name = "DR_FRAME_"+str(frame_id)

        # add frame
        frame = _robodk_plugin_RDK.Item(frame_name, ITEM_TYPE_FRAME)
        if not frame.Valid():
            frame = _robodk_plugin_RDK.AddFrame(frame_name)

        frame.setPose(p)
        frame.setParent(_robodk_plugin_get_ref_frame(ref))
        _robodk_plugin_ref_map[frame_id] = frame
        _robodk_frame_count = _robodk_frame_count + 1
        return frame_id


def overwrite_user_cart_coord(id, pos, ref) -> int:
    """
    This  function changes the  pose and  reference  coordinate  system  of  the requested  user coordinate system [id]
    with the [pos] and [ref], respectively.

    :param id: int - coordinate ID
    :param pos:
    :param ref: int - reference coordinate (DR_BASE, DR_WORLD).
    :return: Successful coordinate setting. Set coordinate ID (101 -200).
    """
    if _robodk_plugin_RDK is not None:
        p = Comau_2_Pose(pos)
        frame_name = "DR_FRAME_" + str(id)

        # add frame
        frame = _robodk_plugin_RDK.Item(frame_name, ITEM_TYPE_FRAME)
        if not frame.Valid():
            frame = _robodk_plugin_RDK.AddFrame(frame_name)

        frame.setPose(p)
        frame.setParent(_robodk_plugin_get_ref_frame(ref))
        _robodk_plugin_ref_map[id] = frame
    return id


def get_user_cart_coord(id) -> (posx, int):
    """
    This function returns the pose and reference coordinate system of the requested user coordinate system [id].

    :param id: int - coordinate ID.
    :return: posx, ref - Position and orientation information of the coordinate to get, Reference coordinateof the coordinate to get.
    """
    if _robodk_plugin_RDK is not None:
        frame = _robodk_plugin_get_ref_frame(id)
        frame_pose = frame.Pose()
        frame_pose_doosan = Pose_2_Comau(frame_pose)
        frame_parent = frame.Parent()

        for key, value in _robodk_plugin_ref_map.items():  # for name, age in dictionary.iteritems():  (for Python 2.x)
            if value == frame_parent:
                return frame_pose_doosan, key

        return frame_pose_doosan, DR_BASE
    return None, None


def robodk_close_gripper(object=None,distance=10000) -> bool:
    """
    Simulate gripper closure in roboDK simulator. Closest object is attached to active tooles.
    :return: bool - True if an object is grasped, False otherwise.
    """
    if _robodk_plugin_RDK is not None:
        tool = _get_active_tool()
        if tool is not None:
            if object is not None:
                itm = _robodk_plugin_RDK.Item(object)
                item = tool.AttachClosest(tolerance_mm=distance,list_objects=[itm])
            else:
                item = tool.AttachClosest(tolerance_mm=distance)
            return item.Valid()
    return False


def robodk_open_gripper():
    """
    Simulate gripper opening in roboDK simulator. Objects attached to active tool are detached.
    """
    if _robodk_plugin_RDK is not None:
        tool = _get_active_tool()
        if tool is not None:
            tool.DetachAll()

def sub_program_run(name):
    """
    It executes a subprogram saved as a separate file.

    :param name: str - Name of subprogram.
    :return: Module object of executed subprogram.
    """
    return None

def get_function_input(index):
    """
    This function reads a state of the function button from the process button device.

    :param index: int - It is the index of the function button mounted on the process button to be read. It is available 1 to 4..
    :return: int - (1 -> ON, 0 -> OFF, Negative value -> Error)
    """
    return ON


def wait_function_input(index, val, timeout=None):
    """
    This function wait for a state of the function button from the process button device.

    :param index: int - It is the index of the function button mounted on the process button to be read. It is available 1 to 4..
    :param val: bool - Value to wait for, ON or OFF.
    :param timeout: int - Max time to wait for.
    :return: int - (1 -> ON, 0 -> OFF, Negative value -> Error)
    """
    pass


def move_home(target):
    """
    Homing is performed by moving to the joint motion to the mechanical or user defined home position.
    According to the input parameter [target], it moves to the mechanical home defined in the system or
    the home set by the user.

    :param target: int - Target of home position: DR_HOME_TARGET_MECHANIC or DR_HOME_TARGET_USER.
    :return: int - (0 -> Success, Negative value -> Error)
    """
    movej(posj(0,0,0,0,0,0))


def set_singular_handling_force(mode=DR_SINGULARITY_ERROR):
    """
    The program is terminated by default through error processing when compliance or force control are used within the
    singularity area. It is possible to ignore error processing within the singularity area by changing the Mode setting.

    :param mode: DR_SINGULARITY_ERROR : Error processing, DR_SINGULARITY_IGNORE : Ignore error processing
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def set_palletizing_mode(mode=DR_OFF):
    """
    In palletizing application motion, path tracking and velocity can be maintained near wrist singular region using
    this function. In the existing singularity handling mode, automatic avoidance mode reduces path tracking accuracy
    and variable velocity mode causes changing velocity in singular region. In some cases, this function can be used for
    requiring path tracking accuracy and maintaining velocity, but it can cause instability in singular region.
    In palletizing application motion, there is no instability in singular region when planes consists of x,y axis based
    on tool coordinate and x,y axis based on base coordinate are paralled.

    :param mode: DR_OFF : Deactivate mode, DR_ON : Activate mode.
    :return: int - (0 -> Success, Negative value -> Error)
    """
    if mode == DR_ON:
        _robodk_plugin_RDK.Command("ToleranceSingularityBack", "-1.0")
        _robodk_plugin_RDK.Command("ToleranceSingularityElbow", "-1.0")
        _robodk_plugin_RDK.Command("ToleranceSingularityWrist", "-1.0")
    else:
        _robodk_plugin_RDK.Command("ToleranceSingularityBack", "1.0")
        _robodk_plugin_RDK.Command("ToleranceSingularityElbow", "1.0")
        _robodk_plugin_RDK.Command("ToleranceSingularityWrist", "1.0")
    return 0


def set_motion_end(mode=DR_CHECK_ON):
    """
    This command sets whether to operate the function to check the stop status of the robot after motion is completed.
    If it is set to DR_CHECK_OFF, the time required in the stop section between motions is reduced, so it can be used
    for the purpose of reducing the overall tact time. It is recommended to set it to DR_CHECK_ON when the tool is
    heavy and an accurate stop position is required for motion commands driven with high acceleration.

    :param mode: DR_CHECK_OFF: Deactivate mode, DR_CHECK_ON: Activate mode
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def get_force_control_state():
    """
    It monitors the state of compliance and force control.

    :return: Risk, Mode, Stx, Fd, ref
    """
    return None


def get_robot_model():
    """
    This is a command to read the model name of the robot.

    :return: Returns the model name in String type.
    """
    return _robodk_plugin_robot.Name()


def get_robot_serial_num():
    """
    This is a command to read the serial number of the robot.

    :return: Returns the robot serial number in String type.
    """
    return "XXXXX"


def _sol_to_doosan(robodk_sol):
    if robodk_sol == [0,0,0,0]:
        return 7
    elif robodk_sol == [0,0,1,0]:
        return 6
    elif robodk_sol == [0,1,0,0]:
        return 5
    elif robodk_sol == [0,1,1,0]:
        return 4
    elif robodk_sol == [0,1,1,0]:
        return 4
    elif robodk_sol == [1,0,0,0]:
        return 1
    elif robodk_sol == [1,0,1,0]:
        return 0
    elif robodk_sol == [1,1,0,0]:
        return 3
    elif robodk_sol == [1,1,1,0]:
        return 2

def focas_connect(ip, port, timeout):
    pass

def focas_disconnect(handle):
    pass

def focas_pmc_read_bit(handle, addr_type, start_num, bit_offset):
    pass

def focas_pmc_read_char(handle, addr_type, start_num, read_count):
    pass

def focas_pmc_read_word(handle, addr_type, start_num, read_count):
    pass

def focas_pmc_read_long(handle, addr_type, start_num, read_count):
    pass

def focas_pmc_read_float(handle, addr_type, start_num, read_count):
    pass

def focas_pmc_read_double(handle, addr_type, start_num, read_count):
    pass

def focas_pmc_write_bit(handle, addr_type, start_num, bit_offset, write_data):
    pass

def focas_pmc_write_char(handle, addr_type, start_num, write_data, write_count):
    pass

def focas_pmc_write_word(handle, addr_type, start_num, write_data, write_count):
    pass

def focas_pmc_write_long(handle, addr_type, start_num, write_data, write_count):
    pass

def focas_pmc_write_float(handle, addr_type, start_num, write_data, write_count):
    pass

def focas_pmc_write_double(handle, addr_type, start_num, write_data, write_count):
    pass

def focas_get_error_str(handle, error_code):
    pass


def check_robot_jts():
    """
    This is a command to check whether the robot is equipped with a joint torque sensor.

    :return: bool - (True -> with JTS, False -> without JTS)
    """
    return True


def check_robot_fts():
    """
    This is a command to check whether the robot is equipped with a force torque sensor.

    :return: bool - (True -> with FTS, False -> without FTS)
    """
    return True


def get_pattern_point(pos1, pos2, pos3, pos4, index=0, pattern=0, row=1, column=1, stack=1, thickness=0.0, point_offset=[0.0, 0.0, 0.0]):
    _pos1 = get_normal_pos(pos1, def_type=posx)
    _pos2 = get_normal_pos(pos2, def_type=posx)
    _pos3 = get_normal_pos(pos3, def_type=posx)
    _pos4 = get_normal_pos(pos4, def_type=posx)
    if type(point_offset) != list:
        raise DR_Error(DR_ERROR_TYPE, 'Invalid type : point_offset({0})'.format(type(point_offset)))
    elif len(point_offset) != 3:
        raise DR_Error(DR_ERROR_VALUE, 'Invalid list size : point_offset({0})'.format(len(point_offset)))
    return None #TODO


def set_oscillation_check(mode=DR_CHECK_OFF):
    if type(mode) != int:
        raise DR_Error(DR_ERROR_TYPE, 'Invalid type : mode')
    if mode < DR_CHECK_OFF or mode > DR_CHECK_ON:
        raise DR_Error(DR_ERROR_VALUE, 'Invalid value : mode({0})'.format(mode))
    return 0


def servoj(pos, vel=None, acc=None, time=None, v=None, a=None, t=None):
    """
    The command is the asynchronous motion command, and the next command is executed at the same time the motion begins.
    That motion follows the most recent target task position that is continuously delivered, within maximum velocity, acceleration.

    :param pos: posj - Target joints position [deg].
    :param vel: float or float[6] - maximum velocity (same to all axes) or velocity (to each axis) [deg/s].
    :param acc: float or float[6] - maximum acceleration (same to all axes) or acceleration (to each axis) [deg/s^2].
    :param time: float - Reach time [sec].
    :param v: float or float[6] - maximum velocity (same to all axes) or velocity (to each axis) [deg/s].
    :param a: float or float[6] - maximum acceleration (same to all axes) or acceleration (to each axis) [deg/s^2].
    :param t: float - Reach time [sec].
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return amovej(pos, vel=vel, acc=acc, time=time, v=v, a=a, t=t)


def servol(pos, vel=None, acc=None, time=None, v=None, a=None, t=None):
    """
    The command is the asynchronous motion command, and the next command is executed at the same time the motion begins.
    That motion follows the most recent target task position that is continuously delivered, within maximum velocity, acceleration.

    :param pos: posx - Task space position.
    :param vel: float or float[2] - maximum linear velocity or [linear velocity, angular velocity].
    :param acc: float or float[2] - maximum  linear acceleration or [linear acceleration, angular acceleration].
    :param time: float - Reach time [sec].
    :param v: float or float[2] - linear velocity or [linear velocity, angular velocity].
    :param a: float or float[2] - linear acceleration or [linear acceleration, angular acceleration].
    :param t: float - Reach time [sec].
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return amovel(pos, vel=vel, acc=acc, time=time, v=v, a=a, t=t)


def speedj(vel, acc=None, time=None, a=None, t=None):
    """
    The command is the asynchronous motion command, and the next command is executed at the same time the motion begins.
    That motion follows the most recent target task position that is continuously delivered, within maximum velocity, acceleration.

    :param vel: list (float[6]) - target joint velocity [deg/s]
    :param acc: float or float[6] - maximum acceleration (same to all axes) or acceleration (to each axis) [deg/s^2].
    :param time: float - Reach time [sec].
    :param a: float or float[6] - maximum acceleration (same to all axes) or acceleration (to each axis) [deg/s^2].
    :param t: float - Reach time [sec].
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def speedl(vel, acc=None, time=None, a=None, t=None):
    """
    The command is the asynchronous motion command, and the next command is executed at the same time the motion begins.
    That motion follows the most recent target task position that is continuously delivered, within maximum velocity, acceleration.

    :param vel: list (float[6]) - target Task velocity
    :param acc: float or float[2] - maximum  linear acceleration or [linear acceleration, angular acceleration].
    :param time: float - Reach time [sec].
    :param a: float or float[2] - linear acceleration or [linear acceleration, angular acceleration].
    :param t: float - Reach time [sec].
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 0


def set_deflection_comp_mode(mode=DR_ON):
    if type(mode) != int:
        raise DR_Error(DR_ERROR_TYPE, 'Invalid type : mode')
    if mode != DR_ON and mode != DR_OFF:
        raise DR_Error(DR_ERROR_VALUE, 'Invalid value : mode({0})'.format(mode))
    return 0


def get_system_time():
    return time.time()


_start_time = 0


def start_timer():
    """
    This is a command to measure the execution time of the simulation program of the controller. When used with
    the end_timer() command, it returns the execution time of the script between the two functions.
    :return: int - (0 -> Success, Negative value -> Error)
    """
    global _start_time
    _start_time = time.time()
    return 0


def end_timer():
    """
    This is a command to measure the execution time of the simulation program of the controller. When used with
    the start_timer() command, it returns the execution time of the script between the two functions.
    :return: float - time in second passed after invocation of function start_timer()
    """
    return time.time() - _start_time


def modbus_unsigned_to_signed(unsigned_data):
    """
    When using Modbus protocol, this is a command to convert 2 bytes unsigned data into signed data.

    :param unsigned_data: int - 2byte unsigned data(0~65535)
    :return: int - 2byte signed data(-32769 ~ 32767)
    """
    if type(unsigned_data) != int:
        raise DR_Error(DR_ERROR_TYPE, 'Invalid type : value{}'.format(type(unsigned_data)))
    if unsigned_data < 0 or unsigned_data > 65535:
        raise DR_Error(DR_ERROR_VALUE, 'Invalid value : value({0}) (Ranges: 0 ~ 65535)'.format(unsigned_data))
    if unsigned_data < 65536 and unsigned_data > 32767:
        return unsigned_data - 65536
    else:
        return unsigned_data


def serial_get_count():
    """
    This function reads the number of devices connected to USB to Serial.

    :return: number of devices connected to USB
    """
    return 0


def serial_get_info(id=1):
    """
    This function reads the port information and device name of the connected USB to Serial.

    :param id: ID of "USB to Serial" to read (1-10)
    :return: Port information (NULL means no device is connected), Device name (NULL means no device is connected)
    """
    return None

def set_force_factor(force_factor=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0], time=0.0):
    pass

def set_damping_factor(damping_factor=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0], time=0.0):
    pass


def wait_analog_input(ch, condition, val, timeout=None):
    pass

def wait_tool_analog_input(ch, condition, val, timeout=None):
    pass

def get_tool_analog_input(ch):
    pass

def set_mode_tool_analog_input(ch, mod):
    pass

def set_tool_digital_output_level(lv=None):
    pass

def set_tool_digital_output_type(port, output_type=None):
    pass

def set_output(port_type, index, val=None, time=None, val2=None):
    pass

def get_input(port_type, index):
    pass

def wait_input(port_type, index, val, timeout=None, condition=None):
    pass

def get_robot_flange_version():
    pass

def check_new_flange():
    pass

def set_comm_log(mode=0, option=0):
    pass

def check_robot_mastering():
    return False

def set_auto_acceleration_mode(mode=DR_ON, ratio=1.1):
    """
    Check motion command's input velocity, acceleration values before operating motion command. If allowable torque is exceeded, this function adjusts motion velocity, acceleration automatically. Input value mode sets activation or not. Input value ratio sets auto correction ratio.

    :param mode: DR_OFF(0): Deactivate mode, DR_ON(1) : Activate mode.
    :param ratio: Changing auto correction ratio within (0 ~ 1.2]
    :return: int - (0 -> Success, Negative value -> Error)
    """
    return 1


def get_safety_configuration():
    return 0


def get_safety_data_version():
    return 0


class joint_range:

    def __init__(self, obj):
        self.obj = obj
        self.max_vel = [
         0, 0, 0, 0, 0, 0]
        self.max_range = [0, 0, 0, 0, 0, 0]
        self.min_range = [0, 0, 0, 0, 0, 0]
        for i in range(0, 6):
            self.max_vel[i] = obj[0][i]
            self.max_range[i] = obj[1][i]
            self.min_range[i] = obj[2][i]


class config_joint_range:

    def __init__(self, obj):
        self.obj = obj
        self.normal = joint_range(obj[0])
        self.reduced = joint_range(obj[1])


def get_joint_range():
    return config_joint_range([[[360, 360, 360, 360, 360, 360], [360, 360, 360, 360, 360, 360], [-360, -360, -360, -360, -360, -360]],
                               [[360, 360, 360, 360, 360, 360], [360, 360, 360, 360, 360, 360], [-360, -360, -360, -360, -360, -360]]])


class general_range:

    def __init__(self, obj):
        self.obj = obj
        self.max_force = obj[0]
        self.max_power = obj[1]
        self.max_speed = obj[2]
        self.max_momentum = obj[3]


class config_general_range:

    def __init__(self, obj):
        self.obj = obj
        self.normal = general_range(obj[0])
        self.reduced = general_range(obj[1])


def get_general_range():
    return config_general_range([[180, 1000, 300, 500], [180, 1000, 300, 500]])


def get_collision_sensitivity():
    return 90


def get_safety_function():
    return None


class config_tool:

    def __init__(self, obj):
        self.obj = obj
        self.weight = obj[0]
        self.xyz = obj[1]
        self.inertia = obj[2]


class config_tool_symbol:

    def __init__(self, obj):
        self.obj = obj
        self.symbol = obj[0]
        self.tool = config_tool(obj[1])


def get_current_tool():
    tool_name = _get_active_tool().Name()
    return config_tool_symbol([tool_name, [0, [0, 0, 0], [0, 0, 0]]])


class config_tcp:

    def __init__(self, obj):
        self.obj = obj
        self.offset = obj


class config_tcp_symbol:

    def __init__(self, obj):
        self.obj = obj
        self.symbol = obj[0]
        self.tcp = config_tcp(obj[1])


def get_current_tcp():
    tcp_name = _get_active_tool().Name()
    return config_tcp_symbol([tcp_name, Pose_2_Comau(_get_active_tool().PoseTool())])


class config_install_pose:

    def __init__(self, obj):
        self.obj = obj
        self.gradient = obj[0]
        self.rotation = obj[1]


def get_install_pose():
    return config_install_pose([0,0])


class config_safety_io:

    def __init__(self, obj):
        self.obj = obj
        self.input = obj[0]
        self.output = obj[1]


def get_safety_io():
    return None


class point_2d:

    def __init__(self, obj):
        self.obj = obj
        self.x = obj[0]
        self.y = obj[1]


class line:

    def __init__(self, obj):
        self.obj = obj
        self.from_point = point_2d(obj[0])
        self.to_point = point_2d(obj[1])


class virtual_fence_object:

    def __init__(self, obj, type):
        self.obj = obj
        if type == 0:
            self.xlolimit = obj[0]
            self.xuplimit = obj[1]
            self.ylolimit = obj[2]
            self.yuplimit = obj[3]
            self.zlolimit = obj[4]
            self.zuplimit = obj[5]
        else:
            if type == 1:
                self.line_count = obj[0]
                self.line = list()
                for i in range(0, 6):
                    self.line.append(line(obj[1][i]))

                self.zlolimit = obj[2]
                self.zuplimit = obj[3]
            elif type == 2:
                self.radius = obj[0]
                self.zlolimit = obj[1]
                self.zuplimit = obj[2]


class config_virtual_fence:

    def __init__(self, obj):
        self.obj = obj
        self.target_ref = obj[0]
        self.fence_type = obj[1]
        self.fence_object = virtual_fence_object(obj[2], obj[1])


def get_virtual_fence():
    return None


class config_safe_zone:

    def __init__(self, obj):
        self.obj = obj
        self.target_ref = obj[0]
        self.line = list()
        for i in range(0, 2):
            self.line.append(line(obj[1][i]))

        self.point = list()
        for i in range(0, 3):
            self.point.append(point_2d(obj[2][i]))


def get_safety_zone():
    return None


class enable_safe_zone:

    def __init__(self, obj):
        self.obj = obj
        self.region = obj[0]


def get_enable_safety_zone():
    return None


class point_3d:

    def __init__(self, obj):
        self.obj = obj
        self.x = obj[0]
        self.y = obj[1]
        self.z = obj[2]


class safety_object_sphere:

    def __init__(self, obj):
        self.obj = obj
        self.radius = obj[0]
        self.target_pos = point_3d(obj[1])


class safety_object_capsule:

    def __init__(self, obj):
        self.obj = obj
        self.radius = obj[0]
        self.target_pos = list()
        for i in range(0, 2):
            self.target_pos.append(point_3d(obj[1][i]))


class safety_object_cube:

    def __init__(self, obj):
        self.obj = obj
        self.target_pos = list()
        for i in range(0, 2):
            self.target_pos.append(point_3d(obj[i]))


class safety_object_obb:

    def __init__(self, obj):
        self.obj = obj
        self.target_pos = list()
        for i in range(0, 4):
            self.target_pos.append(point_3d(obj[0][i]))


class safety_object_polyprism:

    def __init__(self, obj):
        self.obj = obj
        self.point_count = obj[0]
        self.point = list()
        for i in range(0, 10):
            self.target_pos.append(point_2d(obj[1][i]))

        self.zlolimit = obj[2]
        self.zuplimit = obj[3]


class safety_object_data:

    def __init__(self, obj, type):
        self.obj = obj
        if type == 0:
            self.sphere = safety_object_sphere(obj)
        else:
            if type == 1:
                self.capsule = safety_object_capsule(obj)
            else:
                if type == 2:
                    self.cube = safety_object_cube(obj)
                else:
                    if type == 3:
                        self.obb = safety_object_obb(obj)
                    elif type == 4:
                        self.polyprism = safety_object_polyprism(obj)


class safety_object:

    def __init__(self, obj):
        self.obj = obj
        self.target_ref = obj[0]
        self.object_type = obj[1]
        self.object = safety_object_data(obj[2], obj[1])


class config_protected_zone:

    def __init__(self, obj):
        self.obj = obj
        self.validity = obj[0]
        self.zone = list()
        for i in range(0, 10):
            self.zone.append(safety_object(obj[1][i]))


def get_protected_zone():
    return None


class config_collision_mute_zone_property:

    def __init__(self, obj):
        self.obj = obj
        self.id = obj[0]
        self.onoff = obj[1]
        self.safety_io = obj[2]
        self.sensitivity = obj[3]
        self.zone = safety_object(obj[4])


class config_collision_mute_zone:

    def __init__(self, obj):
        self.obj = obj
        self.validity = obj[0]
        self.property = list()
        for i in range(0, 10):
            self.property.append(config_collision_mute_zone_property(obj[1][i]))


def get_collision_mute_zone():
    return None


class safety_tool_orientation_limit:

    def __init__(self, obj):
        self.obj = obj
        self.target_dir = point_3d(obj[0])
        self.target_ang = obj[1]


class config_tool_orientation_limit_zone:

    def __init__(self, obj):
        self.obj = obj
        self.validity = obj[0]
        self.zone = list()
        for i in range(0, 10):
            self.zone.append(safety_object(obj[1][i]))

        self.limit = list()
        for i in range(0, 10):
            self.limit.append(safety_tool_orientation_limit(obj[2][i]))


def get_tool_orientation_limit_zone():
    return None


class config_tool_shape:

    def __init__(self, obj):
        self.obj = obj
        self.validity = obj[0]
        self.shape = list()
        for i in range(0, 5):
            self.shape.append(safety_object(obj[1][i]))


def get_current_tool_shape():
    return None


class config_nudge:

    def __init__(self, obj):
        self.obj = obj
        self.enable = obj[0]
        self.input_force = obj[1]
        self.delay_time = obj[2]


def get_nudge():
    return None


class config_cockpit:

    def __init__(self, obj):
        self.obj = obj
        self.enable = obj[0]
        self.button = obj[1]
        self.recovery_teach = obj[2]


def get_cockpit():
    return None


class config_idle_off:

    def __init__(self, obj):
        self.obj = obj
        self.func_enable = obj[0]
        self.elapse_time = obj[1]


def get_idle_off():
    return None


class config_tcp_list:

    def __init__(self, obj):
        self.obj = obj
        self.count = obj[0]
        self.tcp_list = list()
        for i in range(0, self.count):
            self.tcp_list.append(config_tcp_symbol(obj[1][i]))


def get_tcp_list():
    tools = _robodk_plugin_RDK.ItemList(filter=ITEM_TYPE_TOOL, list_names=False)
    names = []
    for t in tools:
        names.append(t.Name())
    return config_tcp_list([len(names), names])


class config_tool_list:

    def __init__(self, obj):
        self.obj = obj
        self.count = obj[0]
        self.tool_list = list()
        for i in range(0, self.count):
            self.tool_list.append(config_tool_symbol(obj[1][i]))


def get_tool_list():
    tools = _robodk_plugin_RDK.ItemList(filter=ITEM_TYPE_TOOL, list_names=False)
    names = []
    for t in tools:
        names.append(t.Name())
    return config_tool_list([len(names), names])


class config_tool_shape_symbol:

    def __init__(self, obj):
        self.obj = obj
        self.symbol = obj[0]
        self.tool_shape = config_tool_shape(obj[1])


class config_tool_shape_list:

    def __init__(self, obj):
        self.obj = obj
        self.count = obj[0]
        self.tool_shape_list = list()
        for i in range(0, self.count):
            self.tool_shape_list.append(config_tool_shape_symbol(obj[1][i]))


def get_tool_shape_list():
    return config_tool_shape_list([0, []])


def get_tool_symbol():
    return _get_active_tool().Name()

def get_tcp_symbol():
    return _get_active_tool().Name()

def get_tool_shape_symbol():
    return _get_active_tool().Name()

class write_modbus_tcp_data:

    def __init__(self, obj):
        self.obj = obj
        self.symbol = obj[0]
        self.ip = obj[1]
        self.port = obj[2]
        self.slave_id = obj[3]
        self.reg_type = obj[4]
        self.reg_index = obj[5]
        self.reg_value = obj[6]


class write_modbus_rtu_data:

    def __init__(self, obj):
        self.obj = obj
        self.symbol = obj[0]
        self.port = obj[1]
        self.slave_id = obj[2]
        self.baudrate = obj[3]
        self.byte_size = obj[4]
        self.parity = chr(obj[5])
        self.stop_bit = obj[6]
        self.reg_type = obj[7]
        self.reg_index = obj[8]
        self.reg_value = obj[9]


class modbus_data:

    def __init__(self, obj):
        self.obj = obj
        self.type = obj[0]
        if self.type == 0:
            self.tcp = write_modbus_tcp_data(obj[1])
        elif self.type == 1:
            self.rtu = write_modbus_rtu_data(obj[1])


class modbus_data_list:

    def __init__(self, obj):
        self.obj = obj
        self.count = obj[0]
        self.reg = list()
        for i in range(0, self.count):
            self.reg.append(modbus_data(obj[1][i]))


def get_modbus_data_list():
    return None


class config_world_coordinate:

    def __init__(self, obj):
        self.obj = obj
        self.type = obj[0]
        self.pos = obj[1]


def get_world_coord():
    return config_world_coordinate([0, get_user_cart_coord(DR_WORLD)[0]])


def get_io_speed_ratio():
    return None


def get_safety_zone_cnt():
    return 0


class local_zone_property_joint_range:

    def __init__(self, obj):
        self.obj = obj
        self.override = obj[0]
        self.min_range = obj[1]
        self.max_range = obj[2]


class local_zone_property_joint_speed:

    def __init__(self, obj):
        self.obj = obj
        self.override = obj[0]
        self.speed = obj[1]


class local_zone_property_tcp_force:

    def __init__(self, obj):
        self.obj = obj
        self.override = obj[0]
        self.force = obj[1]


class local_zone_property_tcp_power:

    def __init__(self, obj):
        self.obj = obj
        self.override = obj[0]
        self.power = obj[1]


class local_zone_property_tcp_speed:

    def __init__(self, obj):
        self.obj = obj
        self.override = obj[0]
        self.speed = obj[1]


class local_zone_property_tcp_momentum:

    def __init__(self, obj):
        self.obj = obj
        self.override = obj[0]
        self.momentum = obj[1]


class local_zone_property_collision:

    def __init__(self, obj):
        self.obj = obj
        self.override = obj[0]
        self.sensitivity = obj[1]


class local_zone_property_speed_rate:

    def __init__(self, obj):
        self.obj = obj
        self.override = obj[0]
        self.speed_rate = obj[1]


class local_zone_property_collision_stopmode:

    def __init__(self, obj):
        self.obj = obj
        self.override = obj[0]
        self.stop_mode = obj[1]


class local_zone_property_tcpslf_stopmode:

    def __init__(self, obj):
        self.obj = obj
        self.override = obj[0]
        self.stop_mode = obj[1]


class local_zone_property_tool_orientation:

    def __init__(self, obj):
        self.obj = obj
        self.override = obj[0]
        self.direction = obj[1]
        self.angle = obj[2]


class safety_zone_property_space_limit:

    def __init__(self, obj):
        self.obj = obj
        self.inspection_type = obj[0]
        self.joint_range_override = local_zone_property_joint_range(obj[1])
        self.dynamic_zone_enable = obj[2]
        self.inside_zone_detection = obj[3]


class safety_zone_property_local_zone:

    def __init__(self, obj):
        self.obj = obj
        self.joint_range_override = local_zone_property_joint_range(obj[0])
        self.joint_speed_override = local_zone_property_joint_speed(obj[1])
        self.tcp_force_override = local_zone_property_tcp_force(obj[2])
        self.tcp_power_override = local_zone_property_tcp_power(obj[3])
        self.tcp_speed_override = local_zone_property_tcp_speed(obj[4])
        self.tcp_momentum_override = local_zone_property_tcp_momentum(obj[5])
        self.collision_override = local_zone_property_collision(obj[6])
        self.speed_rate = local_zone_property_speed_rate(obj[7])
        self.collision_violation_stop_mode_override = local_zone_property_collision_stopmode(obj[8])
        self.force_violation_stop_mode_override = local_zone_property_tcpslf_stopmode(obj[9])
        self.tool_orientation_limit_override = local_zone_property_tool_orientation(obj[10])
        self.dynamic_zone_enable = obj[11]
        self.led_override = obj[12]
        self.nudge_enable = obj[13]
        self.allow_ress_safe_work = obj[14]
        self.override_reduce = obj[15]
        self.inside_zone_detection = obj[16]
        self.collaborative_zone = obj[17]


class safety_zone_property_data:

    def __init__(self, obj, type):
        self.obj = obj
        if type == 0:
            self.space_limit_zone = safety_zone_property_space_limit(obj[0])
        elif type == 1:
            self.local_zone = safety_zone_property_local_zone(obj[0])


class safety_zone_shape_sphere:

    def __init__(self, obj):
        self.obj = obj
        self.center = point_3d(obj[0])
        self.radius = obj[1]


class safety_zone_shape_cylinder:

    def __init__(self, obj):
        self.obj = obj
        self.center = point_2d(obj[0])
        self.radius = obj[1]
        self.zlolimit = obj[2]
        self.zuplimit = obj[3]


class safety_zone_shape_cuboid:

    def __init__(self, obj):
        self.obj = obj
        self.xlolimit = obj[0]
        self.xuplimit = obj[1]
        self.ylolimit = obj[2]
        self.yuplimit = obj[3]
        self.zlolimit = obj[4]
        self.zuplimit = obj[5]


class safety_zone_shape_tilted_cuboid:

    def __init__(self, obj):
        self.obj = obj
        self.origin = point_3d(obj[0])
        self.u = point_3d(obj[1])
        self.v = point_3d(obj[2])
        self.w = point_3d(obj[3])


class safety_zone_shape_multi_plane:

    def __init__(self, obj):
        self.obj = obj
        self.valid_plane = obj[0]
        self.plane = list()
        for i in range(0, 6):
            self.plane.append(line(obj[1][i]))

        self.zlolimit = obj[2]
        self.zuplimit = obj[3]
        self.space_point = point_2d(obj[4])


class safety_zone_shape_capsule:

    def __init__(self, obj):
        self.obj = obj
        self.center1 = point_3d(obj[0])
        self.center2 = point_3d(obj[1])
        self.radius = obj[2]


class safety_zone_shape_data:

    def __init__(self, obj, type):
        self.obj = obj
        if type == 0:
            self.sphere = safety_zone_shape_sphere(obj[0])
        else:
            if type == 1:
                self.cylinder = safety_zone_shape_cylinder(obj[0])
            else:
                if type == 2:
                    self.cuboid = safety_zone_shape_cuboid(obj[0])
                else:
                    if type == 3:
                        self.obb = safety_zone_shape_tilted_cuboid(obj[0])
                    else:
                        if type == 4:
                            self.multiplane = safety_zone_shape_multi_plane(obj[0])
                        elif type == 5:
                            self.capsule = safety_zone_shape_capsule(obj[0])


class safety_zone_shape:

    def __init__(self, obj):
        self.obj = obj[0]
        self.coordinate = obj[0][0]
        self.shape_type = obj[0][1]
        self.shape_data = safety_zone_shape_data(obj[0][2], obj[0][1])
        self.margin = obj[0][3]
        self.valid_space = obj[0][4]


class config_safety_zone:

    def __init__(self, obj):
        self.obj = obj
        self.id = obj[0]
        self.alias = obj[1]
        self.type = obj[2]
        self.property = safety_zone_property_data(obj[3], obj[2])
        self.shape = safety_zone_shape(obj[4])


def get_safety_zone_list():
    return []


def get_user_coord_cnt():
    return []


class config_user_coordinate:

    def __init__(self, obj):
        self.obj = obj
        self.target_ref = obj[0]
        self.target_pos = obj[1]
        self.user_id = obj[2]


class config_configurable_io:

    def __init__(self, obj):
        self.obj = obj
        self.input = obj[0]
        self.output = obj[1]


def get_user_coord():
    return None


def get_configurable_io():
    return None


def goto(lineno):
    global _g_drl_called_from
    global _g_drl_line_goto_flag
    global _g_drl_target_lineno
    _g_drl_target_lineno = lineno
    frame = sys._getframe().f_back
    _g_drl_called_from = frame
    while frame:
        frame.f_trace = traceit
        frame = frame.f_back

    _g_drl_line_goto_flag = True


def get_current_state():
    return None


def get_current_system():
    return None


def check_homing_done():
    return True



