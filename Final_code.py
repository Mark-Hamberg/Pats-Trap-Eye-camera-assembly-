from DRCF import *
import socket

# Robot connection details
HOST = "192.168.137.75"
PORT = 12345

# Position coordinates
firstpos = posj(-14.4, 0.5, -111.9, -3.0, -64.7, 161.8)

abovepcbslide = posx(-113.9, 370, 320, 97.4, 162.9, -82.2)
grabpcb = posx(-120.9, 372.5, 263.0, 90.1, 164.3, -90.8)
grabpcb2 = posx(-120.9, 374.5, 263.0, 90.1, 164.3, -90.8)

abovepcbholder = posx(-444.9, 255.5, 400, 153.3, -180.0, -117.4)
pcbplace1 = posx(-446.1, 253.1, 190, 76.2, -179.9, 164.3)
pcbplace2 = posx(-446.1, 253.1, 168, 76.2, -179.9, 164.3)

abovepcbholder2 = posx(-437.5, 411.1, 250, 149.5, 176.2, -122.8)
pcbplace3 = posx(-446.1, 417, 190, 76.2, -179.9, 164.3)
pcbplace4 = posx(-446.1, 417, 168, 76.2, -179.9, 164.3)

tussenposcamera = posj(44.7, -20.0, -130.2, 130.7, -69.5, 22.1)
abovecamera = posx(-360.5, -657.7, 240, 91.3, -89.9, 178.4)
grabcameratray = posx(-360.5, -657, 176, 91.3, -89.9, 178.4)
abovealignmold = posx(-477.4, -259.6, 200, 90.0, -90.4, -179.8)
alignmold = posx(-477.4, -259.6, 180.0, 90.0, -90.4, -179.8)
grabcameramold = posx(-477.4, -259.6, 173, 90.0, -90.4, -179.8)

onmold = posx(-481.8, -237.3, 177, 90.5, -90.0, 180.0)

turntoglue = posx(-440.8, -130.1, 216.8, 101.4, -112.1, 47.4)
togluestation = posx(-570.5, -119.6, 260.2, 88.7, -132.2, 17.4)
gluedrop = posx(-587.5, -128.9, 229.7, 89.1, -120.7, 12.2)
removestrings = posx(-606.2, -113.9, 253.1, 87.1, -130.5, 1.9)
removestrings2 = posx(-608, -22.3, 329, 87.0, -130.5, 1.7)

afterglue = posx(-600.9, -15, 330, 87.0, -130.5, 1.7)
tussenlatch = posj(0, 0, -120, 180, -30, 0)

cam1 = posx(-350.8, 186, 200, 0.26, -79.95, -179.93)
cam2 = posx(-352.6, 186, 177, 0.26, -79.95, -179.93)
cam3 = posx(-347, 186, 175.5, 0.26, -79.95, -179.93)
#cam3 = posx(-347, 185.7, 176.0, 0.26, -79.95, -179.93)
cam4 = posx(-371, 186, 187.9, 0.5, -90.0, -180.0)
cam5 = posx(-371.8, 186, 210, 0.5, -90.0, -180.0)

tussenshaky = posx(-366, 186.5, 440, 0, -87.9, 180)
toshaky = posx(-446.1, 253.1, 282, 76.2, -180, 164.3)
sideshake = posx(-446.1, 253.1, 282.5, 179.9, -90, -90)
sideshake2 = posx(-553.7, 305.4, 384.5, 154.2, 98.8, -117.5)

middle3 = posx(-677.5, 168.3, 318.0, 179.6, 91.7, -2.5)
success_box = posx(-495.1, 621.2, 463.0, 120.2, 176.8, -150.7)
reject_box = posx(-470.1, 825.8, 525.0, 159.3, 178.1, -113.5)

set_mode_analog_output(1, DR_ANALOG_VOLTAGE)  # output
set_mode_analog_input(1, DR_ANALOG_VOLTAGE)  # input

speed = 80
accel = 80
cameraCount = 0
pcbdetect = get_analog_input(1)


# Function to start robot task
def start_robot_task():
    # Set digital outputs to start the process
    set_digital_output(1, ON)  # PCB gripper open
    set_digital_output(1, OFF)
    set_digital_output(2, OFF)
    set_analog_output(1, 0.7)  # Camera gripper open
    wait(1)

    pcbdetect = get_analog_input(1)
    wait(2)

    # Move to the first position
    movej(firstpos, speed, accel)


    # Move to the PCB slide
    movel(abovepcbslide, speed, accel)

    # Grab the PCB
    movel(grabpcb, v=50, a=50)
    movel(grabpcb2, v=50, a=50)
    wait(1)
    set_digital_output(1, OFF)
    set_digital_output(2, ON)
    wait(2)
    movel(abovepcbslide, v=500, a=900)

    # Place PCB in the holder
    movel(abovepcbholder, speed, accel)
    movel(pcbplace1, speed, accel)
    movel(pcbplace2, v =50, a=50)
    wait(1)
    set_digital_output(2, OFF)
    set_digital_output(1, ON)
    wait(1)
    movel(abovepcbholder, speed, accel)
    set_analog_output(1, 3.3)
    wait(17.2)
    set_analog_output(1, 3.8)
    wait(2)
    set_analog_output(1, 3.3)
    wait(2)
    set_analog_output(1, 3.8)
    set_digital_output(1, OFF)

    # Move to the camera's
    movej(tussenposcamera, v=50, a=50)

    # Grab the camera
    abovecamera = posx(-360.5 + cameraCount * 19.5, -657.7 - cameraCount * 1, 240, 91.3, -89.9, 178.4)
    grabcameratray = posx(-360.5 + cameraCount * 19.5, -657.7 - cameraCount * 1, 178.7, 91.3, -89.9, 178.4)

    set_analog_output(1, 0.7)
    movel(abovecamera, speed, accel)
    set_digital_output(5, ON)  # Turn on air for suction cup
    movel(grabcameratray, v=50, a=50)
    wait(1)
    movel(abovecamera, speed, accel)
    wait(1)

    # Move to the alignmold and align
    set_analog_output(1, 0.7)
    movel(abovealignmold, speed, accel)
    movel(alignmold, v=50, a=50)
    set_digital_output(5, OFF)
    wait(1)
    movel(grabcameramold, v=50, a=50)
    set_digital_output(5, ON)
    wait(1)
    movel(abovealignmold, speed, accel)
    movel(onmold, v=50, a=50)
    set_analog_output(1, 1.3)  # Close gripper
    wait(2)
    set_analog_output(1, 0)
    wait(5)

    # Move to gluestation and get a drop of glue on the camera
    movel(turntoglue, speed, accel)
    movel(togluestation, v=30, a=30)
    movel(gluedrop, v=30, a=30)
    wait(5)
    movel(removestrings, v=30, a=30)
    movel(removestrings2, speed, accel)

    movel(afterglue, speed, accel)
    movej(tussenlatch, v=50, a=50)

    # Camera placement
    movel(cam1, speed, accel)
    movel(cam2, v=10, a=10)
    movel(cam3, v=10, a=10)
    movel(cam4, v=10, a=10)
    set_digital_output(5, OFF)
    set_analog_output(1, 1.7)
    wait(2)
    movel(cam5, speed, accel)
    set_analog_output(1, 2.3)
    wait(15)
    set_analog_output(1, 2.8)

    # Do Shaky things
    movel(tussenshaky, speed, accel)

    movel(abovepcbholder, speed, accel)
    movel(pcbplace1, speed, accel)
    movel(pcbplace2, v=50, a=50)
    set_digital_output(2, ON)
    wait(2)
    movel(toshaky, speed, accel)
    movel(sideshake, speed, accel)
    movel(toshaky, speed, accel)
    movel(sideshake2, speed, accel)
    movel(toshaky, speed, accel)

    movel(abovepcbholder2, speed, accel)
    movel(pcbplace3, speed, accel)
    movel(pcbplace4, v=30, a=30)
    set_digital_output(2, OFF)
    set_digital_output(1, ON)
    movel(abovepcbholder2, speed, accel)
    wait(10)


# Notify PC to start camera detection
# tp_log("Reached glue station. Sending command to start camera detection...")
# client_socket_write(socket, b"START DETECTION")
# tp_log("Command 'START DETECTION' sent to PC.")

# set_analog_output(1, 2.8)  # Retract actuator 1
# wait(15)
# set_analog_output(1, 3.8)  # Retract actuator 2
# wait(15)
set_analog_output(1,0.6)

tp_log("Opening socket to wait for 'START ROBOT' command...")
socket = client_socket_open(HOST, PORT)

if socket:
    tp_log("Waiting for 'START ROBOT' command...")
    response, data = client_socket_read(socket, length=1000, timeout=240)
    if data:
        tp_log("Received data: " + str(data))
        if response > 0 and data.strip() == b"START ROBOT":
            tp_log("'START ROBOT' command received. Starting robot task...")
            while pcbdetect >= 0.6:  # No PCB detected
                tp_log("No PCB detected!")
                wait(1)
                pcbdetect = get_analog_input(1)

            while pcbdetect <= 0.5:  # PCB detected
                # Wait for 'START ROBOT' command from PC
                start_robot_task()
                # Notify PC that the robot is ready for camera detection
                client_socket_write(socket, b"GOT IT")
                tp_log("Sent 'GOT IT' to PC.")

                # Wait for the detection result from PC
                tp_log("Connecting to PC...")
                # socket = client_socket_open(HOST, PORT)
                if socket:
                    tp_log("Connected to PC. Waiting for detection result...")
                    try:
                        response, data = client_socket_read(socket, length=1000, timeout=1800)  # Read the message
                        decoded_string = data.decode('utf-8')
                        received_number = int(decoded_string)
                        tp_log("Raw response: " + str(response))  # Log the response value
                        tp_log("Raw data received: " + str(data))  # Log the raw data

                        if received_number == 1:
                            tp_log("Success condition triggered.")
                            movel(pcbplace4, v=50, a=50)
                            set_digital_output(1, OFF)
                            set_digital_output(2, ON)
                            wait(1)
                            movel(abovepcbholder2, speed, accel)
                            movel(posx(-495.1, 621.2, 463.0, 120.2, 176.8, -150.7), speed, accel)
                            movel(posx(-503.2, 628.4, 223.6, 11.9, 179.6, 100.8), speed, accel)
                            set_digital_output(2, OFF)
                            set_digital_output(1, ON)
                            movel(posx(-470.1, 825.8, 400.0, 159.3, 178.1, -113.5), speed, accel)
                            cameraCount = +1
                            pcbdetect = get_analog_input(1)

                        else:
                            movel(pcbplace4, v=50, a=50)
                            set_digital_output(1, OFF)
                            set_digital_output(2, ON)
                            wait(1)
                            movel(abovepcbholder2, speed, accel)
                            movel(posx(-470.1, 825.8, 400.0, 159.3, 178.1, -113.5), speed, accel)
                            movel(posx(-495.5, 878.4, 228.2, 1.4, 177.9, 88.0), speed, accel)
                            set_digital_output(2, OFF)
                            set_digital_output(1, ON)
                            movel(posx(-470.1, 825.8, 400.0, 159.3, 178.1, -113.5), speed, accel)
                            cameraCount = +1
                            pcbdetect = get_analog_input(1)
                    except Exception as e:
                        tp_log("Error reading detection result: " + str(e))
                else:
                    tp_log("Failed to connect to PC.")
        else:
                tp_log("No valid 'START ROBOT' command received.")
    else:
        tp_log("No data received.")
    # client_socket_close(socket)
else:
    tp_log("Error: Unable to open socket to wait for command.")