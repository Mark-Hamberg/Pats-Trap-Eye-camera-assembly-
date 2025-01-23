from DRCF import *
import socket

# Robot connection details
HOST = "192.168.137.75"
PORT = 12345

# Position coordinates
firstpos = posj(-14.4, 0.5, -111.9, -3.0, -64.7, 161.8)
firstpcb = posx(-127.9, 374.3, 300.7, 75.0, 164.1, -106.7)
grabpcb = posx(-127.9, 375.3, 266.7, 75.0, 164.1, -106.7)
grabpcb2 = posx(-127.9, 377.3, 266.7, 75.0, 164.1, -106.7)
pcbholder = posx(-444.7, 255.1, 310, 179.7, 178.9, -91.1)
pcbplace1 = posx(-449.7, 256.7, 210, 145.6, -178.9, -125.8)
pcbplace2 = posx(-449.7, 256.7, 167, 145.6, -178.9, -125.8)
middle = posx(-389.2, -154.1, 278.5, 118.7, 177.9, -155.6)
middle2 = posx(-372.2, -441.0, 249.9, 90.1, -94.0, 177.8)
camera = posx(-363.2, -659.3,240, 89.6, -90.0, -180.0)
cameratray = posx(-363.2, -659.3, 177.0, 89.6, -90.0, -180.0)
cameraalign = posx(-467.0, -271.1, 199.4, 2.9, -90.0, -180.0)
cameraalign2 = posx(-467.0, -271.1, 164.8, 2.9, -90.0, 180.0)
firstglue = posx(-557.0, -113.3, 249.4, 0.2, -89.5, -65.3)
glue_station = posx(-552.4, -325.5, 207.4, 0.7, -89.5, -48.0)
middle3 = posx(-677.5, 168.3, 318.0, 179.6, 91.7, -2.5)
success_box = posx(-495.1, 621.2, 463.0, 120.2, 176.8, -150.7)
reject_box = posx(-470.1, 825.8, 525.0, 159.3, 178.1, -113.5)
set_mode_analog_output(1,DR_ANALOG_VOLTAGE) #output
set_mode_analog_input(1, DR_ANALOG_VOLTAGE) #input
PcbDetect = get_analog_input(1)
tp_log(str(PcbDetect))


# Function to start robot task
def start_robot_task():
    # Set digital outputs to start the process
    set_digital_output(1, ON)
    set_digital_output(1, OFF)
    set_digital_output(2, OFF)
    set_analog_output(1,0.5)
    # set_analog_output(1,2.4)
    set_analog_output(1,3.4)
    wait(2)

    # Move to the first position
    movej(firstpos, v=50, a=50)
    # while PcbDetect <= 0.5
    movel(firstpcb, v=100, a=100)

    # Grab the PCB
    movel(grabpcb, v=100, a=100)
    movel(grabpcb2, v=100, a=100)
    wait(1)
    set_digital_output(1, OFF)
    set_digital_output(2, ON)
    wait(1)
    movel(firstpcb, v=400, a=900)

    # Place PCB on the holder
    movel(pcbholder, v=100, a=100)
    movel(pcbplace1, v=100, a=100)
    movel(pcbplace2, v=100, a=100)
    wait(1)
    set_digital_output(2,OFF)
    set_digital_output(1,ON)
    # set_digital_output(1,OFF)
    wait(1)
    movel(pcbholder, v=100, a=100)
    set_analog_output(1,2.9)
    wait(16)
    set_analog_output(1,3.4)
    wait(2)
    set_analog_output(1,2.9)
    tp_log("love")
    #
    #
    # # while True:
    #     set_analog_output(1,0.9)
    movel(middle, v=50, a=50)
    movel(middle2, v=50, a=50)

    # Camera grab and alignment
    movel(camera, v=50, a=50)
    set_digital_output(5, ON)
    wait(1)
    movel(cameratray, v=50, a=50)
    movel(camera, v=50, a=50)
    wait(1)
    movel(cameraalign, v=30, a=30)
    movel(cameraalign2, v=30, a=30)
    set_analog_output(1, 1)
    wait(3)
    set_analog_output(1, 0)
    tp_log("bump")

    # movel(firstglue, v=50, a=50)
    # movel(glue_station, v=50, a=50)
    # movel(middle3, v=50, a=50)
        #
        # # Camera Placement
        # movel(posx(-371.3, 184.9, 267.9, 177.3, 90.7, 1.2), v=50, a=50)
        # movel(posx(-379.0, 189.5, 206.8, 1.6, -90.4, -179.4), v=30, a=30)
        # movel(posx(-379.2, 189.8, 191.4, 1.8, -90.7, -179.0), v=10, a=10)
        # movel(posx(-380.6, 189.9, 189.2, 1.8, -90.9, -178.9), v=10, a=10)
        # movel(posx(-381.7, 190.1, 188.2, 1.9, -91.0, -178.8), v=10, a=10)
        # movel(posx(-379.2, 190.4, 188.4, 1.7, -91.1, -178.8), v=10, a=10)
        # wait(2)
        # set_digital_output(5, OFF)
        # movel(posx(-373.3, 190.2, 208.6, 0.2, -87.9, -178.6), v=10, a=10)

        # Grab PCB on the holder
        # set_digital_output(1, ON)
        # set_digital_output(2, OFF)
        # movel(pcbholder, v=10, a=10)
        # movel(pcbplace1, v=10, a=10)
        # movel(pcbplace2, v=10, a=10)
        # wait(2)
        # set_digital_output(1, OFF)
        # set_digital_output(2, ON)
        # movel(pcbplace1, v=10, a=10)
        # movej(pcbplace3, v=10, a=10)
        # movej(pcbplace4, v=10, a=10)
        # set_digital_output(1, ON)
        # set_digital_output(2, OFF)
        # wait(5)
        # movej(pcbplace3, v=10, a=10)

    # Notify PC to start camera detection
    tp_log("Reached glue station. Sending command to start camera detection...")
    client_socket_write(socket, b"START DETECTION")
    tp_log("Command 'START DETECTION' sent to PC.")


# Wait for 'START ROBOT' command from PC
tp_log("Opening socket to wait for 'START ROBOT' command...")
socket = client_socket_open(HOST, PORT)
if socket:
    tp_log("Waiting for 'START ROBOT' command...")
    response, data = client_socket_read(socket, length=1000, timeout=240)
    if data:
        tp_log("Received data: " + str(data))
        if response > 0 and data.strip() == b"START ROBOT":
            tp_log("'START ROBOT' command received. Starting robot task...")
            start_robot_task()

            # Notify PC that the robot is ready for camera detection
            client_socket_write(socket, b"GOT IT")
            tp_log("Sent 'GOT IT' to PC.")
        else:
            tp_log("No valid 'START ROBOT' command received.")
    else:
        tp_log("No data received.")
    # client_socket_close(socket)
else:
    tp_log("Error: Unable to open socket to wait for command.")

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
            movel(posx(-495.1, 621.2, 463.0, 120.2, 176.8, -150.7), v=150, a=150)
            movel(posx(-503.2, 628.4, 223.6, 11.9, 179.6, 100.8), v=150, a=150)
        else:
            tp_log("Reject condition triggered.")
            movel(posx(-470.1, 825.8, 525.0, 159.3, 178.1, -113.5), v=150, a=150)
            movel(posx(-495.5, 878.4, 228.2, 1.4, 177.9, 88.0), v=150, a=150)

    except Exception as e:
        tp_log("Error reading detection result: " + str(e))
else:
    tp_log("Failed to connect to PC.")
