from DRCF import *
import socket

# Robot connection details
HOST = "192.168.137.69"
PORT = 12345

# Position coordinates
firstpos = posj(-14.4, 0.5, -111.9, -3.0, -64.7, 161.8)

abovepcbslide = posx(-113.9, 370, 320, 97.4, 162.9, -82.2)
grabpcb = posx(-113.9, 368.9, 260, 97.4, 162.9, -82.2)
grabpcb2 = posx(-113.9, 370.7, 260, 97.4, 162.9, -82.2)

abovepcbholder = posx(-444.9, 255.5, 400, 153.3, -180.0, -117.4)
pcbplace1 = posx(-444.9, 255.5, 190, 153.3, -180.0, -117.4)
pcbplace2 = posx(-444.9, 255.5, 169.2, 153.3, -180.0, -117.4)

tussenposcamera = posx(-372.2, -441.0, 249.9, 90, -90, 180)
abovecamera = posx(-364.8, -659.3,240, 90, -90.0, -180.0)
grabcameratray = posx(-364.8, -659.3, 177.0, 90, -90.0, -180.0)
abovealignmold = posx(-477.2, -258, 279.8, 90, -90, -180)
alignmold = posx(-477.2, -258, 179.8, 90, -90, -180)
grabcameramold = posx(-478, -258, 172, 90.0, -90.0, -180.0)

onmold = posx(-481.8, -237.3, 177, 90.5, -90.0, 180.0)

turntoglue = posx(-440.8, -130.1, 216.8, 101.4, -112.1, 47.4)
togluestation = posx(-570.5, -119.6, 260.2, 88.7, -132.2, 17.4)
gluedrop = posx(-585.5, -128.9, 229.7, 89.1, -120.7, 12.2)
removestrings = posx(-605.2, -113.9, 253.1, 87.1, -130.5, 1.9)
removestrings2 = posx(-600.9, -22.3, 329.5, 87.0, -130.5, 1.7)

middle3 = posx(-677.5, 168.3, 318.0, 179.6, 91.7, -2.5)
success_box = posx(-495.1, 621.2, 463.0, 120.2, 176.8, -150.7)
reject_box = posx(-470.1, 825.8, 525.0, 159.3, 178.1, -113.5)

set_mode_analog_output(1,DR_ANALOG_VOLTAGE) #output
set_mode_analog_input(1, DR_ANALOG_VOLTAGE) #input

speed = 100
accel = 100

# Function to start robot task
def start_robot_task():

    # Set digital outputs to start the process
    set_digital_output(1, ON) # PCB gripper open
    set_digital_output(1, OFF)
    set_digital_output(2, OFF)
    set_analog_output(1,0.7) # Camera gripper open
    wait(1)
    #set_analog_output(1,2.8) # Retract actuator 1
    #wait(1)
    #set_analog_output(1,3.8) # Retract actuator 2
    #wait(1)

    pcbdetect = get_analog_input(1)
    wait(2)

    # Move to the first position
    movej(firstpos, speed, accel)

    while pcbdetect >= 0.6: # No PCB detected

        tp_log("No PCB detected!")
        pcbdetect = get_analog_input(1)
        wait(1)


    while pcbdetect <= 0.5: # PCB detected

        # Move to the PCB slide
        movel(abovepcbslide, speed, accel)

        # Grab the PCB
        movel(grabpcb, v=30, a=30)
        movel(grabpcb2, v=30, a=30)
        wait(1)
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(1)
        movel(abovepcbslide, v=400, a=900)

        # Place PCB in the holder
        movel(abovepcbholder, speed, accel)
        movel(pcbplace1, speed, accel)
        movel(pcbplace2, v=30, a=30)
        wait(1)
        set_digital_output(2,OFF)
        set_digital_output(1,ON)
        wait(1)
        movel(abovepcbholder, speed, accel)
        #set_analog_output(1,3.3)
        #wait(16)
        #set_analog_output(1,3.8)
        #wait(2)
        #set_analog_output(1,3.3)
        #wait(2)
        #set_analog_output(1, 3.8)
        #set_digital_output(1,OFF)

        # Move to the camera's
        movel(tussenposcamera, speed, accel)

        # Grab the camera
        movel(abovecamera, speed, accel)
        set_digital_output(5, ON) # Turn on air for suction cup
        wait(1)
        movel(grabcameratray, v=30, a=30)
        movel(abovecamera, speed, accel)
        wait(1)

        # Move to the alignmold and align
        movel(abovealignmold, speed, accel)
        movel(alignmold, v=30, a=30)
        set_digital_output(5, OFF)
        wait(1)
        movel(grabcameramold, v=30, a=30)
        set_digital_output(5, ON)
        wait(1)
        movel(abovealignmold, speed, accel)
        movel(onmold, speed, accel)
        set_analog_output(1, 1.3) # Close gripper
        wait(2)
        set_analog_output(1, 0)
        wait(5)

        # Move to gluestation and get a drop of glue on the camera
        movel(togluestation, speed, accel)
        movel(gluedrop, v=20, a=20)
        wait(1)
        movel(removestrings, v=50, a=50)
        movel(removestrings2, v=50, a=50)

        # set_analog_output(1,2.3) # Close latch

        # Look if there is a PCB
        pcbdetect = get_analog_input(1)

start_robot_task()




# Socket commands:


#         # Notify PC to start camera detection
#         tp_log("Reached glue station. Sending command to start camera detection...")
#         client_socket_write(socket, b"START DETECTION")
#         tp_log("Command 'START DETECTION' sent to PC.")
#
#
# # Wait for 'START ROBOT' command from PC
# tp_log("Opening socket to wait for 'START ROBOT' command...")
# socket = client_socket_open(HOST, PORT)
# if socket:
#     tp_log("Waiting for 'START ROBOT' command...")
#     response, data = client_socket_read(socket, length=1000, timeout=240)
#     if data:
#         tp_log("Received data: " + str(data))
#         if response > 0 and data.strip() == b"START ROBOT":
#             tp_log("'START ROBOT' command received. Starting robot task...")
#             start_robot_task()
#
#             # Notify PC that the robot is ready for camera detection
#             client_socket_write(socket, b"GOT IT")
#             tp_log("Sent 'GOT IT' to PC.")
#         else:
#             tp_log("No valid 'START ROBOT' command received.")
#     else:
#         tp_log("No data received.")
#     # client_socket_close(socket)
# else:
#     tp_log("Error: Unable to open socket to wait for command.")
#
# # Wait for the detection result from PC
# tp_log("Connecting to PC...")
# # socket = client_socket_open(HOST, PORT)
# if socket:
#     tp_log("Connected to PC. Waiting for detection result...")
#     try:
#         response, data = client_socket_read(socket, length=1000, timeout=1800)  # Read the message
#         decoded_string = data.decode('utf-8')
#         received_number = int(decoded_string)
#         tp_log("Raw response: " + str(response))  # Log the response value
#         tp_log("Raw data received: " + str(data))  # Log the raw data
#
#         if received_number == 1:
#             tp_log("Success condition triggered.")
#             movel(posx(-495.1, 621.2, 463.0, 120.2, 176.8, -150.7), v=150, a=150)
#             movel(posx(-503.2, 628.4, 223.6, 11.9, 179.6, 100.8), v=150, a=150)
#         else:
#             tp_log("Reject condition triggered.")
#             movel(posx(-470.1, 825.8, 525.0, 159.3, 178.1, -113.5), v=150, a=150)
#             movel(posx(-495.5, 878.4, 228.2, 1.4, 177.9, 88.0), v=150, a=150)
#
#     except Exception as e:
#         tp_log("Error reading detection result: " + str(e))
# else:
#     tp_log("Failed to connect to PC.")
