import tkinter as tk
from tkinter import ttk
import cv2
from PIL import Image, ImageTk
import threading
import queue
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO
import socket
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Initialize counters
success_count = 0
reject_count = 0
total_tested = 0  # Counter for total tested PCBs

# Load YOLO model
model = YOLO(r'E:\PATS\Train5\Love.pt')

# Global socket connection
robot_socket = None

# Function to start the server
# Function to start the server
def start_server():
    global robot_socket
    HOST = '0.0.0.0'  # Allow connections from any IP
    PORT = 12345      # Port to listen on

    # Start a socket server
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print("Server is starting...")
        s.bind((HOST, PORT))
        s.listen()
        print("Waiting for the robot to connect...")
        conn, addr = s.accept()  # Accept the connection
        robot_socket = conn
        print(f"Connected by {addr}")
        listen_for_robot()


# Function to listen for messages from the robot
def listen_for_robot():
    global robot_socket
    while True:
        try:
            # Receive data from the robot
            data = robot_socket.recv(1024).decode()
            
            # Handle disconnection
            if not data:
                print("Robot disconnected.")
                break
            
            # Process received data
            if data == "GOT IT":
                print("Received 'GOT IT' from the robot. Starting quality control...")
                quality_control_process()
            # elif data == "START DETECTION":
            #     print("Received 'START DETECTION' from the robot. Performing camera detection...")
            #     quality_control_process()
        except ConnectionResetError:
            print("Robot connection reset.")
            break
        except Exception as e:
            print(f"Error receiving data: {e}")
            break
    # Close the socket when the loop ends
    # if robot_socket:
    #     robot_socket.close()
    #     robot_socket = None
 
# Function to send a message to the robot
# def send_to_robot(message):
#     global robot_socket
#     if robot_socket:
#         try:
#             robot_socket.sendall(str(message).encode())  # Send data
#             print(f"Sent to robot: {message}")
#         except Exception as e:
#             print(f"Error sending data: {e}")
#         # Do NOT close the socket immediately after sending

def send_to_robot(message):
    global robot_socket
    if robot_socket:
        try:
            robot_socket.sendall(str(message).encode())  # Send result
            print(f"Sent to robot: {message}")
            
            # Wait briefly to ensure the robot processes the message before the socket closes
            import time
            time.sleep(3)  # Add a delay if needed
        except Exception as e:
            print(f"Error sending data: {e}")

# Function to start the process when the Start button is clicked
def start_action():
    status_label.config(text="Status: Sending 'START ROBOT'...")
    send_to_robot("START ROBOT")

# Function to perform quality control
def quality_control_process():
    global total_tested
    result, result_img = quality_control()  # Call quality control function
    display_result_image(result_img)
    total_tested += 1  # Increment total tested count
    update_total_tested()  # Update total tested display

    if result == 1:
        update_counters("success")
    else:
        update_counters("reject")
    
    send_to_robot(result)  # Send 0 or 1 to the robot

# Function to perform quality control with RealSense and YOLO
def quality_control():
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline = rs.pipeline()
    pipeline.start(config)

    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())
    img = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    # wb = cv2.xphoto.createSimpleWB()
    # img = wb.balanceWhite(color_correct)
    

    cropped_img = img[200:350, 200:500]
    results = model(cropped_img, conf=0.8, verbose=False)
    result_img = results[0].plot()

    pipeline.stop()

    if results[0].obb.xywhr.numel() == 0:
        return 0, result_img
    else:
        for result in results[0].obb.xywhr:
            x_center = int(result[0])
            if 115 < x_center < 133:
                return 1, result_img
            else:
                return 0, result_img

# Function to display the result image in the GUI
def display_result_image(result_img):
    result_img = cv2.cvtColor(result_img, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(result_img)
    imgtk = ImageTk.PhotoImage(image=img)
    result_label.imgtk = imgtk
    result_label.config(image=imgtk)

# Function to update counters and pie chart
def update_counters(result_type):
    global success_count, reject_count
    if result_type == "success":
        success_count += 1
    elif result_type == "reject":
        reject_count += 1

    total_count = success_count + reject_count
    if total_count == 0:
        success_percent = 0
        reject_percent = 0
    else:
        success_percent = (success_count / total_count) * 100
        reject_percent = (reject_count / total_count) * 100

    # Schedule GUI updates in the main thread
    root.after(0, lambda: success_label.config(text=f"Successful PCBs: {success_count} ({success_percent:.1f}%)"))
    root.after(0, lambda: reject_label.config(text=f"Rejected PCBs: {reject_count} ({reject_percent:.1f}%)"))
    root.after(0, lambda: total_label.config(text=f"Total Tested PCBs: {total_count}"))

    # Schedule pie chart update
    root.after(0, lambda: update_pie_chart(success_percent, reject_percent))

def update_pie_chart(success_percent, reject_percent):
    # Create a pie chart
    fig, ax = plt.subplots(figsize=(3, 3))
    sizes = [success_percent, reject_percent]
    labels = ["Success", "Reject"]
    colors = ["green", "red"]

    ax.pie(sizes, labels=labels, autopct="%.1f%%", colors=colors, startangle=90, wedgeprops={"linewidth": 1, "edgecolor": "white"})
    ax.axis("equal")  # Equal aspect ratio ensures that pie is drawn as a circle

    # Display pie chart in the GUI
    canvas = FigureCanvasTkAgg(fig, root)
    canvas.get_tk_widget().pack()
    canvas.draw()

# Function to update total tested PCBs label
def update_total_tested():
    total_label.config(text=f"Total Tested PCBs: {total_tested}")

# Function to reset counters
def reset_counters():
    global success_count, reject_count, total_tested
    success_count = 0
    reject_count = 0
    total_tested = 0
    success_label.config(text="Successful PCBs: 0")
    reject_label.config(text="Rejected PCBs: 0")
    total_label.config(text="Total Tested PCBs: 0")
    status_label.config(text="Status: Counters Reset.")

# Function to exit the application
def exit_application():
    try:
        if robot_socket:
            robot_socket.close()
    except Exception as e:
        print(f"Error closing socket: {e}")
    root.destroy()

# GUI setup
root = tk.Tk()
root.title("PCB Testing Interface")
root.geometry("500x600")

# Labels for counters
success_label = tk.Label(root, text="Successful PCBs: 0 (0.0%)", font=("Arial", 12))
success_label.pack(pady=5)

reject_label = tk.Label(root, text="Rejected PCBs: 0 (0.0%)", font=("Arial", 12))
reject_label.pack(pady=5)

# Total tested label
total_label = tk.Label(root, text="Total Tested PCBs: 0", font=("Arial", 12))
total_label.pack(pady=5)

# Status label
status_label = tk.Label(root, text="Status: Waiting for action...", font=("Arial", 12), fg="blue")
status_label.pack(pady=10)

# Buttons
start_button = tk.Button(root, text="Start", font=("Arial", 12), command=start_action, bg="green", fg="white")
start_button.pack(pady=5)

reset_button = tk.Button(root, text="Reset Counters", font=("Arial", 12), command=reset_counters)
reset_button.pack(pady=5)

exit_button = tk.Button(root, text="Exit", font=("Arial", 12), command=exit_application, bg="red", fg="white")
exit_button.pack(pady=5)

# Image display for result
result_label = tk.Label(root)
result_label.pack(pady=10)

# Start server thread
threading.Thread(target=start_server, daemon=True).start()

# Run the application
root.mainloop()
