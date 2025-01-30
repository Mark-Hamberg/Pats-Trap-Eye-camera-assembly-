import tkinter as tk
from tkinter import ttk
import cv2
from PIL import Image, ImageTk
import threading
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import socket
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Initialize counters
success_count = 0
reject_count = 0
total_tested = 0

# Load YOLO model
model = YOLO(r'E:\PATS\Train5\Love.pt')

# Global socket connection
robot_socket = None

def start_server():
    global robot_socket
    HOST = '0.0.0.0'
    PORT = 12345

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print("Server is starting...")
        s.bind((HOST, PORT))
        s.listen()
        print("Waiting for the robot to connect...")
        conn, addr = s.accept()
        robot_socket = conn
        print(f"Connected by {addr}")
        listen_for_robot()


def listen_for_robot():
    global robot_socket
    while True:
        try:
            data = robot_socket.recv(1024).decode()
            if not data:
                print("Robot disconnected.")
                break

            if data == "GOT IT":
                print("Received 'GOT IT' from the robot. Starting quality control...")
                quality_control_process()
        except ConnectionResetError:
            print("Robot connection reset.")
            break
        except Exception as e:
            print(f"Error receiving data: {e}")
            break

def send_to_robot(message):
    global robot_socket
    if robot_socket:
        try:
            robot_socket.sendall(str(message).encode())
            print(f"Sent to robot: {message}")
        except Exception as e:
            print(f"Error sending data: {e}")

def start_action():
    status_label.config(text="Status: Sending 'START ROBOT'...")
    send_to_robot("START ROBOT")


def quality_control_process():
    global total_tested
    result, result_img = quality_control()
    display_result_image(result_img)
    total_tested += 1
    update_total_tested()

    if result == 1:
        update_counters("success")
    else:
        update_counters("reject")

    send_to_robot(result)

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

def display_result_image(result_img):
    result_img = cv2.cvtColor(result_img, cv2.COLOR_BGR2RGB)
    img = Image.fromarray(result_img)
    imgtk = ImageTk.PhotoImage(image=img)
    result_label.imgtk = imgtk
    result_label.config(image=imgtk)

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

    success_label.config(text=f"Successful PCBs: {success_count}")
    reject_label.config(text=f"Rejected PCBs: {reject_count}")
    total_label.config(text=f"Total Tested PCBs: {total_count}")

    success_bar["value"] = success_percent
    reject_bar["value"] = reject_percent

def update_total_tested():
    total_label.config(text=f"Total Tested PCBs: {total_tested}")

def reset_counters():
    global success_count, reject_count, total_tested
    success_count = 0
    reject_count = 0
    total_tested = 0
    success_label.config(text="Successful PCBs: 0")
    reject_label.config(text="Rejected PCBs: 0")
    total_label.config(text="Total Tested PCBs: 0")
    success_bar["value"] = 0
    reject_bar["value"] = 0
    status_label.config(text="Status: Counters Reset.")

def exit_application():
    try:
        if robot_socket:
            robot_socket.close()
    except Exception as e:
        print(f"Error closing socket: {e}")
    root.destroy()

# GUI setup
root = tk.Tk()
root.title("PCB Testing Dashboard")
root.geometry("800x700")

# Header Label
header_label = tk.Label(root, text="PCB Testing Dashboard", font=("Arial", 24, "bold"), fg="#333")
header_label.pack(pady=20)

# Progress Bars
progress_frame = tk.Frame(root)
progress_frame.pack(pady=10)

success_bar = ttk.Progressbar(progress_frame, orient="horizontal", length=400, mode="determinate", maximum=100)
success_bar.grid(row=0, column=1, pady=5)

reject_bar = ttk.Progressbar(progress_frame, orient="horizontal", length=400, mode="determinate", maximum=100)
reject_bar.grid(row=1, column=1, pady=5)

success_label = tk.Label(progress_frame, text="Successful PCBs: 0", font=("Arial", 14))
success_label.grid(row=0, column=0, padx=10)

reject_label = tk.Label(progress_frame, text="Rejected PCBs: 0", font=("Arial", 14))
reject_label.grid(row=1, column=0, padx=10)

# Total tested label
total_label = tk.Label(root, text="Total Tested PCBs: 0", font=("Arial", 16))
total_label.pack(pady=10)

# Status label
status_label = tk.Label(root, text="Status: Waiting for action...", font=("Arial", 14), fg="blue")
status_label.pack(pady=10)

# Image display for result
result_label = tk.Label(root)
result_label.pack(pady=10)

# Buttons
button_frame = tk.Frame(root)
button_frame.pack(pady=20)

start_button = tk.Button(button_frame, text="Start", font=("Arial", 14), command=start_action, bg="green", fg="white", width=10)
start_button.grid(row=0, column=0, padx=20)

reset_button = tk.Button(button_frame, text="Reset Counters", font=("Arial", 14), command=reset_counters, bg="#666", fg="white", width=15)
reset_button.grid(row=0, column=1, padx=20)

exit_button = tk.Button(button_frame, text="Exit", font=("Arial", 14), command=exit_application, bg="red", fg="white", width=10)
exit_button.grid(row=0, column=2, padx=20)

# Start server thread
threading.Thread(target=start_server, daemon=True).start()

# Run the application
root.mainloop()