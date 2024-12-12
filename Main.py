import tkinter as tk
import cv2
from PIL import Image, ImageTk
import threading
import queue
from Doosan_class import *

# Initialize counters
success_count = 0
reject_count = 0
cap = None  # Video capture variable

# Queue for communication between terminal input and GUI
input_queue = queue.Queue()

def update_counters(response_type):
    """Update counters based on the response."""
    global success_count, reject_count
    if response_type == "success":
        success_count += 1
        success_label.config(text=f"Successful PCBs: {success_count}")
        status_label.config(text="Status: PCB placed in Success Box!")
    elif response_type == "reject":
        reject_count += 1
        reject_label.config(text=f"Rejected PCBs: {reject_count}")
        status_label.config(text="Status: PCB placed in Reject Box!")

def update_camera():
    """Continuously update the video feed."""
    global cap
    if cap and cap.isOpened():
        ret, frame = cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(image=img)
            video_label.imgtk = imgtk
            video_label.config(image=imgtk)
        # Schedule the next frame update
        root.after(10, update_camera)

def stop_camera():
    """Stop the video feed."""
    global cap
    if cap and cap.isOpened():
        cap.release()
    video_label.config(image='')
    status_label.config(text="Status: Camera stopped.")


def start_action():
    """Simulate start action."""
    status_label.config(text="Status: Process Started!")
    #For now the system should count 5 seconds and add either a succes or reject, to be replaced by vision system
    def update_counts():

        if random.choice([True, False]):
            update_counters("succes")
        else:
            update_counters("reject")
        root.after(5000, update_counts)  # Herhaal iedere 5 seconden
    update_counts()

    status_label.config(text="Status: Process Started!")
    print("Start button clicked!")

def test_action():
    """Start the video feed and begin testing."""
    global cap
    status_label.config(text="Status: Testing in progress...")
    cap = cv2.VideoCapture(0)  # Replace 0 with your camera index if needed
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        status_label.config(text="Error: Unable to access the camera.")
        return
    update_camera()  # Start updating frames
    threading.Thread(target=listen_to_robot, daemon=True).start()

def listen_to_robot():
    """Simulate robot response by taking input from the terminal."""
    while True:
        try:
            response = input_queue.get()  # Get response from the queue
            if response == "1":
                update_counters("success")
            elif response == "0":
                update_counters("reject")
            else:
                print("Invalid input. Enter '1' for success or '0' for reject.")
        except Exception as e:
            print(f"Error: {e}")

def reset_counters():
    """Reset all counters to zero."""
    global success_count, reject_count
    success_count = 0
    reject_count = 0
    success_label.config(text="Successful PCBs: 0")
    reject_label.config(text="Rejected PCBs: 0")
    status_label.config(text="Status: Counters Reset.")

def exit_application():
    """Exit the application."""
    stop_camera()
    root.destroy()

def switch_to_test():
    """Switch the current screen to the Test2 view."""
    clear_screen()
    tk.Label(root, text="Test2 View", font=("Arial", 16)).pack(pady=10)

    test_button1 = tk.Button(root, text="Add Succes", font=("Arial", 12), command=lambda: update_counters("success"))
    test_button1.pack(pady=5)

    test_button2 = tk.Button(root, text="Add Reject", font=("Arial", 12), command=lambda: update_counters("reject"))
    test_button2.pack(pady=5)

    test_button3 = tk.Button(root, text="Home robot to unpack", font=("Arial", 12), command=lambda: robot.home_pos())
    test_button3.pack(pady=5)

    back_button = tk.Button(root, text="Back", font=("Arial", 12), command=setup_gui)
    back_button.pack(pady=20)

def clear_screen():
    """Remove all widgets from the root window."""
    for widget in root.winfo_children():
        widget.destroy()

def setup_gui():
    """Setup the GUI components."""
    clear_screen()

    robot = Doosan()   # Initiate a robot for doosan class

    global success_label, reject_label, status_label, video_label

    # Labels for counters
    success_label = tk.Label(root, text=f"Successful PCBs: {success_count}", font=("Arial", 14))
    success_label.pack(pady=10)

    reject_label = tk.Label(root, text=f"Rejected PCBs: {reject_count}", font=("Arial", 14))
    reject_label.pack(pady=10)

    # Status label
    status_label = tk.Label(root, text="Status: Waiting for action...", font=("Arial", 12), fg="blue")
    status_label.pack(pady=10)

    # Buttons
    start_button = tk.Button(root, text="Start", font=("Arial", 12), command=start_action, bg="green", fg="white")
    start_button.pack(pady=5)

    test2_button = tk.Button(root, text="Test", font=("Arial", 12), command=switch_to_test, bg="orange", fg="white")
    test2_button.pack(pady=5)

    reset_button = tk.Button(root, text="Reset Counters", font=("Arial", 12), command=reset_counters)
    reset_button.pack(pady=10)

    exit_button = tk.Button(root, text="Exit", font=("Arial", 12), command=exit_application, bg="red", fg="white")
    exit_button.pack(pady=10)

    # Video display
    video_label = tk.Label(root)
    video_label.pack()

# GUI setup
root = tk.Tk()
root.title("PCB Testing Interface")
root.geometry("800x600")

setup_gui()

# Start a thread for manual input simulation
#threading.Thread(target=manual_input_simulation, daemon=True).start()

# Run the application
root.mainloop()
