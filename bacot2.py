import tkinter as tk
import cv2
from PIL import Image, ImageTk
import threading
import queue

# Initialize counters
success_count = 0
reject_count = 0
cap = None  # Video capture variable

# Queue for communication between terminal input and GUI
input_queue = queue.Queue()

def start_action():
    """Simulate start action."""
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

def manual_input_simulation():
    """Simulate terminal input by letting the user enter data."""
    while True:
        response = input("Enter robot response (1 for success, 0 for reject): ")
        input_queue.put(response)  # Put the response in the queue

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

# GUI setup
root = tk.Tk()
root.title("PCB Testing Interface")
root.geometry("800x600")

# Labels for counters
success_label = tk.Label(root, text="Successful PCBs: 0", font=("Arial", 14))
success_label.pack(pady=10)

reject_label = tk.Label(root, text="Rejected PCBs: 0", font=("Arial", 14))
reject_label.pack(pady=10)

# Status label
status_label = tk.Label(root, text="Status: Waiting for action...", font=("Arial", 12), fg="blue")
status_label.pack(pady=10)

# Buttons
start_button = tk.Button(root, text="Start", font=("Arial", 12), command=start_action, bg="green", fg="white")
start_button.pack(pady=5)

test_button = tk.Button(root, text="Test", font=("Arial", 12), command=test_action, bg="blue", fg="white")
test_button.pack(pady=5)

reset_button = tk.Button(root, text="Reset Counters", font=("Arial", 12), command=reset_counters)
reset_button.pack(pady=10)

exit_button = tk.Button(root, text="Exit", font=("Arial", 12), command=exit_application, bg="red", fg="white")
exit_button.pack(pady=10)

# Video display
video_label = tk.Label(root)
video_label.pack()

# Start a thread for manual input simulation
threading.Thread(target=manual_input_simulation, daemon=True).start()

# Run the application
root.mainloop()
