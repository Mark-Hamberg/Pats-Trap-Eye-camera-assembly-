import socket

# Robot's IP address and port
ROBOT_IP = "192.168.137.4"  # Replace with the robot's IP
ROBOT_PORT = 12345        # Replace with the robot's port

# Create a TCP/IP socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    # Connect to the robot
    print(f"Connecting to Doosan robot at {ROBOT_IP}:{ROBOT_PORT}...")
    client_socket.connect((ROBOT_IP, ROBOT_PORT))
    print("Connected!")

    # Example command to send to the robot
    # Replace 'COMMAND' with the appropriate command for your robot
    command = "MOVEJ 10 20 30 40 50 60"  # Example: Move to joint positions
    client_socket.sendall(command.encode('utf-8'))
    print(f"Command sent: {command}")

    # Receive the response from the robot
    response = client_socket.recv(1024)  # Adjust buffer size if needed
    print(f"Response received: {response.decode('utf-8')}")

except Exception as e:
    print(f"Error: {e}")

finally:
    # Close the connection
    print("Closing connection.")
    client_socket.close()
