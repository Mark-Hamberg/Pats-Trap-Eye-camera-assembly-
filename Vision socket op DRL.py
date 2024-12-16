from DRCF import *
import socket

IP = "192.168.137.69"
PORT = 12345

socket = client_socket_open(IP, PORT)

resp, data = client_socket_read(socket, length=4, timeout=60)
decoded_string = data.decode('utf-8')
received_number = int(decoded_string)
tp_log(str(received_number))

if received_number == 1:
    movel(posx(-365.3, 6.3, 849.8, 5.4, -70.1, 71.0), v = 20, a = 20)
else:
    movel(posx(-348.8, 569.6, 849.8, 5.4, -70.1, 71.0), v = 20, a = 20)