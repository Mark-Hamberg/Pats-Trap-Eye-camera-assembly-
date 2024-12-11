import socket
import select
import time
#import ipaddress # 3.2 not supported

from DR_error import *

# =============================================================================================
# define

DR_TCP_SERVER_CONNET_TIMEOUT = 1
DR_TCP_SERVER_COMM_TIMEOUT   = 0.01
DR_TCP_SERVER_BUFF_SIZE      = 4096

DR_TCP_SERVER_CONN_LIST = dict()
DR_TCP_SERVER_CONN_STATE_LIST = dict()
DR_TCP_SERVER_END_DATA = dict()


def server_socket_open(port) -> socket.socket:
    """
    The robot controller creates a server socket and waits for the connection to the client.
    The connected socket is returned when the client is connected.

    :param port: int - Server port number
    :return: socket.socket instance
    """
    # port
    if type(port) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : port")

    if port < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : port")

    while True:
        try:
            # create socket
            server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 3)

            # bind the socket to the port
            #server_address = ("localhost", port)
            server_address = ('', port)
            server_sock.bind(server_address)
    
            # listen for incoming connections (only 1)
            server_sock.listen(1)
    
            server_sock.settimeout(DR_TCP_SERVER_CONNET_TIMEOUT)

            # accept
            #------------------------------------------------------
            client_sock, client_address = server_sock.accept()
            #------------------------------------------------------

            # connected
            DR_TCP_SERVER_CONN_LIST[id(client_sock)] = (client_sock, server_sock)
            DR_TCP_SERVER_CONN_STATE_LIST[id(client_sock)] = 1
            print("_____OPEN SERVER SOCKET_____ : ", client_sock)
            print("_____OPEN SERVER SOCKET_____ : ", server_sock)

            client_sock.settimeout(DR_TCP_SERVER_COMM_TIMEOUT)
            return client_sock

        except socket.error as msg:

            print("server_socket_open() Socket Error: "+str(msg))
            #print(client_sock)
            print(server_sock)
            #sock.shutdown(socket.SHUT_RDWR)
            #client_sock.close()
            server_sock.close()
            time.sleep(0.5)
            print("SERVER retry openning...")
            continue

    return client_sock


def server_socket_close(sock) -> int:
    """
    This function terminates communication with the client.
    To reconnect to the client, the socket must be closed with server_socket_close(sock) and reopened.

    :param sock: socket.socket - Socket instance returned from server_socket_open()socket instance
    :return: int - (0 -> Success, Negative value -> Error)
    """
    try:
        # sock
        if type(sock) != socket.socket:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'sock' is not socket object.")

        if id(sock) not in DR_TCP_SERVER_CONN_LIST.keys():
            print("sock is not opend!!")
            return -1

        client_socket, server_socket = DR_TCP_SERVER_CONN_LIST[id(sock)]

        print("_____CLOSE SERVER SOCKET_0___ : ", client_socket)
        print("_____CLOSE SERVER SOCKET_0___ : ", server_socket)

        ###client_socket.shutdown(socket.SHUT_RDWR)
        client_socket.close()

        ###server_socket.shutdown(socket.SHUT_RDWR)
        server_socket.close()

        del DR_TCP_SERVER_CONN_STATE_LIST[id(sock)]
        del DR_TCP_SERVER_CONN_LIST[id(sock)]

    except socket.error as msg:
        raise DR_Error(DR_ERROR_RUNTIME, "server_socket_close() Socket Error: ", msg)

    return 0


def clean_server_socket():
    print("         clean_server_socket() call")

    for sock_id in list(DR_TCP_SERVER_CONN_LIST.keys()):
        client_socket, server_socket = DR_TCP_SERVER_CONN_LIST[sock_id]

        print("_____CLOSE SERVER SOCKET_1___ : ", client_socket)
        print("_____CLOSE SERVER SOCKET_1___ : ", server_socket)

        ###client_socket.shutdown(socket.SHUT_RDWR)
        client_socket.close()

        ###server_socket.shutdown(socket.SHUT_RDWR)
        server_socket.close()

        del DR_TCP_SERVER_CONN_STATE_LIST[sock_id]
        del DR_TCP_SERVER_CONN_LIST[sock_id]

    DR_TCP_SERVER_END_DATA.clear()

    return None


def server_socket_state(sock) -> int:
    """
    This function returns the socket connection status.

    :param sock: socket.socket - Socket instance returned from client_socket_open()
    :return: (1: Connected state, 0: Disconnected state)
    """
    # sock
    if type(sock) != socket.socket:
        #raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'sock' is not socket object.")
        return 0

    # check opened
    if id(sock) in DR_TCP_SERVER_CONN_LIST.keys():
        #return 1
        return  DR_TCP_SERVER_CONN_STATE_LIST[id(sock)]
    else:
        return 0


def server_socket_end_data(sock, end_data):
    # sock
    if type(sock) != socket.socket:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'sock' is not socket object.")

    # end_data
    if type(end_data) != str:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : end_data")

    # set
    DR_TCP_SERVER_END_DATA[str(id(sock))] = end_data

    return 0


def server_socket_write(sock, tx_data) -> int:
    """
    This function transmits data to the client.

    :param sock: socket.socket - Socket instance returned from server_socket_open()
    :param tx_data: byte - Data to be transmitted.
    :return: int - (0 -> Success, Negative value -> Error)
    """
    try:
        # sock
        if type(sock) != socket.socket:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'sock' is not socket object.")

        # data
        if type(tx_data) != bytes:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : tx_data")

        # check opened
        if id(sock) not in DR_TCP_SERVER_CONN_LIST.keys():
            print("Connection is not alive!!")
            return -1

        com_end_data = DR_TCP_SERVER_END_DATA.get(id(sock), "")
        sock.sendall(tx_data + bytes(com_end_data, 'ascii'))

    except socket.error as msg:
        print("server_socket_write() Socket Error: ", msg)
        return -2

    return 0


def server_socket_read(sock, length=-1, timeout=-1) -> (int, bytes):
    """
    This function receives data from the client.

    :param sock: socket.socket - Socket instance returned from server_socket_open()
    :param length: Number of bytes of the received data. -1: Not specified (The number of bytes to read is not specified). n(>=0): The specified number of bytes is read.
    :param timeout: Waiting time for receipt. (-1: Indefinite wait, n(>0): n seconds)
    :return: Number of bytes ofthe received data.
    """
    rx_data = None
    time_cnt = 0
    rxd_size = 0

    # sock
    if type(sock) != socket.socket:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'sock' is not socket object.")

    # length
    if type(length) != int:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : length")

    if length != -1 and length < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : length")

    # timeout
    #if type(timeout) != int:
    if type(timeout) != int and type(timeout) != float:
        raise DR_Error(DR_ERROR_TYPE, "Invalid type : timeout")

    if timeout != -1 and timeout < 0:
        raise DR_Error(DR_ERROR_VALUE, "Invalid value : timeout")

    # check opened
    if id(sock) not in DR_TCP_SERVER_CONN_LIST.keys():
        print("Connection is not alive!!")
        return -1, None

    # check length
    if length == -1:
        rxd_size = DR_TCP_SERVER_BUFF_SIZE
    else:
        rxd_size = length

    #---- READ DATA ---------------------------------------------------------
    while True:
        try:
            rxd = sock.recv(rxd_size)
        except socket.timeout as e:
            err = e.args[0]
            # this next if/else is a bit redundant, but illustrates how the
            # timeout exception is setup
            if err == "timed out":
                if timeout == -1:
                    #print("server_socket_read(): recv timed out, retry later")
                    continue
                else: 
                    time_cnt = time_cnt +1 
                    if (DR_TCP_SERVER_COMM_TIMEOUT*time_cnt) >= timeout:
                        print("server_socket_read() time-out")
                        return -3, None
                    else:
                        continue
            else:
                print(e)
                print("server_socket_read(): except socket.error <1>]")
                return -2, None
        except socket.error as e:
            # Something else happened, handle error, exit, etc.
            print(e)
            print("server_socket_read(): except socket.error <2>")
            return -2, None
        else:
            if len(rxd) == 0:
                print("server_socket_read(): Client is disconnected")
                DR_TCP_SERVER_CONN_STATE_LIST[id(sock)] = 0
                return -1, None
            else:
                #print("OKOKOKOK!!!!!!!!!!!!!!!!!!")
                #print(rxd)
                rx_data = bytes(rxd)
                break

    return len(rx_data), rx_data


def server_socket_flush(sock):
    try:
        # sock
        if type(sock) != socket.socket:
            raise DR_Error(DR_ERROR_TYPE, "Invalid type : 'sock' is not socket object.")

        # check opened
        if id(sock) not in DR_TCP_SERVER_CONN_LIST.keys():
            print("Connection is not alive!!")
            return -1

        # check ready
        while True:
            r_ready, w_ready, error = select.select([sock], [sock], [sock], 0.1)

            # check error
            if error:
                print("Failed to read!! (socket)")
                return -3

            # if read data...
            if r_ready:
                dummy = sock.recv(DR_TCP_SERVER_BUFF_SIZE)
                continue

            # if write ready...
            if w_ready:
                break

            time.sleep(0.01)

    except socket.error as msg:
        print("server_socket_flush() Socket Error: ", msg)
        return -2

    return 0
