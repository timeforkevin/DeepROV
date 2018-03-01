import socket
import sys

if __name__ == '__main__':
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host = sys.argv[1]
    port = int(sys.argv[2])
    print("Connecting to server:{}:{}".format(host, port))
    client_socket.connect((host, port))

    while True:
        data = client_socket.recv(5000)
        if data:
            print(data.decode())
