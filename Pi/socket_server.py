import socket
import sys

if __name__ == '__main__':
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    host = sys.argv[1]
    port = int(sys.argv[2])
    server_address = (host, port)
    print("Starting socket server {}:{}".format(host, port))
    server.bind(server_address)
    server.listen(1)
    conn, addr = server.accept()

    while True:
        data = conn.recv(5000)
        print(data.decode())
