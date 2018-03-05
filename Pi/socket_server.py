import socket
import sys
import serial

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


    arduino = serial.Serial(sys.argv[3], sys.argv[4], timeout=2)

    while True:
        data = conn.recv(5000)
        if data:
            data = data.decode()
            data = data.split('\n')[-2] + '\n'
            # print('Client Data= {}'.format(data))
            arduino.write(data.encode())
            ard_input = arduino.readline().rstrip('\n')
            if ard_input:
                print("Arduino Data= {}".format(ard_input))
