import socket
import sys
import serial
import time


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


    arduino = serial.Serial(sys.argv[3], sys.argv[4], timeout=3)
    arduino.flushInput()
    arduino.flushOutput()

    count = 0

    flag = False
    while True:
        try:
            ard_input = arduino.read(arduino.inWaiting())
            if ard_input:
                count += 1
        except Exception as ex:
            print("There was error reading the arduino serial input: {}".format(ex))
            break
        try:
            data = conn.recv(5000)
            if data and count > 20:
                data = data.decode()
                data = data.split('\n')[-2] + '\n'
                print('Client Data= {}'.format(data))
                try:
                    arduino.write(data.encode())
                except Exception as ex:
                    print("There was an error writing data to the arduino: {}".format(ex))
                    break
        except Exception as ex:
            print("There was an error reading the client data: {}".format(ex))
            break
