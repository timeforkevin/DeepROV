import sys
import time
from xbox_controller import XboxController
import socket

def left_thumb_x(x):
    if (x > 0):
        motors[0] = x + 1500
    else:
        motors[1] = abs(x) + 1500

def left_thumb_y(y):
    val = y + 1500
    motors[0] = val
    motors[1] = val

def right_thumb_x(x):
    if(x > 0):
        motors[3] = x + 1500
    else:
        motors[2] = abs(x) + 1500


def right_thumb_y(y):
    val1 = y + 1500
    val2 = -y + 1500
    if(y > 0):
        motors[2] = val1
        motors[3] = val1
        motors[4] = val2
    else:
        motors[2] = val2
        motors[3] = val2
        motors[4] = val1



if __name__ == '__main__':

    motors = [1500] * 5


    xboxCont = XboxController(deadzone = 30, scale = 200, invertYAxis = True)

    xboxCont.setupControlCallback(xboxCont.XboxControls.LTHUMBX, left_thumb_x)
    xboxCont.setupControlCallback(xboxCont.XboxControls.LTHUMBY, left_thumb_y)
    xboxCont.setupControlCallback(xboxCont.XboxControls.RTHUMBX, right_thumb_x)
    xboxCont.setupControlCallback(xboxCont.XboxControls.RTHUMBY, right_thumb_y)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    host = sys.argv[1]
    port = int(sys.argv[2])
    server_address = (host, port)
    print("Starting socket server {}:{}".format(host, port))
    server.bind(server_address)
    server.listen(1)
    conn, addr = server.accept()


    try:
        xboxCont.start()
        print("xbox controller running")
        while True:
            output = ''
            for idx, motor in enumerate(motors, 1):
                if idx == len(motors):
                    output += str(idx) + str(int(motor))
                else:
                    output += str(idx) + str(int(motor)) + ','

            conn.sendall(output.encode())
            time.sleep(0.25)
    #Ctrl C
    except KeyboardInterrupt:
        print("User cancelled")

    #error
    except:
        print("Unexpected error:", sys.exc_info()[0])
        raise

    finally:
        #stop the controller
        xboxCont.stop()
