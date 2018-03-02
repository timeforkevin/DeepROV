import sys
import time
from xbox_controller import XboxController
import socket

OFFSET = 1500
MOTORS = [OFFSET] * 5

def left_thumb_x(x):
    if (x > 0):
        MOTORS[0] = x + OFFSET
    else:
        MOTORS[1] = abs(x) + OFFSET

def left_thumb_y(y):
    val = y + OFFSET
    MOTORS[0] = val
    MOTORS[1] = val

def right_thumb_x(x):
    if(x > 0):
        MOTORS[3] = x + OFFSET
        MOTORS[2] = -x + OFFSET
    else:
        MOTORS[2] = abs(x) + OFFSET
        MOTORS[3] = x + OFFSET

def right_thumb_y(y):
    val1 = y + OFFSET
    val2 = -y + OFFSET
    if(y > 0):
        MOTORS[2] = val1
        MOTORS[3] = val1
        MOTORS[4] = val2
    else:
        MOTORS[2] = val1
        MOTORS[3] = val1
        MOTORS[4] = val2

def right_trigger(val):
    MOTORS[2] = MOTORS[3] = MOTORS[4] = val + OFFSET

def left_trigger(val):
    MOTORS[2] = MOTORS[3] = MOTORS[4] = -val + OFFSET



if __name__ == '__main__':
    xboxCont = XboxController(deadzone = 30, scale = 300, invertYAxis = True)

    xboxCont.setupControlCallback(xboxCont.XboxControls.LTHUMBX, left_thumb_x)
    xboxCont.setupControlCallback(xboxCont.XboxControls.LTHUMBY, left_thumb_y)
    xboxCont.setupControlCallback(xboxCont.XboxControls.RTHUMBX, right_thumb_x)
    xboxCont.setupControlCallback(xboxCont.XboxControls.RTHUMBY, right_thumb_y)
    xboxCont.setupControlCallback(xboxCont.XboxControls.RTRIGGER, right_trigger)
    xboxCont.setupControlCallback(xboxCont.XboxControls.LTRIGGER, left_trigger)

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host = sys.argv[1]
    port = int(sys.argv[2])
    print('Connecting to server:{}:{}'.format(host, port))
    client_socket.connect((host, port))


    try:
        xboxCont.start()
        print('xbox controller running')
        while True:
            output = ''
            for idx, motor in enumerate(MOTORS, 1):
                if idx == len(MOTORS):
                    output += str(idx) + str(int(motor)) + '\n'
                else:
                    output += str(idx) + str(int(motor)) + ','
            print(output)
            client_socket.sendall(output.encode())
            time.sleep(0.2)

    except KeyboardInterrupt:
        print('User cancelled')

    except:
        print('Unexpected error:', sys.exc_info()[0])
        raise

    finally:
        xboxCont.stop()
