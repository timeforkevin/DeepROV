import sys
import time
from xbox_controller import XboxController
import socket
import math

COMMAND_MAP = {
    'X': '0',
    'Z': '0',
    'Y': '0',
    'P': '0',
    'R': '0',
}


def build_msg():
    msg = COMMAND_MAP['X'] + ',' + COMMAND_MAP['R'] + ',' + COMMAND_MAP['Y'] + ',' + COMMAND_MAP['Z'] + ',' + COMMAND_MAP['P'] + '\n'
    return msg

def left_thumb_x(val):
    out = int(abs(val) * math.pi / 100)
    if val < 0:
        out = -out
    COMMAND_MAP['Y'] = str(out)

def left_thumb_y(val):
    COMMAND_MAP['X'] = str(int(val))

def right_trigger(val):
    val = val/10
    COMMAND_MAP['Z'] = str(int(val))

def left_trigger(val):
    val = -val/10
    COMMAND_MAP['Z'] = str(int(val))

def Y(val):
    if val == 1:
        COMMAND_MAP['P'] = str(5)

def A(val):
    if val == 1:
        COMMAND_MAP['P'] = str(-5)

def B(val):
    if val == 1:
        COMMAND_MAP['R'] = str(5)

def X(val):
    if val == 1:
        COMMAND_MAP['R'] = str(-5)

def reset_command_vals():
    COMMAND_MAP['R'] = str(0)
    COMMAND_MAP['P'] = str(0)


if __name__ == '__main__':

     #generic call back
    def controlCallBack(xboxControlId, value):
        print("Control Id = {}, Value = {}".format(xboxControlId, value))

    xboxCont = XboxController(deadzone = 30, scale = 100, invertYAxis = True)

    xboxCont.setupControlCallback(xboxCont.XboxControls.LTHUMBX, left_thumb_x)
    xboxCont.setupControlCallback(xboxCont.XboxControls.LTHUMBY, left_thumb_y)
    xboxCont.setupControlCallback(xboxCont.XboxControls.RTRIGGER, right_trigger)
    xboxCont.setupControlCallback(xboxCont.XboxControls.LTRIGGER, left_trigger)
    xboxCont.setupControlCallback(xboxCont.XboxControls.A, A)
    xboxCont.setupControlCallback(xboxCont.XboxControls.B, B)
    xboxCont.setupControlCallback(xboxCont.XboxControls.X, X)
    xboxCont.setupControlCallback(xboxCont.XboxControls.Y, Y)
    #
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host = sys.argv[1]
    port = int(sys.argv[2])
    print('Connecting to server:{}:{}'.format(host, port))
    client_socket.connect((host, port))


    try:
        xboxCont.start()
        print('xbox controller running')
        while True:
            msg = build_msg()
            print(msg)
            client_socket.sendall(msg.encode())
            reset_command_vals()
            time.sleep(0.2)

    except KeyboardInterrupt:
        print('User cancelled')

    except:
        print('Unexpected error:', sys.exc_info()[0])
        raise

    finally:
        xboxCont.stop()
