import sys
import time
from xbox_controller import XboxController
import socket
import math

COMMAD_MAP = {
    'V': '0',
    'Z': '0',
    'Y': '0',
    'P': '0',
    'R': '0',
}

def build_msg():
    msg = ''
    for key, value in COMMAND_MAP.items():
        msg += '{}{}'.format(key,value)
    return msg

def left_thumb_x(val):
    out = abs(val) * math.pi / 100
    if val < 0:
        out = -out
    COMMAND_MAP['Y'] = str(out)

def left_thumb_y(val):
    COMMAND_MAP['V'] = str(val)

def right_trigger(val):
    val = val/10
    COMMAND_MAP['Z'] = str(val)

def left_trigger(val):
    val = -val/10
    COMMAND_MAP['Z'] = str(val)

def Y(val):
    COMMAND_MAP['P'] = str(1)

def A(val):
    COMMAND_MAP['P'] = str(-1)

def B(val):
    COMMAND_MAP['R'] = str(1)

def X(val):
    COMMAND_MAP['R'] = str(-1)


if __name__ == '__main__':
    xboxCont = XboxController(deadzone = 30, scale = 100, invertYAxis = True)

    xboxCont.setupControlCallback(xboxCont.XboxControls.LTHUMBX, left_thumb_x)
    xboxCont.setupControlCallback(xboxCont.XboxControls.LTHUMBY, left_thumb_y)
    xboxCont.setupControlCallback(xboxCont.XboxControls.RTRIGGER, right_trigger)
    xboxCont.setupControlCallback(xboxCont.XboxControls.LTRIGGER, left_trigger)
    xboxCont.setupControlCallback(xboxCont.XboxControls.A, A)
    xboxCont.setupControlCallback(xboxCont.XboxControls.B, B)
    xboxCont.setupControlCallback(xboxCont.XboxControls.X, X)
    xboxCont.setupControlCallback(xboxCont.XboxControls.Y, Y)

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host = sys.argv[1]
    port = int(sys.argv[2])
    print('Connecting to server:{}:{}'.format(host, port))
    client_socket.connect((host, port))


    try:
        xboxCont.start()
        print('xbox controller running')
        while True:
            msg = build_msg
            client_socket.sendall(msg.encode())
            time.sleep(0.2)

    except KeyboardInterrupt:
        print('User cancelled')

    except:
        print('Unexpected error:', sys.exc_info()[0])
        raise

    finally:
        xboxCont.stop()
