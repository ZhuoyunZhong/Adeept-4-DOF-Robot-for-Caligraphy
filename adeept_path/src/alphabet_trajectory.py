import numpy as np


def get_alphabet_trajectory(alphabet, prev_pos=[0, 0, 0], offset=[0, 0, 0], rate=25, scale=1):
    '''
    Generate the waypoints for a given character
    :param alphabet: an English Capital letter charater
    :param prev_pos: the previous x, y, z position of the end effector
    :param offset: the x, y, z, formatting offset
    :param rate: the speed of drawing, the bigger the number, fewer waypoints. (should be less than 50)
    :param scale: to make the letter bigger (scale > 1) or smaller (0 < scale < 1)
    :return:
    '''
    if alphabet == 'A':
        return drawA(prev_pos, offset, rate, scale)
    elif alphabet == 'B':
        return drawB(prev_pos, offset)
    elif alphabet == 'C':
        return drawC(prev_pos, offset)
    elif alphabet == 'D':
        return drawD(prev_pos, offset)
    elif alphabet == 'E':
        return drawE(prev_pos, offset)
    elif alphabet == 'F':
        return drawF(prev_pos, offset)
    elif alphabet == 'G':
        return drawG(prev_pos, offset)
    elif alphabet == 'H':
        return drawH(prev_pos, offset)
    elif alphabet == 'I':
        return drawI(prev_pos, offset, rate, scale)
    elif alphabet == 'J':
        return drawJ(prev_pos, offset)
    elif alphabet == 'K':
        return drawK(prev_pos, offset)
    elif alphabet == 'L':
        return drawL(prev_pos, offset)
    elif alphabet == 'M':
        return drawM(prev_pos, offset)
    elif alphabet == 'N':
        return drawN(prev_pos, offset)
    elif alphabet == 'O':
        return drawO(prev_pos, offset)
    elif alphabet == 'P':
        return drawP(prev_pos, offset)
    elif alphabet == 'Q':
        return drawQ(prev_pos, offset)
    elif alphabet == 'R':
        return drawR(prev_pos, offset)
    elif alphabet == 'S':
        return drawS(prev_pos, offset)
    elif alphabet == 'T':
        return drawT(prev_pos, offset)
    elif alphabet == 'U':
        return drawU(prev_pos, offset)
    elif alphabet == 'V':
        return drawV(prev_pos, offset)
    elif alphabet == 'W':
        return drawW(prev_pos, offset)
    elif alphabet == 'X':
        return drawX(prev_pos, offset)
    elif alphabet == 'Y':
        return drawY(prev_pos, offset)
    elif alphabet == 'Z':
        return drawZ(prev_pos, offset)


def drawA(prev_pos, offset, rate, scale):
    t_A = 3700
    waypoints = np.array([[prev_pos(0), prev_pos(1), prev_pos(2)]])
    for t in range(0, t_A, rate):
        waypoints = np.concatenate((waypoints, waypointA(t, prev_pos, offset, scale)))
    return waypoints

def drawB(prev_pos, offset):
    return 0

def drawC(prev_pos, offset):
    return 0

def drawD(prev_pos, offset):
    return 0

def drawE(prev_pos, offset):
    return 0

def drawF(prev_pos, offset):
    return 0

def drawG(prev_pos, offset):
    return 0

def drawH(prev_pos, offset):
    return 0

def drawI(prev_pos, offset, rate, scale):
    t_I = 1600
    waypoints = np.array([[prev_pos(0), prev_pos(1), prev_pos(2)]])
    for t in range(0, t_I, rate):
        waypoints = np.concatenate((waypoints, waypointI(t, prev_pos, offset, scale)))
    return waypoints

def drawJ(prev_pos, offset):
    return 0

def drawK(prev_pos, offset):
    return 0

def drawL(prev_pos, offset):
    return 0

def drawM(prev_pos, offset):
    return 0

def drawN(prev_pos, offset):
    return 0

def drawO(prev_pos, offset):
    return 0

def drawP(prev_pos, offset):
    return 0

def drawQ(prev_pos, offset):
    return 0

def drawR(prev_pos, offset):
    return 0

def drawS(prev_pos, offset):
    return 0

def drawT(prev_pos, offset):
    return 0

def drawU(prev_pos, offset):
    return 0

def drawV(prev_pos, offset):
    return 0

def drawW(prev_pos, offset):
    return 0

def drawX(prev_pos, offset):
    return 0

def drawY(prev_pos, offset):
    return 0

def drawZ(prev_pos, offset):
    return 0


def waypointA(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time * scale
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time * scale
        z = prev_xyz[2] + (0.5 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 0
        y = offset[1] + 0
        z = offset[2] + 0.5 + (0 - 0.5) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + 0 + (1.5 - 0) / 1000 * (time - 550) * scale
        y = offset[1] + 0 + (4.0 - 0) / 1000 * (time - 550) * scale
        z = offset[2] + 0
    elif time < 2550:
        x = offset[0] + (1.5 + (3.0 - 1.5) / 1000 * (time - 1550)) * scale
        y = offset[1] + (4.0 + (0 - 4.0) / 1000 * (time - 1550)) * scale
        z = offset[2] + 0
    elif time < 2600:
        x = offset[0] + 3 * scale
        y = offset[1] + 0
        z = offset[2] + 0 + (0.5 - 0) / 50 * (time - 2550)
    elif time < 3100:
        x = offset[0] + (3 + (0.75 - 3) / 500 * (time - 2600)) * scale
        y = offset[1] + (0 + (2 - 0) / 500 * (time - 2600)) * scale
        z = offset[2] + 0.5
    elif time < 3150:
        x = offset[0] + 0.75 * scale
        y = offset[1] + 2 * scale
        z = offset[2] + 0.5 + (0 - 0.5) / 50 * (time - 3100)
    elif time < 3650:
        x = offset[0] + (0.75 + (2.25 - 0.75) / 500 * (time - 3150)) * scale
        y = offset[1] + 2 * scale
        z = offset[2] + 0
    elif time <= 3700:
        x = offset[0] + 2.25 * scale
        y = offset[1] + 2 * scale
        z = offset[2] + 0 + (0.5 - 0) / 50 * (time - 3650)
    return np.array([[x/100, y/100, z/100]])


def waypointI(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz(1) + (1.5 + offset[0] - prev_xyz(1)) / 500 * time * scale
        y = prev_xyz(2) + (4.0 + offset[1] - prev_xyz(2)) / 500 * time * scale
        z = prev_xyz(3) + (0.5 - offset[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 1.5 * scale
        y = offset[1] + 4 * scale
        z = offset[2] + 0.5 + (0 - 0.5) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + 1.5 * scale
        y = offset[1] + (4 + (0 - 4.0) / 1000 * (time - 550)) * scale
        z = offset[2] + 0
    elif time <= 1600:
        x = offset[0] + 1.5 * scale
        y = offset[1] + 0 * scale
        z = offset[2] + 0 + (0.5 - 0) / 50 * (time - 1550)
    return np.array([[x / 100, y / 100, z / 100]])
