import numpy as np

def get_string_trajectory(string, start_pos=[0,0,0], fix_scale = -1, y_alinement = 'left'):

    # change the x and y limit of the canvas to get the right scale letters
    # the total length from y_min to y_max with 0 in the middle, in meters
    y_len = 30
    char_len = len(string)
    if fix_scale == -1:
        scale = (y_len/char_len)/4 # scale * 4 = actual char width
    else:
        scale = fix_scale

    char_y_len = scale * 4
    char_list = list(string)
    waypoints = np.array([[start_pos[0], start_pos[1], start_pos[2]]])
    if y_alinement == 'left':
        for i in range(0, char_len - 1):
            waypoints = np.concatenate((waypoints,
                                        get_alphabet_trajectory(char_list[i], list(waypoints[-1]),
                                                                [0, -(y_len/2) + i * char_y_len, 0], scale)))
        return waypoints
    elif y_alinement == 'right':
        for i in range(0, char_len - 1):
            waypoints = np.concatenate((waypoints,
                                        get_alphabet_trajectory(char_list[i], list(waypoints[-1]),
                                                                [0, (y_len/2-char_y_len*char_len)+i*char_y_len, 0],
                                                                scale)))
        return waypoints
    elif y_alinement == 'center':
        for i in range(0, char_len - 1):
            waypoints = np.concatenate((waypoints,
                                        get_alphabet_trajectory(char_list[i], list(waypoints[-1]),
                                                                [0, -(char_y_len*char_len/2)+i*char_y_len, 0], scale)))
        return waypoints








def get_alphabet_trajectory(alphabet, prev_pos=[0, 0, 0], offset=[0, 0, 0], scale=0.05, rate=25):
    '''
    Generate the waypoints for a given character
    :param alphabet: an English Capital letter charater
    :param prev_pos: the previous x, y, z position of the end effector
    :param offset: the x, y, z, formatting offset
    :param scale: to make the letter bigger (scale > 1) or smaller (0 < scale < 1)
    :param rate: the speed of drawing, the bigger the number, fewer waypoints. (should be less than 50)
    :return: n by 3 array of way points
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
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_A, rate):
        waypoints = np.concatenate((waypoints, waypointA(t, prev_pos, offset, scale)))
    return waypoints

def drawB(prev_pos, offset, rate, scale):
    t_B = 5600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_B, rate):
        waypoints = np.concatenate((waypoints, waypointB(t, prev_pos, offset, scale)))
    return waypoints

def drawC(prev_pos, offset, rate, scale):
    t_C = 4100
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_C, rate):
        waypoints = np.concatenate((waypoints, waypointC(t, prev_pos, offset, scale)))
    return waypoints

def drawD(prev_pos, offset, rate, scale):
    t_D = 3600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_D, rate):
        waypoints = np.concatenate((waypoints, waypointD(t, prev_pos, offset, scale)))
    return waypoints

def drawE(prev_pos, offset, rate, scale):
    t_E = 5200
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_E, rate):
        waypoints = np.concatenate((waypoints, waypointE(t, prev_pos, offset, scale)))
    return waypoints

def drawF(prev_pos, offset, rate, scale):
    t_F = 4200
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_F, rate):
        waypoints = np.concatenate((waypoints, waypointF(t, prev_pos, offset, scale)))
    return waypoints

def drawG(prev_pos, offset, rate, scale):
    t_G = 5100
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_G, rate):
        waypoints = np.concatenate((waypoints, waypointG(t, prev_pos, offset, scale)))
    return waypoints

def drawH(prev_pos, offset, rate, scale):
    t_H = 4800
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_H, rate):
        waypoints = np.concatenate((waypoints, waypointH(t, prev_pos, offset, scale)))
    return waypoints

def drawI(prev_pos, offset, rate, scale):
    t_I = 1600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_I, rate):
        waypoints = np.concatenate((waypoints, waypointI(t, prev_pos, offset, scale)))
    return waypoints

import numpy as np

def get_string_trajectory(string, start_pos=[0,0,0], fix_scale = -1, y_alinement = 'left'):

    # change the x and y limit of the canvas to get the right scale letters
    # the total length from y_min to y_max with 0 in the middle, in meters
    y_len = 30
    char_len = len(string)
    if fix_scale == -1:
        scale = (y_len/char_len)/4 # scale * 4 = actual char width
    else:
        scale = fix_scale

    char_y_len = scale * 4
    char_list = list(string)
    waypoints = np.array([[start_pos[0], start_pos[1], start_pos[2]]])
    if y_alinement == 'left':
        for i in range(0, char_len - 1):
            waypoints = np.concatenate((waypoints,
                                        get_alphabet_trajectory(char_list[i], list(waypoints[-1]),
                                                                [0, -(y_len/2) + i * char_y_len, 0], scale)))
        return waypoints
    elif y_alinement == 'right':
        for i in range(0, char_len - 1):
            waypoints = np.concatenate((waypoints,
                                        get_alphabet_trajectory(char_list[i], list(waypoints[-1]),
                                                                [0, (y_len/2-char_y_len*char_len)+i*char_y_len, 0],
                                                                scale)))
        return waypoints
    elif y_alinement == 'center':
        for i in range(0, char_len - 1):
            waypoints = np.concatenate((waypoints,
                                        get_alphabet_trajectory(char_list[i], list(waypoints[-1]),
                                                                [0, -(char_y_len*char_len/2)+i*char_y_len, 0], scale)))
        return waypoints








def get_alphabet_trajectory(alphabet, prev_pos=[0, 0, 0], offset=[0, 0, 0], scale=0.05, rate=25):
    '''
    Generate the waypoints for a given character
    :param alphabet: an English Capital letter charater
    :param prev_pos: the previous x, y, z position of the end effector
    :param offset: the x, y, z, formatting offset
    :param scale: to make the letter bigger (scale > 1) or smaller (0 < scale < 1)
    :param rate: the speed of drawing, the bigger the number, fewer waypoints. (should be less than 50)
    :return: n by 3 array of way points
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
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_A, rate):
        waypoints = np.concatenate((waypoints, waypointA(t, prev_pos, offset, scale)))
    return waypoints

def drawB(prev_pos, offset, rate, scale):
    t_B = 5600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_B, rate):
        waypoints = np.concatenate((waypoints, waypointB(t, prev_pos, offset, scale)))
    return waypoints

def drawC(prev_pos, offset, rate, scale):
    t_C = 4100
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_C, rate):
        waypoints = np.concatenate((waypoints, waypointC(t, prev_pos, offset, scale)))
    return waypoints

def drawD(prev_pos, offset, rate, scale):
    t_D = 3600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_D, rate):
        waypoints = np.concatenate((waypoints, waypointD(t, prev_pos, offset, scale)))
    return waypoints

def drawE(prev_pos, offset, rate, scale):
    t_E = 5200
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_E, rate):
        waypoints = np.concatenate((waypoints, waypointE(t, prev_pos, offset, scale)))
    return waypoints

def drawF(prev_pos, offset, rate, scale):
    t_F = 4200
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_F, rate):
        waypoints = np.concatenate((waypoints, waypointF(t, prev_pos, offset, scale)))
    return waypoints

def drawG(prev_pos, offset, rate, scale):
    t_G = 5100
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_G, rate):
        waypoints = np.concatenate((waypoints, waypointG(t, prev_pos, offset, scale)))
    return waypoints

def drawH(prev_pos, offset, rate, scale):
    t_H = 4800
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_H, rate):
        waypoints = np.concatenate((waypoints, waypointH(t, prev_pos, offset, scale)))
    return waypoints

def drawI(prev_pos, offset, rate, scale):
    t_I = 1600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_I, rate):
        waypoints = np.concatenate((waypoints, waypointI(t, prev_pos, offset, scale)))
    return waypoints
import numpy as np

def get_string_trajectory(string, start_pos=[0,0,0], fix_scale = -1, y_alinement = 'left'):

    # change the x and y limit of the canvas to get the right scale letters
    # the total length from y_min to y_max with 0 in the middle, in meters
    y_len = 30
    char_len = len(string)
    if fix_scale == -1:
        scale = (y_len/char_len)/4 # scale * 4 = actual char width
    else:
        scale = fix_scale

    char_y_len = scale * 4
    char_list = list(string)
    waypoints = np.array([[start_pos[0], start_pos[1], start_pos[2]]])
    if y_alinement == 'left':
        for i in range(0, char_len - 1):
            waypoints = np.concatenate((waypoints,
                                        get_alphabet_trajectory(char_list[i], list(waypoints[-1]),
                                                                [0, -(y_len/2) + i * char_y_len, 0], scale)))
        return waypoints
    elif y_alinement == 'right':
        for i in range(0, char_len - 1):
            waypoints = np.concatenate((waypoints,
                                        get_alphabet_trajectory(char_list[i], list(waypoints[-1]),
                                                                [0, (y_len/2-char_y_len*char_len)+i*char_y_len, 0],
                                                                scale)))
        return waypoints
    elif y_alinement == 'center':
        for i in range(0, char_len - 1):
            waypoints = np.concatenate((waypoints,
                                        get_alphabet_trajectory(char_list[i], list(waypoints[-1]),
                                                                [0, -(char_y_len*char_len/2)+i*char_y_len, 0], scale)))
        return waypoints








def get_alphabet_trajectory(alphabet, prev_pos=[0, 0, 0], offset=[0, 0, 0], scale=0.05, rate=25):
    '''
    Generate the waypoints for a given character
    :param alphabet: an English Capital letter charater
    :param prev_pos: the previous x, y, z position of the end effector
    :param offset: the x, y, z, formatting offset
    :param scale: to make the letter bigger (scale > 1) or smaller (0 < scale < 1)
    :param rate: the speed of drawing, the bigger the number, fewer waypoints. (should be less than 50)
    :return: n by 3 array of way points
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
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_A, rate):
        waypoints = np.concatenate((waypoints, waypointA(t, prev_pos, offset, scale)))
    return waypoints

def drawB(prev_pos, offset, rate, scale):
    t_B = 5600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_B, rate):
        waypoints = np.concatenate((waypoints, waypointB(t, prev_pos, offset, scale)))
    return waypoints

def drawC(prev_pos, offset, rate, scale):
    t_C = 4100
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_C, rate):
        waypoints = np.concatenate((waypoints, waypointC(t, prev_pos, offset, scale)))
    return waypoints

def drawD(prev_pos, offset, rate, scale):
    t_D = 3600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_D, rate):
        waypoints = np.concatenate((waypoints, waypointD(t, prev_pos, offset, scale)))
    return waypoints

def drawE(prev_pos, offset, rate, scale):
    t_E = 5200
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_E, rate):
        waypoints = np.concatenate((waypoints, waypointE(t, prev_pos, offset, scale)))
    return waypoints

def drawF(prev_pos, offset, rate, scale):
    t_F = 4200
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_F, rate):
        waypoints = np.concatenate((waypoints, waypointF(t, prev_pos, offset, scale)))
    return waypoints

def drawG(prev_pos, offset, rate, scale):
    t_G = 5100
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_G, rate):
        waypoints = np.concatenate((waypoints, waypointG(t, prev_pos, offset, scale)))
    return waypoints

def drawH(prev_pos, offset, rate, scale):
    t_H = 4800
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_H, rate):
        waypoints = np.concatenate((waypoints, waypointH(t, prev_pos, offset, scale)))
    return waypoints

def drawI(prev_pos, offset, rate, scale):
    t_I = 1600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_I, rate):
        waypoints = np.concatenate((waypoints, waypointI(t, prev_pos, offset, scale)))
    return waypoints

def drawJ(prev_pos, offset, rate, scale):
    t_J = 2600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_J, rate):
        waypoints = np.concatenate((waypoints, waypointJ(t, prev_pos, offset, scale)))
    return waypoints

def drawK(prev_pos, offset, rate, scale):
    t_K = 4200
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_K, rate):
        waypoints = np.concatenate((waypoints, waypointK(t, prev_pos, offset, scale)))
    return waypoints

def drawL(prev_pos, offset, rate, scale):
    t_L = 2600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_L, rate):
        waypoints = np.concatenate((waypoints, waypointL(t, prev_pos, offset, scale)))
    return waypoints

def drawM(prev_pos, offset, rate, scale):
    t_M = 4600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_M, rate):
        waypoints = np.concatenate((waypoints, waypointM(t, prev_pos, offset, scale)))
    return waypoints

def drawN(prev_pos, offset, rate, scale):
    t_N = 3600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_N, rate):
        waypoints = np.concatenate((waypoints, waypointN(t, prev_pos, offset, scale)))
    return waypoints

def drawO(prev_pos, offset, rate, scale):
    t_O = 4600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_O, rate):
        waypoints = np.concatenate((waypoints, waypointO(t, prev_pos, offset, scale)))
    return waypoints

def drawP(prev_pos, offset, rate, scale):
    t_P = 3600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_P, rate):
        waypoints = np.concatenate((waypoints, waypointP(t, prev_pos, offset, scale)))
    return waypoints

def drawQ(prev_pos, offset, rate, scale):
    t_Q = 5700
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_Q, rate):
        waypoints = np.concatenate((waypoints, waypointQ(t, prev_pos, offset, scale)))
    return waypoints

def drawR(prev_pos, offset, rate, scale):
    t_R = 2600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_R, rate):
        waypoints = np.concatenate((waypoints, waypointR(t, prev_pos, offset, scale)))
    return waypoints

def drawS(prev_pos, offset, rate, scale):
    t_S = 4850
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_S, rate):
        waypoints = np.concatenate((waypoints, waypointS(t, prev_pos, offset, scale)))
    return waypoints

def drawT(prev_pos, offset, rate, scale):
    t_T = 3200
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_T, rate):
        waypoints = np.concatenate((waypoints, waypointT(t, prev_pos, offset, scale)))
    return waypoints

def drawU(prev_pos, offset, rate, scale):
    t_U = 3850
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_U, rate):
        waypoints = np.concatenate((waypoints, waypointU(t, prev_pos, offset, scale)))
    return waypoints

def drawV(prev_pos, offset, rate, scale):
    t_V = 2600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_V, rate):
        waypoints = np.concatenate((waypoints, waypointV(t, prev_pos, offset, scale)))
    return waypoints

def drawW(prev_pos, offset, rate, scale):
    t_W = 4600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_W, rate):
        waypoints = np.concatenate((waypoints, waypointW(t, prev_pos, offset, scale)))
    return waypoints

def drawX(prev_pos, offset, rate, scale):
    t_X = 3200
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_X, rate):
        waypoints = np.concatenate((waypoints, waypointX(t, prev_pos, offset, scale)))
    return waypoints

def drawY(prev_pos, offset, rate, scale):
    t_Y = 4200
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_Y, rate):
        waypoints = np.concatenate((waypoints, waypointY(t, prev_pos, offset, scale)))
    return waypoints

def drawZ(prev_pos, offset, rate, scale):
    t_Z = 3600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_Z, rate):
        waypoints = np.concatenate((waypoints, waypointZ(t, prev_pos, offset, scale)))
    return waypoints


def waypointA(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 0
        y = offset[1] + 0
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + (0 + (4.0 - 0) / 1000 * (time - 550)) * scale
        y = offset[1] - (0 + (1.5 - 0) / 1000 * (time - 550)) * scale
        z = offset[2] + 0
    elif time < 2550:
        x = offset[0] + (4.0 + (0 - 4.0) / 1000 * (time - 1550)) * scale
        y = offset[1] - (1.5 + (3.0 - 1.5) / 1000 * (time - 1550)) * scale
        z = offset[2] + 0
    elif time < 2600:
        x = offset[0] + 0
        y = offset[1] - 3 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 2550)
    elif time < 3100:
        x = offset[0] + (0 + (2 - 0) / 500 * (time - 2600)) * scale
        y = offset[1] - (3 + (0.75 - 3) / 500 * (time - 2600)) * scale
        z = offset[2] + 0.05
    elif time < 3150:
        x = offset[0] + 2 * scale
        y = offset[1] - 0.75 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 3100)
    elif time < 3650:
        x = offset[0] + 2 * scale
        y = offset[1] - (0.75 + (2.25 - 0.75) / 500 * (time - 3150)) * scale
        z = offset[2] + 0
    elif time <= 3700:
        x = offset[0] + 2 * scale
        y = offset[1] - 2.25 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3650)
    return np.array([[x, y, z]])

def waypointB(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 0
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + (0 + (4 - 0) / 1000 * (time - 550)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2050:
        x = offset[0] + 4 * scale
        y = offset[1] - (0 + (1 - 0) / 500 * (time - 1550)) * scale
        z = offset[2] + 0
    elif time < 3050:
        x = offset[0] + (3 + np.cos((time - 2050) * np.pi / 1000) * 1) * scale
        y = offset[1] - (1 + np.sin((time - 2050) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 3550:
        x = offset[0] + 2 * scale
        y = offset[1] - (1 + (0 - 1) / 500 * (time - 3050)) * scale
        z = offset[2] + 0
    elif time < 4050:
        x = offset[0] + 2 * scale
        y = offset[1] - (0 + (1.5 - 0) / 500 * (time - 3550)) * scale
        z = offset[2] + 0
    elif time < 5050:
        x = offset[0] + (1 + np.cos((time - 4050) * np.pi / 1000) * 1) * scale
        y = offset[1] - (1.5 + np.sin((time - 4050) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 5550:
        x = offset[0] + 0
        y = offset[1] - (1.5 + (0 - 1.5) / 500 * (time - 5050)) * scale
        z = offset[2] + 0
    elif time <= 5600:
        x = offset[0] + 0
        y = offset[1] + 0
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 5550)
    return np.array([[x, y, z]])

def waypointC(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (3 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (-3 * scale + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 3 * scale
        y = offset[1] - 3 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1050:
        x = offset[0] + (3 + np.sin((time - 550) * np.pi / 1000) * 1) * scale
        y = offset[1] - (2 + np.cos((time - 550) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 1550:
        x = offset[0] + 4 * scale
        y = offset[1] - (2 + (1 - 2) / 500 * (time - 1050)) * scale
        z = offset[2] + 0
    elif time < 2050:
        x = offset[0] + (3 + np.cos((time - 1550) * np.pi / 1000) * 1) * scale
        y = offset[1] - (1 - np.sin((time - 1550) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 2550:
        x = offset[0] + (3 + (1 - 3) / 500 * (time - 2050)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 3050:
        x = offset[0] + (1 - np.cos((time - 2550) * np.pi / 1000) * 1) * scale
        y = offset[1] - (1 - np.sin((time - 2550) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 3550:
        x = offset[0] + 0
        y = offset[1] - (1 + (2 - 1) / 500 * (time - 3050)) * scale
        z = offset[2] + 0
    elif time < 4050:
        x = offset[0] + (1 - np.cos((time - 3550) * np.pi / 1000) * 1) * scale
        y = offset[1] - (2 + np.sin((time - 3550) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time <= 4100:
        x = offset[0] + 1 * scale
        y = offset[1] - 3 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 4050)
    return np.array([[x, y, z]])

def waypointD(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 0
        y = offset[1] + 0
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + (0 + (4 - 0) / 1000 * (time - 550)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 1800:
        x = offset[0] + 4 * scale
        y = offset[1] - (0 + (0.5 - 0) / 250 * (time - 1550)) * scale
        z = offset[2] + 0
    elif time < 3300:
        x = offset[0] + (2 + np.cos((time - 1800) * np.pi / 1500) * 2) * scale
        y = offset[1] - (0.5 + np.sin((time - 1800) * np.pi / 1500) * 2) * scale
        z = offset[2] + 0
    elif time < 3550:
        x = offset[0] + 0
        y = offset[1] - (0.5 + (0 - 0.5) / 250 * (time - 3300)) * scale
        z = offset[2] + 0
    elif time <= 3600:
        x = offset[0] + 0
        y = offset[1] + 0
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3550)
    return np.array([[x, y, z]])

def waypointE(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (-2.5 * scale + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 0
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + 0
        y = offset[1] - (2.5 + (0 - 2.5) / 1000 * (time - 550)) * scale
        z = offset[2] + 0
    elif time < 2550:
        x = offset[0] + (0 + (4 - 0) / 1000 * (time - 1550)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 3550:
        x = offset[0] + 4 * scale
        y = offset[1] - (0 + (2.5 - 0) / 1000 * (time - 2550)) * scale
        z = offset[2] + 0
    elif time < 3600:
        x = offset[0] + 4 * scale
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3550)
    elif time < 4100:
        x = offset[0] + (4 + (2 - 4) / 500 * (time - 3600)) * scale
        y = offset[1] - (2.5 + (0 - 2.5) / 500 * (time - 3600)) * scale
        z = offset[2] + 0.5
    elif time < 4150:
        x = offset[0] + 2 * scale
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 4100)
    elif time < 5150:
        x = offset[0] + 2 * scale
        y = offset[1] - (0 + (2 - 0) / 1000 * (time - 4150)) * scale
        z = offset[2] + 0
    elif time <= 5200:
        x = offset[0] + 2 * scale
        y = offset[1] - 2 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 5150)
    return np.array([[x, y, z]])

def waypointF(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 0
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + (0 + (4 - 0) / 1000 * (time - 550)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2550:
        x = offset[0] + 4 * scale
        y = offset[1] - (0 + (2.5 - 0) / 1000 * (time - 1550)) * scale
        z = offset[2] + 0
    elif time < 2600:
        x = offset[0] + 4 * scale
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 2550)
    elif time < 3100:
        x = offset[0] + (4 + (2 - 4) / 500 * (time - 2600)) * scale
        y = offset[1] - (2.5 + (0 - 2.5) / 500 * (time - 2600)) * scale
        z = offset[2] + 0.05
    elif time < 3150:
        x = offset[0] + 2 * scale
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 3100)
    elif time < 4150:
        x = offset[0] + 2 * scale
        y = offset[1] - (0 + (2 - 0) / 1000 * (time - 3150)) * scale
        z = offset[2] + 0
    elif time <= 4200:
        x = offset[0] + 2 * scale
        y = offset[1] - 2 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 4150)
    return np.array([[x, y, z]])

def waypointG(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (3 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (-3 * scale + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 3 * scale
        y = offset[1] - 3 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1050:
        x = offset[0] + (3 + np.sin((time - 550) * np.pi / 1000) * 1) * scale
        y = offset[1] - (2 + np.cos((time - 550) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 1550:
        x = offset[0] + 4 * scale
        y = offset[1] - (2 + (1 - 2) / 500 * (time - 1050)) * scale
        z = offset[2] + 0
    elif time < 2050:
        x = offset[0] + (3 + np.cos((time - 1550) * np.pi / 1000) * 1) * scale
        y = offset[1] - (1 - np.sin((time - 1550) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 2550:
        x = offset[0] + (3 + (1 - 3) / 500 * (time - 2050)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 3050:
        x = offset[0] + (1 - np.cos((time - 2550) * np.pi / 1000) * 1) * scale
        y = offset[1] - (1 - np.sin((time - 2550) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 3550:
        x = offset[0] + 0
        y = offset[1] - (1 + (2 - 1) / 500 * (time - 3050)) * scale
        z = offset[2] + 0
    elif time < 4050:
        x = offset[0] + (1 - np.cos((time - 3550) * np.pi / 1000) * 1) * scale
        y = offset[1] - (2 + np.sin((time - 3550) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 4550:
        x = offset[0] + (1 + (2 - 1) / 500 * (time - 4050)) * scale
        y = offset[1] - 3 * scale
        z = offset[2] + 0
    elif time < 5050:
        x = offset[0] + 2 * scale
        y = offset[1] - (3 + (2 - 3) / 500 * (time - 4550)) * scale
        z = offset[2] + 0
    elif time <= 5100:
        x = offset[0] + 2 * scale
        y = offset[1] - 2 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 5050)
    return np.array([[x, y, z]])

def waypointH(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 0
        y = offset[1] + 0
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + (0 + (4 - 0) / 1000 * (time - 550)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 1600:
        x = offset[0] + 4 * scale
        y = offset[1] - 0
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 1550)
    elif time < 2100:
        x = offset[0] + (4 + (2 - 4) / 500 * (time - 1600)) * scale
        y = offset[1] - 0
        z = offset[2] + 0.05
    elif time < 2150:
        x = offset[0] + 2 * scale
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 2100)
    elif time < 3150:
        x = offset[0] + 2 * scale
        y = offset[1] - (0 + (2.5 - 0) / 1000 * (time - 2150)) * scale
        z = offset[2] + 0
    elif time < 3200:
        x = offset[0] + 2 * scale
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3150)
    elif time < 3700:
        x = offset[0] + (2 + (4 - 2) / 500 * (time - 3200)) * scale
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0.05
    elif time < 3750:
        x = offset[0] + 4 * scale
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 3700)
    elif time < 4750:
        x = offset[0] + (4 + (0 - 4) / 1000 * (time - 3750)) * scale
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0
    elif time <= 4800:
        x = offset[0] + 0
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 4750)
    return np.array([[x, y, z]])

def waypointI(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (4.0 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (-1.5 * scale + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 4 * scale
        y = offset[1] - 1.5 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + (4 + (0 - 4.0) / 1000 * (time - 550)) * scale
        y = offset[1] - 1.5 * scale
        z = offset[2] + 0
    elif time <= 1600:
        x = offset[0] + 0 * scale
        y = offset[1] - 1.5 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 1550)
    return np.array([[x, y, z]])


def waypointJ(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (4 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (-2 * scale + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 4 * scale
        y = offset[1] - 2 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + (4 + (1 - 4) / 1000 * (time - 550)) * scale
        y = offset[1] - 2 * scale
        z = offset[2] + 0
    elif time < 2550:
        x = offset[0] + (1 - np.sin((time - 1550) * np.pi / 1000) * 1) * scale
        y = offset[1] - (1 + np.cos((time - 1550) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time <= 2600:
        x = offset[0] + 1 * scale
        y = offset[1] - 0
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 2550)
    return np.array([[x, y, z]])


def waypointK(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 0
        y = offset[1] + 0
        z = offset[2] + 0.5 + (0 - 0.5) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + (0 + (4 - 0) / 1000 * (time - 550)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 1600:
        x = offset[0] + 4 * scale
        y = offset[1] - 0
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 1550)
    elif time < 2100:
        x = offset[0] + 4 * scale
        y = offset[1] - (0 + (2 - 0) / 500 * (time - 1600)) * scale
        z = offset[2] + 0.05
    elif time < 2150:
        x = offset[0] + 4 * scale
        y = offset[1] - 2 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 2100)
    elif time < 3150:
        x = offset[0] + (4 + (2 - 4) / 1000 * (time - 2150)) * scale
        y = offset[1] - (2 + (0 - 2) / 1000 * (time - 2150)) * scale
        z = offset[2] + 0
    elif time < 4150:
        x = offset[0] + (2 + (0 - 2) / 1000 * (time - 3150)) * scale
        y = offset[1] - (0 + (2.5 - 0) / 1000 * (time - 3150)) * scale
        z = offset[2] + 0
    elif time <= 4200:
        x = offset[0] + 0
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 4150)
    return np.array([[x, y, z]])



def waypointL(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (4 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 4 * scale
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + (4 + (0 - 4) / 1000 * (time - 550)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2550:
        x = offset[0] + 0
        y = offset[1] - (0 + (2 - 0) / 1000 * (time - 1550)) * scale
        z = offset[2] + 0
    elif time <= 2600:
        x = offset[0] + 0
        y = offset[1] - 2 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 2550)
    return np.array([[x, y, z]])




def waypointM(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 0
        y = offset[1] + 0
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + (0 + (4 - 0) / 1000 * (time - 550)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2550:
        x = offset[0] + (4 + (2 - 4) / 1000 * (time - 1550)) * scale
        y = offset[1] - (0 + (1.5 - 0) / 1000 * (time - 1550)) * scale
        z = offset[2] + 0
    elif time < 3550:
        x = offset[0] + (2 + (4 - 2) / 1000 * (time - 2550)) * scale
        y = offset[1] - (1.5 + (3 - 1.5) / 1000 * (time - 2550)) * scale
        z = offset[2] + 0
    elif time < 4550:
        x = offset[0] + (4 + (0 - 4) / 1000 * (time - 3550)) * scale
        y = offset[1] - 3 * scale
        z = offset[2] + 0
    elif time <= 4600:
        x = offset[0] + 0
        y = offset[1] - 3 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 4550)
    return np.array([[x, y, z]])

def waypointN(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 0
        y = offset[1] + 0
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + (0 + (4 - 0) / 1000 * (time - 550)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2550:
        x = offset[0] + (4 + (0 - 4) / 1000 * (time - 1550)) * scale
        y = offset[1] - (0 + (3 - 0) / 1000 * (time - 1550)) * scale
        z = offset[2] + 0
    elif time < 3550:
        x = offset[0] + (0 + (4 - 0) / 1000 * (time - 2550)) * scale
        y = offset[1] - 3 * scale
        z = offset[2] + 0
    elif time <= 3600:
        x = offset[0] + 4 * scale
        y = offset[1] - 3 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3550)
    return np.array([[x, y, z]])

def waypointO(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (-1 * scale + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 0
        y = offset[1] - 1 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1050:
        x = offset[0] + (1 - np.sin((time - 550) * np.pi / 1000) * 1) * scale
        y = offset[1] - (1 - np.cos((time - 550) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 1550:
        x = offset[0] + (1 + (3 - 1) / 500 * (time - 1050)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2050:
        x = offset[0] + (3 + np.cos((time - 1550) * np.pi / 1000) * 1) * scale
        y = offset[1] - (1 - np.sin((time - 1550) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 2550:
        x = offset[0] + 4 * scale
        y = offset[1] - (1 + (2 - 1) / 500 * (time - 2050)) * scale
        z = offset[2] + 0
    elif time < 3050:
        x = offset[0] + (3 + np.cos((time - 2550) * np.pi / 1000) * 1) * scale
        y = offset[1] - (2 + np.sin((time - 2550) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 3550:
        x = offset[0] + (3 + (1 - 3) / 500 * (time - 3050)) * scale
        y = offset[1] - 3 * scale
        z = offset[2] + 0
    elif time < 4050:
        x = offset[0] + (1 - np.cos((time - 3550) * np.pi / 1000) * 1) * scale
        y = offset[1] - (2 + np.sin((time - 3550) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 4550:
        x = offset[0] + 0
        y = offset[1] - (2 + (1 - 2) / 500 * (time - 4050)) * scale
        z = offset[2] + 0
    elif time <= 4600:
        x = offset[0] + 0
        y = offset[1] - 1 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 4550)
    return np.array([[x, y, z]])

def waypointP(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 0
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + (0 + (4 - 0) / 1000 * (time - 550)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2050:
        x = offset[0] + 4 * scale
        y = offset[1] - (0 + (1.5 - 0) / 500 * (time - 1550)) * scale
        z = offset[2] + 0
    elif time < 3050:
        x = offset[0] + (3 + np.cos((time - 2050) * np.pi / 1000) * 1) * scale
        y = offset[1] - (1.5 + np.sin((time - 2050) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 3550:
        x = offset[0] + 2 * scale
        y = offset[1] - (1.5 + (0 - 1.5) / 500 * (time - 3050)) * scale
        z = offset[2] + 0
    elif time <= 3600:
        x = offset[0] + 2 * scale
        y = offset[1] - 0
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3550)
    return np.array([[x, y, z]])

def waypointQ(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (-1 * scale + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 0
        y = offset[1] - 1 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1050:
        x = offset[0] + (1 - np.sin((time - 550) * np.pi / 1000) * 1) * scale
        y = offset[1] - (1 - np.cos((time - 550) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 1550:
        x = offset[0] + (1 + (3 - 1) / 500 * (time - 1050)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2050:
        x = offset[0] + (3 + np.cos((time - 1550) * np.pi / 1000) * 1) * scale
        y = offset[1] - (1 - np.sin((time - 1550) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 2550:
        x = offset[0] + 4 * scale
        y = offset[1] - (1 + (2 - 1) / 500 * (time - 2050)) * scale
        z = offset[2] + 0
    elif time < 3050:
        x = offset[0] + (3 + np.cos((time - 2550) * np.pi / 1000) * 1) * scale
        y = offset[1] - (2 + np.sin((time - 2550) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 3550:
        x = offset[0] + (3 + (1 - 3) / 500 * (time - 3050)) * scale
        y = offset[1] - 3 * scale
        z = offset[2] + 0
    elif time < 4050:
        x = offset[0] + (1 - np.cos((time - 3550) * np.pi / 1000) * 1) * scale
        y = offset[1] - (2 + np.sin((time - 3550) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 4550:
        x = offset[0] + 0
        y = offset[1] - (2 + (1 - 2) / 500 * (time - 4050)) * scale
        z = offset[2] + 0
    elif time < 4600:
        x = offset[0] + 0
        y = offset[1] - 1 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 4550)
    elif time < 5100:
        x = offset[0] + (0 + (0.75 - 0)/500 * (time - 4600)) * scale
        y = offset[1] - (1 + (2.25 - 1)/500 * (time - 4600)) * scale
        z = offset[2] + 0.05
    elif time < 5150:
        x = offset[0] + 0.75 * scale
        y = offset[1] - 2.25 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 5100)
    elif time < 5650:
        x = offset[0] + (0.75 + (0 - 0.75)/500 * (time - 5150)) * scale
        y = offset[1] - (2.25 + (3 - 2.25)/500 * (time - 5150)) * scale
        z = offset[2] + 0
    elif time <= 5700:
        x = offset[0] + 0
        y = offset[1] - 3 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3550)
    return np.array([[x, y, z]])

def waypointR(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 0
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + (0 + (4 - 0) / 1000 * (time - 550)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2050:
        x = offset[0] + 4 * scale
        y = offset[1] - (0 + (1.5 - 0) / 500 * (time - 1550)) * scale
        z = offset[2] + 0
    elif time < 3050:
        x = offset[0] + (3 + np.cos((time - 2050) * np.pi / 1000) * 1) * scale
        y = offset[1] - (1.5 + np.sin((time - 2050) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 3550:
        x = offset[0] + 2 * scale
        y = offset[1] - (1.5 + (0 - 1.5) / 500 * (time - 3050)) * scale
        z = offset[2] + 0
    elif time < 3600:
        x = offset[0] + 2 * scale
        y = offset[1] - 0
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3550)
    elif time < 4100:
        x = offset[0] + 2 * scale
        y = offset[1] - (0 + (1 - 0)/500 * (time - 3600)) * scale
        z = offset[2] + 0.05
    elif time < 4150:
        x = offset[0] + 2 * scale
        y = offset[1] - 1 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 5100)
    elif time < 5150:
        x = offset[0] + (2 + (0 - 2)/1000 * (time - 4150)) * scale
        y = offset[1] - (1 + (3 - 1)/1000 * (time - 4150)) * scale
        z = offset[2] + 0
    elif time <= 5200:
        x = offset[0] + 0
        y = offset[1] - 3 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3550)
    return np.array([[x, y, z]])

def waypointS(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (3.707 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (-2.707 * scale + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 3.707 * scale
        y = offset[1] - 2.707 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 800:
        x = offset[0] + (3 + np.sin(np.pi / 4 + (time - 550) * np.pi / 1000) * 1) * scale
        y = offset[1] - (2 + np.cos(np.pi / 4 + (time - 550) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 1300:
        x = offset[0] + 4 * scale
        y = offset[1] - (2 + (1 - 2) / 500 * (time - 800)) * scale
        z = offset[2] + 0
    elif time < 2300:
        x = offset[0] + (3 + np.cos((time - 1300) * np.pi / 1000) * 1) * scale
        y = offset[1] - (1 - np.sin((time - 1300) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 2800:
        x = offset[0] + 2 * scale
        y = offset[1] - (1 + (2 - 1) / 500 * (time - 2300)) * scale
        z = offset[2] + 0
    elif time < 3800:
        x = offset[0] + (1 + np.cos((time - 2800) * np.pi / 1000) * 1) * scale
        y = offset[1] - (2 + np.sin((time - 2800) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 4300:
        x = offset[0] + 0
        y = offset[1] - (2 + (1 - 2) / 500 * (time - 3800)) * scale
        z = offset[2] + 0
    elif time < 4800:
        x = offset[0] + (1 - np.cos((time - 4300) * np.pi / 1000) * 1) * scale
        y = offset[1] - (1 - np.sin((time - 4300) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time <= 4850:
        x = offset[0] + 1 * scale
        y = offset[1] - 0
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 4800)
    return np.array([[x, y, z]])

def waypointT(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (4 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 4 * scale
        y = offset[1] + 0
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + 4 * scale
        y = offset[1] - (0 + (3 - 0) / 1000 * (time - 550)) * scale
        z = offset[2] + 0
    elif time < 1600:
        x = offset[0] + 4 * scale
        y = offset[1] - 3 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 1550)
    elif time < 2100:
        x = offset[0] + 4 * scale
        y = offset[1] - (3 + (1.5 - 3) / 500 * (time - 1600)) * scale
        z = offset[2] + 0.05
    elif time < 2150:
        x = offset[0] + 4 * scale
        y = offset[1] - 1.5 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 2100)
    elif time < 3150:
        x = offset[0] + (4 + (0 - 4) / 1000 * (time - 2150)) * scale
        y = offset[1] - 1.5 * scale
        z = offset[2] + 0
    elif time <= 3200:
        x = offset[0] + 0
        y = offset[1] - 1.5 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3150)
    return np.array([[x, y, z]])

def waypointU(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (4 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 4 * scale
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + (4 + (1 - 4) / 1000 * (time - 550)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2050:
        x = offset[0] + (1 - np.sin((time - 1550) * np.pi / 1000) * 1) * scale
        y = offset[1] - (1 - np.cos((time - 1550) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 2300:
        x = offset[0] + 0
        y = offset[1] - (1 + (1.5 - 1) / 250 * (time - 2050)) * scale
        z = offset[2] + 0
    elif time < 2800:
        x = offset[0] + (1 - np.cos((time - 2300) * np.pi / 1000) * 1) * scale
        y = offset[1] - (1.5 + np.sin((time - 2300) * np.pi / 1000) * 1) * scale
        z = offset[2] + 0
    elif time < 3800:
        x = offset[0] + (1 + (4 - 1) / 1000 * (time - 2800)) * scale
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0
    elif time <= 3850:
        x = offset[0] + 4 * scale
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3800)
    return np.array([[x, y, z]])

def waypointV(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (4 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 4 * scale
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + (4 + (0 - 4) / 1000 * (time - 550)) * scale
        y = offset[1] - (0 + (1.5 - 0) / 1000 * (time - 550)) * scale
        z = offset[2] + 0
    elif time < 2550:
        x = offset[0] + (0 + (4 - 0) / 1000 * (time - 1550)) * scale
        y = offset[1] - (1.5 + (3 - 1.5) / 1000 * (time - 1550)) * scale
        z = offset[2] + 0
    elif time <= 2600:
        x = offset[0] + 4 * scale
        y = offset[1] - 3 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 2550)
    return np.array([[x, y, z]])

def waypointW(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (4 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 4 * scale
        y = offset[1] + 0
        z = offset[2] + 0.5 + (0 - 0.5) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + (4 + (0 - 4) / 1000 * (time - 550)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2550:
        x = offset[0] + (0 + (2 - 0) / 1000 * (time - 1550)) * scale
        y = offset[1] - (0 + (1.5 - 0) / 1000 * (time - 1550)) * scale
        z = offset[2] + 0
    elif time < 3550:
        x = offset[0] + (2 + (0 - 2) / 1000 * (time - 2550)) * scale
        y = offset[1] - (1.5 + (3 - 1.5) / 1000 * (time - 2550)) * scale
        z = offset[2] + 0
    elif time < 4550:
        x = offset[0] + (0 + (4 - 0) / 1000 * (time - 3550)) * scale
        y = offset[1] - 3 * scale
        z = offset[2] + 0
    elif time <= 4600:
        x = offset[0] + 4 * scale
        y = offset[1] - 3 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 4550)
    return np.array([[x, y, z]])

def waypointX(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 0
        y = offset[1] + 0
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + (0 + (4 - 0) / 1000 * (time - 550)) * scale
        y = offset[1] - (0 + (3 - 0) / 1000 * (time - 550)) * scale
        z = offset[2] + 0
    elif time < 1600:
        x = offset[0] + 4 * scale
        y = offset[1] - 3 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 1550)
    elif time < 2100:
        x = offset[0] + (4 + (0 - 4) / 500 * (time - 1600)) * scale
        y = offset[1] - 3 * scale
        z = offset[2] + 0.05
    elif time < 2150:
        x = offset[0] + 0
        y = offset[1] - 3 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 2100)
    elif time < 3150:
        x = offset[0] + (0 + (4 - 0) / 1000 * (time - 2150)) * scale
        y = offset[1] - (3 + (0 - 3) / 1000 * (time - 2150)) * scale
        z = offset[2] + 0
    elif time <= 3200:
        x = offset[0] + 4 * scale
        y = offset[1] - 0
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3150)
    return np.array([[x, y, z]])

def waypointY(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (4 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 4 * scale
        y = offset[1] + 0
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + (4 + (2 - 4) / 1000 * (time - 550)) * scale
        y = offset[1] - (0 + (1.5 - 0) / 1000 * (time - 550)) * scale
        z = offset[2] + 0
    elif time < 2550:
        x = offset[0] + (2 + (4 - 2) / 1000 * (time - 1550)) * scale
        y = offset[1] - (1.5 + (3 - 1.5) / 1000 * (time - 1550)) * scale
        z = offset[2] + 0
    elif time < 2600:
        x = offset[0] + 4 * scale
        y = offset[1] - 3 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 2550)
    elif time < 3100:
        x = offset[0] + (4 + (2 - 4) / 500 * (time - 2600)) * scale
        y = offset[1] - (3 + (1.5 - 3) / 500 * (time - 2600)) * scale
        z = offset[2] + 0.05
    elif time < 3150:
        x = offset[0] + 2 * scale
        y = offset[1] - 1.5 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 3100)
    elif time < 4150:
        x = offset[0] + (2 + (0 - 2) / 1000 * (time - 3150)) * scale
        y = offset[1] - 1.5 * scale
        z = offset[2] + 0
    elif time <= 4200:
        x = offset[0] + 0
        y = offset[1] - 1.5 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 4150)
    return np.array([[x, y, z]])

def waypointZ(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (4 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 550:
        x = offset[0] + 4 * scale
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 50 * (time - 500)
    elif time < 1550:
        x = offset[0] + 4 * scale
        y = offset[1] - (0 + (2 - 0) / 1000 * (time - 550)) * scale
        z = offset[2] + 0
    elif time < 2550:
        x = offset[0] + (4 + (0 - 4) / 1000 * (time - 1550)) * scale
        y = offset[1] - (2 + (0 - 2) / 1000 * (time - 1550)) * scale
        z = offset[2] + 0
    elif time < 3550:
        x = offset[0] + 0
        y = offset[1] - (0 + (3 - 0) / 1000 * (time - 2550)) * scale
        z = offset[2] + 0
    elif time <= 3600:
        x = offset[0] + 0
        y = offset[1] - 3 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3550)
    return np.array([[x, y, z]])

