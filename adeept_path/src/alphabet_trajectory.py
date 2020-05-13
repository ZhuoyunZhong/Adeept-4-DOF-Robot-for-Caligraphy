import numpy as np


def get_string_trajectory(string, start_pos=[0,0,0], offset=[0.11, 0, 0], 
                          fix_scale=-1, y_alinement = 'center'):
    # change the x and y limit of the canvas to get the right scale letters
    # the total length from y_min to y_max with 0 in the middle, in meters
    y_len = 0.08
    char_len = len(string)
    if fix_scale == -1:
        scale = (y_len/char_len) 
    else:
        scale = fix_scale
    # Maximum scale
    if scale > 0.015:
        scale = 0.015

    # scale * 4.0 = actual char width
    char_y_len = scale * 4
    char_list = list(string)
    waypoints = np.array([[start_pos[0], start_pos[1], start_pos[2]]])
    if y_alinement == 'left':
        for i in range(char_len):
            waypoints = np.concatenate((waypoints,
                get_alphabet_trajectory(char_list[i], list(waypoints[-1]),
                    [offset[0], (y_len*4/2.0) - i*char_y_len, offset[2]], scale)))
        return waypoints
    elif y_alinement == 'right':
        for i in range(char_len):
            waypoints = np.concatenate((waypoints,
                get_alphabet_trajectory(char_list[i], list(waypoints[-1]),
                    [offset[0], -(y_len*4/2.0-char_y_len*char_len) - i*char_y_len, offset[2]], scale)))
        return waypoints
    elif y_alinement == 'center':
        for i in range(char_len):
            waypoints = np.concatenate((waypoints,
                get_alphabet_trajectory(char_list[i], list(waypoints[-1]),
                    [offset[0], (char_y_len*char_len/2.0) - i*char_y_len, offset[2]], scale)))
        return waypoints


def get_alphabet_trajectory(alphabet, prev_pos=[0, 0, 0], offset=[0, 0, 0], scale=0.05, rate=25):
    '''
    Generate the waypoints for a given character
    :param alphabet: an English Capital letter charater
    :param prev_pos: the previous x, y, z position of the end effector
    :param offset: the x, y, z, formatting offset
    :param scale: to make the letter bigger (scale > 1.0) or smaller (0 < scale < 1.0)
    :param rate: the speed of drawing, the bigger the number, fewer waypoints. (should be less than 50)
    :return: n by 3.0 array of way points
    '''
    if alphabet == 'A':
        return drawA(prev_pos, offset, rate, scale)
    elif alphabet == 'B':
        return drawB(prev_pos, offset, rate, scale)
    elif alphabet == 'C':
        return drawC(prev_pos, offset, rate, scale)
    elif alphabet == 'D':
        return drawD(prev_pos, offset, rate, scale)
    elif alphabet == 'E':
        return drawE(prev_pos, offset, rate, scale)
    elif alphabet == 'F':
        return drawF(prev_pos, offset, rate, scale)
    elif alphabet == 'G':
        return drawG(prev_pos, offset, rate, scale)
    elif alphabet == 'H':
        return drawH(prev_pos, offset, rate, scale)
    elif alphabet == 'I':
        return drawI(prev_pos, offset, rate, scale)
    elif alphabet == 'J':
        return drawJ(prev_pos, offset, rate, scale)
    elif alphabet == 'K':
        return drawK(prev_pos, offset, rate, scale)
    elif alphabet == 'L':
        return drawL(prev_pos, offset, rate, scale)
    elif alphabet == 'M':
        return drawM(prev_pos, offset, rate, scale)
    elif alphabet == 'N':
        return drawN(prev_pos, offset, rate, scale)
    elif alphabet == 'O':
        return drawO(prev_pos, offset, rate, scale)
    elif alphabet == 'P':
        return drawP(prev_pos, offset, rate, scale)
    elif alphabet == 'Q':
        return drawQ(prev_pos, offset, rate, scale)
    elif alphabet == 'R':
        return drawR(prev_pos, offset, rate, scale)
    elif alphabet == 'S':
        return drawS(prev_pos, offset, rate, scale)
    elif alphabet == 'T':
        return drawT(prev_pos, offset, rate, scale)
    elif alphabet == 'U':
        return drawU(prev_pos, offset, rate, scale)
    elif alphabet == 'V':
        return drawV(prev_pos, offset, rate, scale)
    elif alphabet == 'W':
        return drawW(prev_pos, offset, rate, scale)
    elif alphabet == 'X':
        return drawX(prev_pos, offset, rate, scale)
    elif alphabet == 'Y':
        return drawY(prev_pos, offset, rate, scale)
    elif alphabet == 'Z':
        return drawZ(prev_pos, offset, rate, scale)


def drawA(prev_pos, offset, rate, scale):
    t_A = 4100
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_A, rate):
        waypoints = np.concatenate((waypoints, waypointA(t, prev_pos, offset, scale)))
    return waypoints

def drawB(prev_pos, offset, rate, scale):
    t_B = 5800
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_B, rate):
        waypoints = np.concatenate((waypoints, waypointB(t, prev_pos, offset, scale)))
    return waypoints

def drawC(prev_pos, offset, rate, scale):
    t_C = 4300
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_C, rate):
        waypoints = np.concatenate((waypoints, waypointC(t, prev_pos, offset, scale)))
    return waypoints

def drawD(prev_pos, offset, rate, scale):
    t_D = 3800
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_D, rate):
        waypoints = np.concatenate((waypoints, waypointD(t, prev_pos, offset, scale)))
    return waypoints

def drawE(prev_pos, offset, rate, scale):
    t_E = 5600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_E, rate):
        waypoints = np.concatenate((waypoints, waypointE(t, prev_pos, offset, scale)))
    return waypoints

def drawF(prev_pos, offset, rate, scale):
    t_F = 4600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_F, rate):
        waypoints = np.concatenate((waypoints, waypointF(t, prev_pos, offset, scale)))
    return waypoints

def drawG(prev_pos, offset, rate, scale):
    t_G = 5300
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_G, rate):
        waypoints = np.concatenate((waypoints, waypointG(t, prev_pos, offset, scale)))
    return waypoints

def drawH(prev_pos, offset, rate, scale):
    t_H = 5400
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_H, rate):
        waypoints = np.concatenate((waypoints, waypointH(t, prev_pos, offset, scale)))
    return waypoints

def drawI(prev_pos, offset, rate, scale):
    t_I = 1800
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_I, rate):
        waypoints = np.concatenate((waypoints, waypointI(t, prev_pos, offset, scale)))
    return waypoints

def drawJ(prev_pos, offset, rate, scale):
    t_J = 2800
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_J, rate):
        waypoints = np.concatenate((waypoints, waypointJ(t, prev_pos, offset, scale)))
    return waypoints

def drawK(prev_pos, offset, rate, scale):
    t_K = 4600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_K, rate):
        waypoints = np.concatenate((waypoints, waypointK(t, prev_pos, offset, scale)))
    return waypoints

def drawL(prev_pos, offset, rate, scale):
    t_L = 2800
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_L, rate):
        waypoints = np.concatenate((waypoints, waypointL(t, prev_pos, offset, scale)))
    return waypoints

def drawM(prev_pos, offset, rate, scale):
    t_M = 4800
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_M, rate):
        waypoints = np.concatenate((waypoints, waypointM(t, prev_pos, offset, scale)))
    return waypoints

def drawN(prev_pos, offset, rate, scale):
    t_N = 3800
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_N, rate):
        waypoints = np.concatenate((waypoints, waypointN(t, prev_pos, offset, scale)))
    return waypoints

def drawO(prev_pos, offset, rate, scale):
    t_O = 4800
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_O, rate):
        waypoints = np.concatenate((waypoints, waypointO(t, prev_pos, offset, scale)))
    return waypoints

def drawP(prev_pos, offset, rate, scale):
    t_P = 3800
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_P, rate):
        waypoints = np.concatenate((waypoints, waypointP(t, prev_pos, offset, scale)))
    return waypoints

def drawQ(prev_pos, offset, rate, scale):
    t_Q = 6100
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_Q, rate):
        waypoints = np.concatenate((waypoints, waypointQ(t, prev_pos, offset, scale)))
    return waypoints

def drawR(prev_pos, offset, rate, scale):
    t_R = 5600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_R, rate):
        waypoints = np.concatenate((waypoints, waypointR(t, prev_pos, offset, scale)))
    return waypoints

def drawS(prev_pos, offset, rate, scale):
    t_S = 5050
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_S, rate):
        waypoints = np.concatenate((waypoints, waypointS(t, prev_pos, offset, scale)))
    return waypoints

def drawT(prev_pos, offset, rate, scale):
    t_T = 3600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_T, rate):
        waypoints = np.concatenate((waypoints, waypointT(t, prev_pos, offset, scale)))
    return waypoints

def drawU(prev_pos, offset, rate, scale):
    t_U = 4050
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_U, rate):
        waypoints = np.concatenate((waypoints, waypointU(t, prev_pos, offset, scale)))
    return waypoints

def drawV(prev_pos, offset, rate, scale):
    t_V = 2800
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_V, rate):
        waypoints = np.concatenate((waypoints, waypointV(t, prev_pos, offset, scale)))
    return waypoints

def drawW(prev_pos, offset, rate, scale):
    t_W = 4800
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_W, rate):
        waypoints = np.concatenate((waypoints, waypointW(t, prev_pos, offset, scale)))
    return waypoints

def drawX(prev_pos, offset, rate, scale):
    t_X = 3600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_X, rate):
        waypoints = np.concatenate((waypoints, waypointX(t, prev_pos, offset, scale)))
    return waypoints

def drawY(prev_pos, offset, rate, scale):
    t_Y = 4600
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_Y, rate):
        waypoints = np.concatenate((waypoints, waypointY(t, prev_pos, offset, scale)))
    return waypoints

def drawZ(prev_pos, offset, rate, scale):
    t_Z = 3800
    waypoints = np.array([[prev_pos[0], prev_pos[1], prev_pos[2]]])
    for t in range(0, t_Z, rate):
        waypoints = np.concatenate((waypoints, waypointZ(t, prev_pos, offset, scale)))
    return waypoints


def waypointA(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 0
        y = offset[1] + 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + (0 + (4.0 - 0) / 1000 * (time - 750)) * scale
        y = offset[1] - (0 + (1.5 - 0) / 1000 * (time - 750)) * scale
        z = offset[2] + 0
    elif time < 2750:
        x = offset[0] + (4.0 + (0 - 4.0) / 1000 * (time - 1750)) * scale
        y = offset[1] - (1.5 + (3.0 - 1.5) / 1000 * (time - 1750)) * scale
        z = offset[2] + 0
    elif time < 2800:
        x = offset[0] + 0
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 2750)
    elif time < 3300:
        x = offset[0] + (0 + (2.0 - 0) / 500 * (time - 2800)) * scale
        y = offset[1] - (3.0 + (0.75 - 3.0) / 500 * (time - 2800)) * scale
        z = offset[2] + 0.05
    elif time < 3550:
        x = offset[0] + 2.0 * scale
        y = offset[1] - 0.75 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 3300)
    elif time < 4050:
        x = offset[0] + 2.0 * scale
<<<<<<< HEAD
        y = offset[1] - (0.9 + (2.25 - 0.75) / 500 * (time - 3150)) * scale
=======
        y = offset[1] - (0.75 + (2.25 - 0.75) / 500 * (time - 3550)) * scale
>>>>>>> 679a911e6e0224090c99e1a89996aebb8e4d49d3
        z = offset[2] + 0
    elif time <= 4100:
        x = offset[0] + 2.0 * scale
        y = offset[1] - 2.25 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 4050)
    return np.array([[x, y, z]])

def waypointB(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 0
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + (0 + (4.0 - 0) / 1000 * (time - 750)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2250:
        x = offset[0] + 4.0 * scale
        y = offset[1] - (0 + (1.0 - 0) / 500 * (time - 1750)) * scale
        z = offset[2] + 0
    elif time < 3250:
        x = offset[0] + (3.0 + np.cos((time - 2250) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (1.0 + np.sin((time - 2250) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 3750:
        x = offset[0] + 2.0 * scale
        y = offset[1] - (1.0 + (0 - 1.0) / 500 * (time - 3250)) * scale
        z = offset[2] + 0
    elif time < 4250:
        x = offset[0] + 2.0 * scale
        y = offset[1] - (0 + (1.5 - 0) / 500 * (time - 3750)) * scale
        z = offset[2] + 0
    elif time < 5250:
        x = offset[0] + (1.0 + np.cos((time - 4250) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (1.5 + np.sin((time - 4250) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 5750:
        x = offset[0] + 0
        y = offset[1] - (1.5 + (0 - 1.5) / 500 * (time - 5250)) * scale
        z = offset[2] + 0
    elif time <= 5800:
        x = offset[0] + 0
        y = offset[1] + 0
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 5750)
    return np.array([[x, y, z]])

def waypointC(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (3.0 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (-3.0 * scale + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 3.0 * scale
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1250:
        x = offset[0] + (3.0 + np.sin((time - 750) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (2.0 + np.cos((time - 750) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 1750:
        x = offset[0] + 4.0 * scale
        y = offset[1] - (2.0 + (1.0 - 2.0) / 500 * (time - 1250)) * scale
        z = offset[2] + 0
    elif time < 2250:
        x = offset[0] + (3.0 + np.cos((time - 1750) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (1.0 - np.sin((time - 1750) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 2750:
        x = offset[0] + (3.0 + (1.0 - 3.0) / 500 * (time - 2250)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 3250:
        x = offset[0] + (1.0 - np.cos((time - 2750) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (1.0 - np.sin((time - 2750) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 3750:
        x = offset[0] + 0
        y = offset[1] - (1.0 + (2.0 - 1.0) / 500 * (time - 3250)) * scale
        z = offset[2] + 0
    elif time < 4250:
        x = offset[0] + (1.0 - np.cos((time - 3750) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (2.0 + np.sin((time - 3750) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time <= 4300:
        x = offset[0] + 1.0 * scale
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 4250)
    return np.array([[x, y, z]])

def waypointD(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 0
        y = offset[1] + 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + (0 + (4.0 - 0) / 1000 * (time - 750)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2000:
        x = offset[0] + 4.0 * scale
        y = offset[1] - (0 + (0.5 - 0) / 250 * (time - 1750)) * scale
        z = offset[2] + 0
    elif time < 3500:
        x = offset[0] + (2.0 + np.cos((time - 2000) * np.pi / 1500) * 2.0) * scale
        y = offset[1] - (0.5 + np.sin((time - 2000) * np.pi / 1500) * 2.0) * scale
        z = offset[2] + 0
    elif time < 3750:
        x = offset[0] + 0
        y = offset[1] - (0.5 + (0 - 0.5) / 250 * (time - 3500)) * scale
        z = offset[2] + 0
    elif time <= 3800:
        x = offset[0] + 0
        y = offset[1] + 0
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3750)
    return np.array([[x, y, z]])

def waypointE(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (-2.5 * scale + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 0
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + 0
        y = offset[1] - (2.5 + (0 - 2.5) / 1000 * (time - 750)) * scale
        z = offset[2] + 0
    elif time < 2750:
        x = offset[0] + (0 + (4.0 - 0) / 1000 * (time - 1750)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 3750:
        x = offset[0] + 4.0 * scale
        y = offset[1] - (0 + (2.5 - 0) / 1000 * (time - 2750)) * scale
        z = offset[2] + 0
    elif time < 3800:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3750)
    elif time < 4300:
        x = offset[0] + (4.0 + (2.0 - 4.0) / 500 * (time - 3800)) * scale
        y = offset[1] - (2.5 + (0 - 2.5) / 500 * (time - 3800)) * scale
        z = offset[2] + 0.05
    elif time < 4550:
        x = offset[0] + 2.0 * scale
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 4300)
    elif time < 5550:
        x = offset[0] + 2.0 * scale
        y = offset[1] - (0 + (2.0 - 0) / 1000 * (time - 4550)) * scale
        z = offset[2] + 0
    elif time <= 5600:
        x = offset[0] + 2.0 * scale
        y = offset[1] - 2.0 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 5550)
    return np.array([[x, y, z]])

def waypointF(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 0
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + (0 + (4.0 - 0) / 1000 * (time - 750)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2750:
        x = offset[0] + 4.0 * scale
        y = offset[1] - (0 + (2.5 - 0) / 1000 * (time - 1750)) * scale
        z = offset[2] + 0
    elif time < 2800:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 2750)
    elif time < 3300:
        x = offset[0] + (4.0 + (2.0 - 4.0) / 500 * (time - 2800)) * scale
        y = offset[1] - (2.5 + (0 - 2.5) / 500 * (time - 2800)) * scale
        z = offset[2] + 0.05
    elif time < 3550:
        x = offset[0] + 2.0 * scale
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 3300)
    elif time < 4550:
        x = offset[0] + 2.0 * scale
        y = offset[1] - (0 + (2.0 - 0) / 1000 * (time - 3550)) * scale
        z = offset[2] + 0
    elif time <= 4600:
        x = offset[0] + 2.0 * scale
        y = offset[1] - 2.0 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 4550)
    return np.array([[x, y, z]])

def waypointG(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (3.0 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (-3.0 * scale + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 3.0 * scale
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1250:
        x = offset[0] + (3.0 + np.sin((time - 750) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (2.0 + np.cos((time - 750) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 1750:
        x = offset[0] + 4.0 * scale
        y = offset[1] - (2.0 + (1.0 - 2.0) / 500 * (time - 1250)) * scale
        z = offset[2] + 0
    elif time < 2250:
        x = offset[0] + (3.0 + np.cos((time - 1750) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (1.0 - np.sin((time - 1750) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 2750:
        x = offset[0] + (3.0 + (1.0 - 3.0) / 500 * (time - 2250)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 3250:
        x = offset[0] + (1.0 - np.cos((time - 2750) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (1.0 - np.sin((time - 2750) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 3750:
        x = offset[0] + 0
        y = offset[1] - (1.0 + (2.0 - 1.0) / 500 * (time - 3250)) * scale
        z = offset[2] + 0
    elif time < 4250:
        x = offset[0] + (1.0 - np.cos((time - 3750) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (2.0 + np.sin((time - 3750) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 4750:
        x = offset[0] + (1.0 + (2.0 - 1.0) / 500 * (time - 4250)) * scale
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0
    elif time < 5250:
        x = offset[0] + 2.0 * scale
        y = offset[1] - (3.0 + (2.0 - 3.0) / 500 * (time - 4750)) * scale
        z = offset[2] + 0
    elif time <= 5300:
        x = offset[0] + 2.0 * scale
        y = offset[1] - 2.0 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 5250)
    return np.array([[x, y, z]])

def waypointH(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 0
        y = offset[1] + 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + (0 + (4.0 - 0) / 1000 * (time - 750)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 1800:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 0
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 1750)
    elif time < 2300:
        x = offset[0] + (4.0 + (2.0 - 4.0) / 500 * (time - 1800)) * scale
        y = offset[1] - 0
        z = offset[2] + 0.05
    elif time < 2550:
        x = offset[0] + 2.0 * scale
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 2100)
    elif time < 3550:
        x = offset[0] + 2.0 * scale
        y = offset[1] - (0 + (2.5 - 0) / 1000 * (time - 2550)) * scale
        z = offset[2] + 0
    elif time < 3600:
        x = offset[0] + 2.0 * scale
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3550)
    elif time < 4100:
        x = offset[0] + (2.0 + (4.0 - 2.0) / 500 * (time - 3600)) * scale
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0.05
    elif time < 4350:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 4100)
    elif time < 5350:
        x = offset[0] + (4.0 + (0 - 4.0) / 1000 * (time - 4350)) * scale
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0
    elif time <= 5400:
        x = offset[0] + 0
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 5350)
    return np.array([[x, y, z]])

def waypointI(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (4.0 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (-1.5 * scale + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 1.5 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + (4.0 + (0 - 4.0) / 1000 * (time - 750)) * scale
        y = offset[1] - 1.5 * scale
        z = offset[2] + 0
    elif time <= 1800:
        x = offset[0] + 0 * scale
        y = offset[1] - 1.5 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 1750)
    return np.array([[x, y, z]])


def waypointJ(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (4.0 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (-2.0 * scale + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 2.0 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + (4.0 + (1.0 - 4.0) / 1000 * (time - 750)) * scale
        y = offset[1] - 2.0 * scale
        z = offset[2] + 0
    elif time < 2750:
        x = offset[0] + (1.0 - np.sin((time - 1750) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (1.0 + np.cos((time - 1750) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time <= 2800:
        x = offset[0] + 1.0 * scale
        y = offset[1] - 0
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 2750)
    return np.array([[x, y, z]])


def waypointK(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 0
        y = offset[1] + 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + (0 + (4.0 - 0) / 1000 * (time - 750)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 1800:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 0
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 1750)
    elif time < 2300:
        x = offset[0] + 4.0 * scale
        y = offset[1] - (0 + (2.0 - 0) / 500 * (time - 1800)) * scale
        z = offset[2] + 0.05
    elif time < 2550:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 2.0 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 2300)
    elif time < 3550:
        x = offset[0] + (4.0 + (2.0 - 4.0) / 1000 * (time - 2550)) * scale
        y = offset[1] - (2.0 + (0 - 2.0) / 1000 * (time - 2550)) * scale
        z = offset[2] + 0
    elif time < 4550:
        x = offset[0] + (2.0 + (0 - 2.0) / 1000 * (time - 3550)) * scale
        y = offset[1] - (0 + (2.5 - 0) / 1000 * (time - 3550)) * scale
        z = offset[2] + 0
    elif time <= 4600:
        x = offset[0] + 0
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 4550)
    return np.array([[x, y, z]])



def waypointL(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (4.0 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + (4.0 + (0 - 4.0) / 1000 * (time - 750)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2750:
        x = offset[0] + 0
        y = offset[1] - (0 + (2.0 - 0) / 1000 * (time - 1750)) * scale
        z = offset[2] + 0
    elif time <= 2800:
        x = offset[0] + 0
        y = offset[1] - 2.0 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 2750)
    return np.array([[x, y, z]])




def waypointM(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 0
        y = offset[1] + 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + (0 + (4.0 - 0) / 1000 * (time - 750)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2750:
        x = offset[0] + (4.0 + (2.0 - 4.0) / 1000 * (time - 1750)) * scale
        y = offset[1] - (0 + (1.5 - 0) / 1000 * (time - 1750)) * scale
        z = offset[2] + 0
    elif time < 3750:
        x = offset[0] + (2.0 + (4.0 - 2.0) / 1000 * (time - 2750)) * scale
        y = offset[1] - (1.5 + (3.0 - 1.5) / 1000 * (time - 2750)) * scale
        z = offset[2] + 0
    elif time < 4750:
        x = offset[0] + (4.0 + (0 - 4.0) / 1000 * (time - 3750)) * scale
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0
    elif time <= 4800:
        x = offset[0] + 0
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 4750)
    return np.array([[x, y, z]])

def waypointN(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 0
        y = offset[1] + 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + (0 + (4.0 - 0) / 1000 * (time - 750)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2750:
        x = offset[0] + (4.0 + (0 - 4.0) / 1000 * (time - 1750)) * scale
        y = offset[1] - (0 + (3.0 - 0) / 1000 * (time - 1750)) * scale
        z = offset[2] + 0
    elif time < 3750:
        x = offset[0] + (0 + (4.0 - 0) / 1000 * (time - 2750)) * scale
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0
    elif time <= 3800:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3750)
    return np.array([[x, y, z]])

def waypointO(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (-1.0 * scale + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 0
        y = offset[1] - 1.0 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1250:
        x = offset[0] + (1.0 - np.sin((time - 750) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (1.0 - np.cos((time - 750) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 1750:
        x = offset[0] + (1.0 + (3.0 - 1.0) / 500 * (time - 1250)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2250:
        x = offset[0] + (3.0 + np.cos((time - 1750) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (1.0 - np.sin((time - 1750) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 2750:
        x = offset[0] + 4.0 * scale
        y = offset[1] - (1.0 + (2.0 - 1.0) / 500 * (time - 2250)) * scale
        z = offset[2] + 0
    elif time < 3250:
        x = offset[0] + (3.0 + np.cos((time - 2750) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (2.0 + np.sin((time - 2750) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 3750:
        x = offset[0] + (3.0 + (1.0 - 3.0) / 500 * (time - 3250)) * scale
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0
    elif time < 4250:
        x = offset[0] + (1.0 - np.cos((time - 3750) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (2.0 + np.sin((time - 3750) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 4750:
        x = offset[0] + 0
        y = offset[1] - (2.0 + (1.0 - 2.0) / 500 * (time - 4250)) * scale
        z = offset[2] + 0
    elif time <= 4800:
        x = offset[0] + 0
        y = offset[1] - 1.0 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 4750)
    return np.array([[x, y, z]])

def waypointP(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 0
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + (0 + (4.0 - 0) / 1000 * (time - 750)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2250:
        x = offset[0] + 4.0 * scale
        y = offset[1] - (0 + (1.5 - 0) / 500 * (time - 1750)) * scale
        z = offset[2] + 0
    elif time < 3250:
        x = offset[0] + (3.0 + np.cos((time - 2250) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (1.5 + np.sin((time - 2250) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 3750:
        x = offset[0] + 2.0 * scale
        y = offset[1] - (1.5 + (0 - 1.5) / 500 * (time - 3250)) * scale
        z = offset[2] + 0
    elif time <= 3800:
        x = offset[0] + 2.0 * scale
        y = offset[1] - 0
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3750)
    return np.array([[x, y, z]])

def waypointQ(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (-1.0 * scale + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 0
        y = offset[1] - 1.0 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1250:
        x = offset[0] + (1.0 - np.sin((time - 750) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (1.0 - np.cos((time - 750) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 1750:
        x = offset[0] + (1.0 + (3.0 - 1.0) / 500 * (time - 1250)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2250:
        x = offset[0] + (3.0 + np.cos((time - 1750) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (1.0 - np.sin((time - 1750) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 2750:
        x = offset[0] + 4.0 * scale
        y = offset[1] - (1.0 + (2.0 - 1.0) / 500 * (time - 2250)) * scale
        z = offset[2] + 0
    elif time < 3250:
        x = offset[0] + (3.0 + np.cos((time - 2750) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (2.0 + np.sin((time - 2750) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 3750:
        x = offset[0] + (3.0 + (1.0 - 3.0) / 500 * (time - 3250)) * scale
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0
    elif time < 4250:
        x = offset[0] + (1.0 - np.cos((time - 3750) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (2.0 + np.sin((time - 3750) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 4750:
        x = offset[0] + 0
        y = offset[1] - (2.0 + (1.0 - 2.0) / 500 * (time - 4250)) * scale
        z = offset[2] + 0
    elif time < 4800:
        x = offset[0] + 0
        y = offset[1] - 1.0 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 4750)
    elif time < 5300:
        x = offset[0] + (0 + (0.75 - 0)/500 * (time - 4800)) * scale
        y = offset[1] - (1.0 + (2.25 - 1.0)/500 * (time - 4800)) * scale
        z = offset[2] + 0.05
    elif time < 5550:
        x = offset[0] + 0.75 * scale
        y = offset[1] - 2.25 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 5300)
    elif time < 6050:
        x = offset[0] + (0.75 + (0 - 0.75)/500 * (time - 5550)) * scale
        y = offset[1] - (2.25 + (3.0 - 2.25)/500 * (time -5550)) * scale
        z = offset[2] + 0
    elif time <= 6100:
        x = offset[0] + 0
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 6050)
    return np.array([[x, y, z]])

def waypointR(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 0
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + (0 + (4.0 - 0) / 1000 * (time - 750)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2250:
        x = offset[0] + 4.0 * scale
        y = offset[1] - (0 + (1.5 - 0) / 500 * (time - 1750)) * scale
        z = offset[2] + 0
    elif time < 3250:
        x = offset[0] + (3.0 + np.cos((time - 2250) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (1.5 + np.sin((time - 2250) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 3750:
        x = offset[0] + 2.0 * scale
        y = offset[1] - (1.5 + (0 - 1.5) / 500 * (time - 3250)) * scale
        z = offset[2] + 0
    elif time < 3800:
        x = offset[0] + 2.0 * scale
        y = offset[1] - 0
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3750)
    elif time < 4300:
        x = offset[0] + 2.0 * scale
        y = offset[1] - (0 + (1.0 - 0)/500 * (time - 3800)) * scale
        z = offset[2] + 0.05
    elif time < 4550:
        x = offset[0] + 2.0 * scale
        y = offset[1] - 1.0 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 4300)
    elif time < 5550:
        x = offset[0] + (2.0 + (0 - 2.0)/1000 * (time - 4550)) * scale
        y = offset[1] - (1.0 + (3.0 - 1.0)/1000 * (time - 4550)) * scale
        z = offset[2] + 0
    elif time <= 5600:
        x = offset[0] + 0
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 5550)
    return np.array([[x, y, z]])

def waypointS(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (3.707 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (-2.707 * scale + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 3.707 * scale
        y = offset[1] - 2.707 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1000:
        x = offset[0] + (3.0 + np.sin(np.pi / 4.0 + (time - 750) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (2.0 + np.cos(np.pi / 4.0 + (time - 750) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 1500:
        x = offset[0] + 4.0 * scale
        y = offset[1] - (2.0 + (1.0 - 2.0) / 500 * (time - 1000)) * scale
        z = offset[2] + 0
    elif time < 2500:
        x = offset[0] + (3.0 + np.cos((time - 1500) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (1.0 - np.sin((time - 1500) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 3000:
        x = offset[0] + 2.0 * scale
        y = offset[1] - (1.0 + (2.0 - 1.0) / 500 * (time - 2500)) * scale
        z = offset[2] + 0
    elif time < 4000:
        x = offset[0] + (1.0 + np.cos((time - 3000) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (2.0 + np.sin((time - 3000) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 4500:
        x = offset[0] + 0
        y = offset[1] - (2.0 + (1.0 - 2.0) / 500 * (time - 4000)) * scale
        z = offset[2] + 0
    elif time < 5000:
        x = offset[0] + (1.0 - np.cos((time - 4500) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (1.0 - np.sin((time - 4500) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time <= 5050:
        x = offset[0] + 1.0 * scale
        y = offset[1] - 0
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 5000)
    return np.array([[x, y, z]])

def waypointT(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (4.0 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 4.0 * scale
        y = offset[1] + 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + 4.0 * scale
        y = offset[1] - (0 + (3.0 - 0) / 1000 * (time - 750)) * scale
        z = offset[2] + 0
    elif time < 1800:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 1750)
    elif time < 2300:
        x = offset[0] + 4.0 * scale
        y = offset[1] - (3.0 + (1.5 - 3.0) / 500 * (time - 1800)) * scale
        z = offset[2] + 0.05
    elif time < 2550:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 1.5 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 2300)
    elif time < 3550:
        x = offset[0] + (4.0 + (0 - 4.0) / 1000 * (time - 2550)) * scale
        y = offset[1] - 1.5 * scale
        z = offset[2] + 0
    elif time <= 3600:
        x = offset[0] + 0
        y = offset[1] - 1.5 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3550)
    return np.array([[x, y, z]])

def waypointU(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (4.0 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + (4.0 + (1.0 - 4.0) / 1000 * (time - 750)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2250:
        x = offset[0] + (1.0 - np.sin((time - 1750) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (1.0 - np.cos((time - 1750) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 2500:
        x = offset[0] + 0
        y = offset[1] - (1.0 + (1.5 - 1.0) / 250 * (time - 2250)) * scale
        z = offset[2] + 0
    elif time < 3000:
        x = offset[0] + (1.0 - np.cos((time - 2500) * np.pi / 1000) * 1.0) * scale
        y = offset[1] - (1.5 + np.sin((time - 2500) * np.pi / 1000) * 1.0) * scale
        z = offset[2] + 0
    elif time < 4000:
        x = offset[0] + (1.0 + (4.0 - 1.0) / 1000 * (time - 3000)) * scale
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0
    elif time <= 4050:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 2.5 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 4000)
    return np.array([[x, y, z]])

def waypointV(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (4.0 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + (4.0 + (0 - 4.0) / 1000 * (time - 750)) * scale
        y = offset[1] - (0 + (1.5 - 0) / 1000 * (time - 750)) * scale
        z = offset[2] + 0
    elif time < 2750:
        x = offset[0] + (0 + (4.0 - 0) / 1000 * (time - 1750)) * scale
        y = offset[1] - (1.5 + (3.0 - 1.5) / 1000 * (time - 1750)) * scale
        z = offset[2] + 0
    elif time <= 2800:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 2750)
    return np.array([[x, y, z]])

def waypointW(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (4.0 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 4.0 * scale
        y = offset[1] + 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + (4.0 + (0 - 4.0) / 1000 * (time - 750)) * scale
        y = offset[1] - 0
        z = offset[2] + 0
    elif time < 2750:
        x = offset[0] + (0 + (2.0 - 0) / 1000 * (time - 1750)) * scale
        y = offset[1] - (0 + (1.5 - 0) / 1000 * (time - 1750)) * scale
        z = offset[2] + 0
    elif time < 3750:
        x = offset[0] + (2.0 + (0 - 2.0) / 1000 * (time - 2750)) * scale
        y = offset[1] - (1.5 + (3.0 - 1.5) / 1000 * (time - 2750)) * scale
        z = offset[2] + 0
    elif time < 4750:
        x = offset[0] + (0 + (4.0 - 0) / 1000 * (time - 3750)) * scale
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0
    elif time <= 4800:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 4750)
    return np.array([[x, y, z]])

def waypointX(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (0 + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 0
        y = offset[1] + 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + (0 + (4.0 - 0) / 1000 * (time - 750)) * scale
        y = offset[1] - (0 + (3.0 - 0) / 1000 * (time - 750)) * scale
        z = offset[2] + 0
    elif time < 1800:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 1750)
    elif time < 2300:
        x = offset[0] + (4.0 + (0 - 4.0) / 500 * (time - 1800)) * scale
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0.05
    elif time < 2550:
        x = offset[0] + 0
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 2300)
    elif time < 3550:
        x = offset[0] + (0 + (4.0 - 0) / 1000 * (time - 2550)) * scale
        y = offset[1] - (3.0 + (0 - 3.0) / 1000 * (time - 2550)) * scale
        z = offset[2] + 0
    elif time <= 3600:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 0
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3550)
    return np.array([[x, y, z]])

def waypointY(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (4.0 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 4.0 * scale
        y = offset[1] + 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + (4.0 + (2.0 - 4.0) / 1000 * (time - 750)) * scale
        y = offset[1] - (0 + (1.5 - 0) / 1000 * (time - 750)) * scale
        z = offset[2] + 0
    elif time < 2750:
        x = offset[0] + (2.0 + (4.0 - 2.0) / 1000 * (time - 1750)) * scale
        y = offset[1] - (1.5 + (3.0 - 1.5) / 1000 * (time - 1750)) * scale
        z = offset[2] + 0
    elif time < 2800:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 2750)
    elif time < 3300:
        x = offset[0] + (4.0 + (2.0 - 4.0) / 500 * (time - 2800)) * scale
        y = offset[1] - (3.0 + (1.5 - 3.0) / 500 * (time - 2800)) * scale
        z = offset[2] + 0.05
    elif time < 3550:
        x = offset[0] + 2.0 * scale
        y = offset[1] - 1.5 * scale
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 3300)
    elif time < 4550:
        x = offset[0] + (2.0 + (0 - 2.0) / 1000 * (time - 3550)) * scale
        y = offset[1] - 1.5 * scale
        z = offset[2] + 0
    elif time <= 4600:
        x = offset[0] + 0
        y = offset[1] - 1.5 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 4550)
    return np.array([[x, y, z]])

def waypointZ(time, prev_xyz, offset, scale):
    if time < 500:
        x = prev_xyz[0] + (4.0 * scale + offset[0] - prev_xyz[0]) / 500 * time
        y = prev_xyz[1] + (0 + offset[1] - prev_xyz[1]) / 500 * time
        z = prev_xyz[2] + (0.05 + offset[2] - prev_xyz[2]) / 500 * time
    elif time < 750:
        x = offset[0] + 4.0 * scale
        y = offset[1] - 0
        z = offset[2] + 0.05 + (0 - 0.05) / 250 * (time - 500)
    elif time < 1750:
        x = offset[0] + 4.0 * scale
        y = offset[1] - (0 + (2.0 - 0) / 1000 * (time - 750)) * scale
        z = offset[2] + 0
    elif time < 2750:
        x = offset[0] + (4.0 + (0 - 4.0) / 1000 * (time - 1750)) * scale
        y = offset[1] - (2.0 + (0 - 2.0) / 1000 * (time - 1750)) * scale
        z = offset[2] + 0
    elif time < 3750:
        x = offset[0] + 0
        y = offset[1] - (0 + (3.0 - 0) / 1000 * (time - 2750)) * scale
        z = offset[2] + 0
    elif time <= 3800:
        x = offset[0] + 0
        y = offset[1] - 3.0 * scale
        z = offset[2] + 0 + (0.05 - 0) / 50 * (time - 3750)
    return np.array([[x, y, z]])


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    #waypoints = drawH([0.1, 0, 0.1], [0.11, 0, 0], 25, 0.015)
    waypoints1 = get_string_trajectory("HELLOWORLD", start_pos=[0.1,0,0.01], y_alinement = 'center')

    # Print the planned trajectory
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(waypoints1[:,0], waypoints1[:,1], waypoints1[:,2], c='r')
    # print(waypoints)
    plt.show()
