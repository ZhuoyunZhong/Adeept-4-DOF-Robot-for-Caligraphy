

def parse_letter(letter, prev_pos, offset, rate):
    if letter == 'A':
        return drawA(prev_pos, offset, rate)
    elif letter == 'B':
        return drawB(prev_pos, offset)
    elif letter == 'C':
        return drawC(prev_pos, offset)
    elif letter == 'D':
        return drawD(prev_pos, offset)
    elif letter == 'E':
        return drawE(prev_pos, offset)
    elif letter == 'F':
        return drawF(prev_pos, offset)
    elif letter == 'G':
        return drawG(prev_pos, offset)
    elif letter == 'H':
        return drawH(prev_pos, offset)
    elif letter == 'I':
        return drawI(prev_pos, offset)
    elif letter == 'J':
        return drawJ(prev_pos, offset)
    elif letter == 'K':
        return drawK(prev_pos, offset)
    elif letter == 'L':
        return drawL(prev_pos, offset)
    elif letter == 'M':
        return drawM(prev_pos, offset)
    elif letter == 'N':
        return drawN(prev_pos, offset)
    elif letter == 'O':
        return drawO(prev_pos, offset)
    elif letter == 'P':
        return drawP(prev_pos, offset)
    elif letter == 'Q':
        return drawQ(prev_pos, offset)
    elif letter == 'R':
        return drawR(prev_pos, offset)
    elif letter == 'S':
        return drawS(prev_pos, offset)
    elif letter == 'T':
        return drawT(prev_pos, offset)
    elif letter == 'U':
        return drawU(prev_pos, offset)
    elif letter == 'V':
        return drawV(prev_pos, offset)
    elif letter == 'W':
        return drawW(prev_pos, offset)
    elif letter == 'X':
        return drawX(prev_pos, offset)
    elif letter == 'Y':
        return drawY(prev_pos, offset)
    elif letter == 'Z':
        return drawZ(prev_pos, offset)





def drawA(prev_pos, offset, rate):
    t_A = 3700
    return 0

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

def drawI(prev_pos, offset):
    return 0

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


def waypointA(time, prev_xyz, offset):
    if time < 500:
        x = prev_xyz(0) + (0 + offset(0) - prev_xyz(0)) / 500 * time
        y = prev_xyz(1) + (0 + offset(1) - prev_xyz(1)) / 500 * time
        z = prev_xyz(2) + (0.5 + offset(2) - prev_xyz(2)) / 500 * time
    elif time < 550:
        x = offset(0) + 0
        y = offset(1) + 0
        z = offset(2) + 0.5 + (0 - 0.5) / 50 * (time - 500)
    elif time < 1550:
        x = offset(0) + 0 + (1.5 - 0) / 1000 * (time - 550)
        y = offset(1) + 0 + (4.0 - 0) / 1000 * (time - 550)
        z = offset(2) + 0
    elif time < 2550:
        x = offset(0) + 1.5 + (3.0 - 1.5) / 1000 * (time - 1550)
        y = offset(1) + 4.0 + (0 - 4.0) / 1000 * (time - 1550)
        z = offset(2) + 0
    elif time < 2600:
        x = offset(0) + 3
        y = offset(1) + 0
        z = offset(2) + 0 + (0.5 - 0) / 50 * (time - 2550)
    elif time < 3100:
        x = offset(0) + 3 + (0.75 - 3) / 500 * (time - 2600)
        y = offset(1) + 0 + (2 - 0) / 500 * (time - 2600)
        z = offset(2) + 0.5
    elif time < 3150:
        x = offset(0) + 0.75
        y = offset(1) + 2
        z = offset(2) + 0.5 + (0 - 0.5) / 50 * (time - 3100)
    elif time < 3650:
        x = offset(0) + 0.75 + (2.25 - 0.75) / 500 * (time - 3150)
        y = offset(1) + 2
        z = offset(2) + 0
    elif time <= 3700:
        x = offset(0) + 2.25
        y = offset(1) + 2
        z = offset(2) + 0 + (0.5 - 0) / 50 * (time - 3650)
    return (x, y, z)/100

