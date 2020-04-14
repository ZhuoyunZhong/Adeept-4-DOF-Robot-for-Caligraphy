%% RBE501 Project Alphabet Trajectory generation

% time lookup table
t_A = 3700; t_E = 5200; t_F = 4200; t_H = 4800;
t_I = 1600; t_K = 4200; t_L = 2600; t_M = 4600;
t_N = 3600; t_T = 3200; t_V = 2600; t_W = 4600;
t_X = 3200; t_Y = 4200; t_Z = 3600;

prev_pos = [0; 0; 0.5];
figure('Name','Draw String HKLM');
scatter3(0, 0, 0);
hold on
for t1 = 0: 25: t_V
    curpos = drawV(0, 0, prev_pos, t1);
    scatter3(curpos(1),curpos(2),curpos(3));
    if t1 == t_V
        prev_pos = curpos;
    end
end
for t2 = 0: 25: t_W
    curpos = drawW(4, 0, prev_pos, t2);
    scatter3(curpos(1),curpos(2),curpos(3));
    if t2 == t_W
        prev_pos = curpos;
    end
end
for t3= 0: 25: t_X
    curpos = drawX(8, 0, prev_pos, t3);
    scatter3(curpos(1),curpos(2),curpos(3));
    if t3 == t_X
        prev_pos = curpos;
    end
end
for t4= 0: 25: t_Z
    curpos = drawZ(12, 0, prev_pos, t4);
    scatter3(curpos(1),curpos(2),curpos(3));
    if t4 == t_Z
        prev_pos = curpos;
    end
end
xlabel('x');
ylabel('y');
zlabel('z');
xlim([-1 18]);
ylim([-1 5]);
zlim([-1 3]);
title('Trajectory Planning for HKLM');


%% Alphabet Functions
% Letter 'A'
function pos = drawA(x_offset, y_offset, prev_xyz, time)
    pos = [prev_xyz(1); prev_xyz(2); prev_xyz(3)];
    if time < 500
        pos(1) = prev_xyz(1) + (0 + x_offset - prev_xyz(1))/500 * time;
        pos(2) = prev_xyz(2) + (0 + y_offset - prev_xyz(2))/500 * time;
        pos(3) = prev_xyz(3) + (0.5 - prev_xyz(3))/500 * time;
    elseif time < 550
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 0;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 500);
    elseif time < 1550
        pos(1) = x_offset + 0 + (1.5 - 0)/1000 * (time - 550);
        pos(2) = y_offset + 0 + (4.0 - 0)/1000 * (time - 550);
        pos(3) = 0;
    elseif time < 2550
        pos(1) = x_offset + 1.5 + (3.0 - 1.5)/1000 * (time - 1550);
        pos(2) = y_offset + 4.0 + (0 - 4.0)/1000 * (time - 1550);
        pos(3) = 0;
    elseif time < 2600
        pos(1) = x_offset + 3;
        pos(2) = y_offset + 0;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 2550);
    elseif time < 3100
        pos(1) = x_offset + 3 + (0.75 - 3)/500 * (time - 2600);
        pos(2) = y_offset + 0 + (2 - 0)/500 * (time - 2600);
        pos(3) = 0.5;
    elseif time < 3150
        pos(1) = x_offset + 0.75;
        pos(2) = y_offset + 2;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 3100);
    elseif time < 3650
        pos(1) = x_offset + 0.75 + (2.25 - 0.75)/500 * (time - 3150);
        pos(2) = y_offset + 2;
        pos(3) = 0;
    elseif time <= 3700
        pos(1) = x_offset + 2.25;
        pos(2) = y_offset + 2;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 3650);
    end
end

% letter 'E'
function pos = drawE(x_offset, y_offset, prev_xyz, time)
    pos = [prev_xyz(1); prev_xyz(2); prev_xyz(3)];
    if time < 500 % [2.5; 0; 0.5] move
        pos(1) = prev_xyz(1) + (2.5 + x_offset - prev_xyz(1))/500 * time;
        pos(2) = prev_xyz(2) + (0 + y_offset - prev_xyz(2))/500 * time;
        pos(3) = prev_xyz(3) + (0.5 - prev_xyz(3))/500 * time;
    elseif time < 550 % [2.5; 0; 0] down
        pos(1) = x_offset + 2.5;
        pos(2) = y_offset + 0;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 500);
    elseif time < 1550 % [0; 0; 0] write
        pos(1) = x_offset + 2.5 + (0 - 2.5)/1000 * (time - 550);
        pos(2) = y_offset + 0;
        pos(3) = 0;
    elseif time < 2550 % [0; 4; 0] write
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 0 + (4 - 0)/1000 * (time - 1550);
        pos(3) = 0;
    elseif time < 3550 % [2.5; 4; 0] write
        pos(1) = x_offset + 0 + (2.5 - 0)/1000 * (time - 2550);
        pos(2) = y_offset + 4;
        pos(3) = 0;
    elseif time < 3600 % [2.5; 4; 0.5] up
        pos(1) = x_offset + 2.5;
        pos(2) = y_offset + 4;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 3550);
    elseif time < 4100 % [0; 2; 0.5] move
        pos(1) = x_offset + 2.5 + (0 - 2.5)/500 * (time - 3600);
        pos(2) = y_offset + 4 + (2 - 4)/500 * (time - 3600);
        pos(3) = 0.5;
    elseif time < 4150 % [0; 2; 0] down
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 2;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 4100);
    elseif time < 5150 % [2; 2; 0] write
        pos(1) = x_offset + 0 + (2 - 0)/1000 * (time - 4150);
        pos(2) = y_offset + 2;
        pos(3) = 0;
    elseif time <= 5200 % [2; 2; 0.5] up
        pos(1) = x_offset + 2;
        pos(2) = y_offset + 2;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 5150);
    end
end

% letter 'F'
function pos = drawF(x_offset, y_offset, prev_xyz, time)
    pos = [prev_xyz(1); prev_xyz(2); prev_xyz(3)];
    if time < 500 % [0; 0; 0.5] move
        pos(1) = prev_xyz(1) + (0 + x_offset - prev_xyz(1))/500 * time;
        pos(2) = prev_xyz(2) + (0 + y_offset - prev_xyz(2))/500 * time;
        pos(3) = prev_xyz(3) + (0.5 - prev_xyz(3))/500 * time;
    elseif time < 550 % [0; 0; 0] down
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 0;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 500);
    elseif time < 1550 % [0; 4; 0] write
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 0 + (4 - 0)/1000 * (time - 550);
        pos(3) = 0;
    elseif time < 2550 % [2.5; 4; 0] write
        pos(1) = x_offset + 0 + (2.5 - 0)/1000 * (time - 1550);
        pos(2) = y_offset + 4;
        pos(3) = 0;
    elseif time < 2600 % [2.5; 4; 0.5] up
        pos(1) = x_offset + 2.5;
        pos(2) = y_offset + 4;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 2550);
    elseif time < 3100 % [0; 2; 0.5] move
        pos(1) = x_offset + 2.5 + (0 - 2.5)/500 * (time - 2600);
        pos(2) = y_offset + 4 + (2 - 4)/500 * (time - 2600);
        pos(3) = 0.5;
    elseif time < 3150 % [0; 2; 0] down
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 2;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 3100);
    elseif time < 4150 % [2; 2; 0] write
        pos(1) = x_offset + 0 + (2 - 0)/1000 * (time - 3150);
        pos(2) = y_offset + 2;
        pos(3) = 0;
    elseif time <= 4200 % [2; 2; 0.5] up
        pos(1) = x_offset + 2;
        pos(2) = y_offset + 2;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 4150);
    end
end

% letter 'H'
function pos = drawH(x_offset, y_offset, prev_xyz, time)
    pos = [prev_xyz(1); prev_xyz(2); prev_xyz(3)];
    if time < 500 % [0; 0; 0.5] move
        pos(1) = prev_xyz(1) + (0 + x_offset - prev_xyz(1))/500 * time;
        pos(2) = prev_xyz(2) + (0 + y_offset - prev_xyz(2))/500 * time;
        pos(3) = prev_xyz(3) + (0.5 - prev_xyz(3))/500 * time;
    elseif time < 550 % [0; 0; 0] down
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 0;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 500);
    elseif time < 1550 % [0; 4; 0] write
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 0 + (4 - 0)/1000 * (time - 550);
        pos(3) = 0;
    elseif time < 1600 % [0; 4; 0.5] up
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 4;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 1550);
    elseif time < 2100 % [0; 2; 0.5] move
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 4 + (2 - 4)/500 * (time - 1600);
        pos(3) = 0.5;
    elseif time < 2150 % [0; 2; 0] down
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 2;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 2100);
    elseif time < 3150 % [2.5; 2; 0] write
        pos(1) = x_offset + 0 + (2.5 - 0)/1000 * (time - 2150);
        pos(2) = y_offset + 2;
        pos(3) = 0;
    elseif time < 3200 % [2.5; 2; 0.5] up
        pos(1) = x_offset + 2.5;
        pos(2) = y_offset + 2;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 3150);
    elseif time < 3700 % [2.5; 4; 0.5] move
        pos(1) = x_offset + 2.5;
        pos(2) = y_offset + 2 + (4 - 2)/500 * (time - 3200);
        pos(3) = 0.5;
    elseif time < 3750 % [2.5; 4; 0] down
        pos(1) = x_offset + 2.5;
        pos(2) = y_offset + 4;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 3700);
    elseif time < 4750 % [2.5; 0; 0] write
        pos(1) = x_offset + 2.5;
        pos(2) = y_offset + 4 + (0 - 4)/1000 * (time - 3750);
        pos(3) = 0;
    elseif time <= 4800 % [2.5; 0; 0.5] up
        pos(1) = x_offset + 2.5;
        pos(2) = y_offset + 0;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 4750);
    end
end

% letter 'I'
function pos = drawI(x_offset, y_offset, prev_xyz, time)
    pos = [prev_xyz(1); prev_xyz(2); prev_xyz(3)];
    if time < 500 % [1.5; 4; 0.5] move
        pos(1) = prev_xyz(1) + (1.5 + x_offset - prev_xyz(1))/500 * time;
        pos(2) = prev_xyz(2) + (4.0 + y_offset - prev_xyz(2))/500 * time;
        pos(3) = prev_xyz(3) + (0.5 - prev_xyz(3))/500 * time;
    elseif time < 550 % [1.5; 4; 0] down
        pos(1) = x_offset + 1.5;
        pos(2) = y_offset + 4;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 500);
    elseif time < 1550 % [1.5; 0; 0] write
        pos(1) = x_offset + 1.5;
        pos(2) = y_offset + 4 + (0 - 4.0)/1000 * (time - 550);
        pos(3) = 0;
    elseif time <= 1600 % [1.5; 0; 0.5] up
        pos(1) = x_offset + 1.5;
        pos(2) = y_offset;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 1550);
    end
end

% letter 'K'
function pos = drawK(x_offset, y_offset, prev_xyz, time)
    pos = [prev_xyz(1); prev_xyz(2); prev_xyz(3)];
    if time < 500 % [0; 0; 0.5] move
        pos(1) = prev_xyz(1) + (0 + x_offset - prev_xyz(1))/500 * time;
        pos(2) = prev_xyz(2) + (0 + y_offset - prev_xyz(2))/500 * time;
        pos(3) = prev_xyz(3) + (0.5 - prev_xyz(3))/500 * time;
    elseif time < 550 % [0; 0; 0] down
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 0;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 500);
    elseif time < 1550 % [0; 4; 0] write
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 0 + (4 - 0)/1000 * (time - 550);
        pos(3) = 0;
    elseif time < 1600 % [0; 4; 0.5] up
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 4;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 1550);
    elseif time < 2100 % [2; 4; 0.5] move
        pos(1) = x_offset + 0 + (2 - 0)/500 * (time - 1600);
        pos(2) = y_offset + 4;
        pos(3) = 0.5;
    elseif time < 2150 % [2; 4; 0] down
        pos(1) = x_offset + 2;
        pos(2) = y_offset + 4;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 2100);
    elseif time < 3150 % [0; 2; 0] write
        pos(1) = x_offset + 2 + (0 - 2)/1000 * (time - 2150);
        pos(2) = y_offset + 4 + (2 - 4)/1000 * (time - 2150);
        pos(3) = 0;
    elseif time < 4150 % [2.5; 0; 0] write
        pos(1) = x_offset + 0 + (2.5 - 0)/1000 * (time - 3150);
        pos(2) = y_offset + 2 + (0 - 2)/1000 * (time - 3150);
        pos(3) = 0;
    elseif time <= 4200 % [2.5; 0; 0.5] up
        pos(1) = x_offset + 2.5;
        pos(2) = y_offset + 0;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 4150);
    end
end

% letter 'L'
function pos = drawL(x_offset, y_offset, prev_xyz, time)
    pos = [prev_xyz(1); prev_xyz(2); prev_xyz(3)];
    if time < 500 % [0; 4; 0.5] move
        pos(1) = prev_xyz(1) + (0 + x_offset - prev_xyz(1))/500 * time;
        pos(2) = prev_xyz(2) + (4 + y_offset - prev_xyz(2))/500 * time;
        pos(3) = prev_xyz(3) + (0.5 - prev_xyz(3))/500 * time;
    elseif time < 550 % [0; 4; 0] down
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 4;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 500);
    elseif time < 1550 % [0; 0; 0] write
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 4 + (0 - 4)/1000 * (time - 550);
        pos(3) = 0;
    elseif time < 2550 % [2; 0; 0] write
        pos(1) = x_offset + 0 + (2 - 0)/1000 * (time - 1550);
        pos(2) = y_offset + 0;
        pos(3) = 0;
    elseif time <= 2600 % [2; 0; 0.5] up
        pos(1) = x_offset + 2;
        pos(2) = y_offset + 0;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 2550);
    end
end

% letter 'M'
function pos = drawM(x_offset, y_offset, prev_xyz, time)
    pos = [prev_xyz(1); prev_xyz(2); prev_xyz(3)];
    if time < 500 % [0; 0; 0.5] move
        pos(1) = prev_xyz(1) + (0 + x_offset - prev_xyz(1))/500 * time;
        pos(2) = prev_xyz(2) + (0 + y_offset - prev_xyz(2))/500 * time;
        pos(3) = prev_xyz(3) + (0.5 - prev_xyz(3))/500 * time;
    elseif time < 550 % [0; 0; 0] down
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 0;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 500);
    elseif time < 1550 % [0; 4; 0] write
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 0 + (4 - 0)/1000 * (time - 550);
        pos(3) = 0;
    elseif time < 2550 % [1.5; 2; 0] write
        pos(1) = x_offset + 0 + (1.5 - 0)/1000 * (time - 1550);
        pos(2) = y_offset + 4 + (2 - 4)/1000 * (time - 1550);
        pos(3) = 0;
    elseif time < 3550 % [3; 4; 0] write
        pos(1) = x_offset + 1.5 + (3 - 1.5)/1000 * (time - 2550);
        pos(2) = y_offset + 2 + (4 - 2)/1000 * (time - 2550);
        pos(3) = 0;
    elseif time < 4550 % [3; 0; 0] write
        pos(1) = x_offset + 3;
        pos(2) = y_offset + 4 + (0 - 4)/1000 * (time - 3550);
        pos(3) = 0;
    elseif time <= 4600 % [3; 0; 0.5] up
        pos(1) = x_offset + 3;
        pos(2) = y_offset + 0;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 4550);
    end
end

% letter 'N'
function pos = drawN(x_offset, y_offset, prev_xyz, time)
    pos = [prev_xyz(1); prev_xyz(2); prev_xyz(3)];
    if time < 500 % [0; 0; 0.5] move
        pos(1) = prev_xyz(1) + (0 + x_offset - prev_xyz(1))/500 * time;
        pos(2) = prev_xyz(2) + (0 + y_offset - prev_xyz(2))/500 * time;
        pos(3) = prev_xyz(3) + (0.5 - prev_xyz(3))/500 * time;
    elseif time < 550 % [0; 0; 0] down
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 0;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 500);
    elseif time < 1550 % [0; 4; 0] write
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 0 + (4 - 0)/1000 * (time - 550);
        pos(3) = 0;
    elseif time < 2550 % [3; 0; 0] write
        pos(1) = x_offset + 0 + (3 - 0)/1000 * (time - 1550);
        pos(2) = y_offset + 4 + (0 - 4)/1000 * (time - 1550);
        pos(3) = 0;
    elseif time < 3550 % [3; 4; 0] write
        pos(1) = x_offset + 3;
        pos(2) = y_offset + 0 + (4 - 0)/1000 * (time - 2550);
        pos(3) = 0;
    elseif time <= 3600 % [3; 4; 0.5] up
        pos(1) = x_offset + 3;
        pos(2) = y_offset + 4;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 3550);
    end
end

% letter 'T'
function pos = drawT(x_offset, y_offset, prev_xyz, time)
    pos = [prev_xyz(1); prev_xyz(2); prev_xyz(3)];
    if time < 500 % [0; 4; 0.5] move
        pos(1) = prev_xyz(1) + (0 + x_offset - prev_xyz(1))/500 * time;
        pos(2) = prev_xyz(2) + (4 + y_offset - prev_xyz(2))/500 * time;
        pos(3) = prev_xyz(3) + (0.5 - prev_xyz(3))/500 * time;
    elseif time < 550 % [0; 4; 0] down
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 4;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 500);
    elseif time < 1550 % [3; 4; 0] write
        pos(1) = x_offset + 0 + (3 - 0)/1000 * (time - 550);
        pos(2) = y_offset + 4;
        pos(3) = 0;
    elseif time < 1600 % [3; 4; 0.5] up
        pos(1) = x_offset + 3;
        pos(2) = y_offset + 4;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 1550);
    elseif time < 2100 % [1.5; 4; 0.5] move
        pos(1) = x_offset + 3 + (1.5 - 3)/500 * (time - 1600);
        pos(2) = y_offset + 4;
        pos(3) = 0.5;
    elseif time < 2150 % [1.5; 4; 0] down
        pos(1) = x_offset + 1.5;
        pos(2) = y_offset + 4;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 2100);
    elseif time < 3150 % [1.5; 0; 0] write
        pos(1) = x_offset + 1.5;
        pos(2) = y_offset + 4 + (0 - 4)/1000 * (time - 2150);
        pos(3) = 0;
    elseif time <= 3200 % [1.5; 0; 0.5] up
        pos(1) = x_offset + 1.5;
        pos(2) = y_offset + 0;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 3150);
    end
end

% letter 'V'
function pos = drawV(x_offset, y_offset, prev_xyz, time)
    pos = [prev_xyz(1); prev_xyz(2); prev_xyz(3)];
    if time < 500 % [0; 4; 0.5] move
        pos(1) = prev_xyz(1) + (0 + x_offset - prev_xyz(1))/500 * time;
        pos(2) = prev_xyz(2) + (4 + y_offset - prev_xyz(2))/500 * time;
        pos(3) = prev_xyz(3) + (0.5 - prev_xyz(3))/500 * time;
    elseif time < 550 % [0; 4; 0] down
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 4;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 500);
    elseif time < 1550 % [1.5; 0; 0] write
        pos(1) = x_offset + 0 + (1.5 - 0)/1000 * (time - 550);
        pos(2) = y_offset + 4 + (0 - 4)/1000 * (time - 550);
        pos(3) = 0;
    elseif time < 2550 % [3; 4; 0] write
        pos(1) = x_offset + 1.5 + (3 - 1.5)/1000 * (time - 1550);
        pos(2) = y_offset + 0 + (4 - 0)/1000 * (time - 1550);
        pos(3) = 0;
    elseif time <= 2600 % [3; 4; 0.5] up
        pos(1) = x_offset + 3;
        pos(2) = y_offset + 4;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 2550);
    end
end

% letter 'W'
function pos = drawW(x_offset, y_offset, prev_xyz, time)
    pos = [prev_xyz(1); prev_xyz(2); prev_xyz(3)];
    if time < 500 % [0; 4; 0.5] move
        pos(1) = prev_xyz(1) + (0 + x_offset - prev_xyz(1))/500 * time;
        pos(2) = prev_xyz(2) + (4 + y_offset - prev_xyz(2))/500 * time;
        pos(3) = prev_xyz(3) + (0.5 - prev_xyz(3))/500 * time;
    elseif time < 550 % [0; 4; 0] down
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 4;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 500);
    elseif time < 1550 % [0; 0; 0] write
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 4 + (0 - 4)/1000 * (time - 550);
        pos(3) = 0;
    elseif time < 2550 % [1.5; 2; 0] write
        pos(1) = x_offset + 0 + (1.5 - 0)/1000 * (time - 1550);
        pos(2) = y_offset + 0 + (2 - 0)/1000 * (time - 1550);
        pos(3) = 0;
    elseif time < 3550 % [3; 0; 0] write
        pos(1) = x_offset + 1.5 + (3 - 1.5)/1000 * (time - 2550);
        pos(2) = y_offset + 2 + (0 - 2)/1000 * (time - 2550);
        pos(3) = 0;
    elseif time < 4550 % [3; 4; 0] write
        pos(1) = x_offset + 3;
        pos(2) = y_offset + 0 + (4 - 0)/1000 * (time - 3550);
        pos(3) = 0;
    elseif time <= 4600 % [3; 4; 0.5] up
        pos(1) = x_offset + 3;
        pos(2) = y_offset + 4;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 4550);
    end
end

% letter 'X'
function pos = drawX(x_offset, y_offset, prev_xyz, time)
    pos = [prev_xyz(1); prev_xyz(2); prev_xyz(3)];
    if time < 500 % [0; 0; 0.5] move
        pos(1) = prev_xyz(1) + (0 + x_offset - prev_xyz(1))/500 * time;
        pos(2) = prev_xyz(2) + (0 + y_offset - prev_xyz(2))/500 * time;
        pos(3) = prev_xyz(3) + (0.5 - prev_xyz(3))/500 * time;
    elseif time < 550 % [0; 0; 0] down
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 0;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 500);
    elseif time < 1550 % [3; 4; 0] write
        pos(1) = x_offset + 0 + (3 - 0)/1000 * (time - 550);
        pos(2) = y_offset + 0 + (4 - 0)/1000 * (time - 550);
        pos(3) = 0;
    elseif time < 1600 % [3; 4; 0.5] up
        pos(1) = x_offset + 3;
        pos(2) = y_offset + 4;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 1550);
    elseif time < 2100 % [3; 0; 0.5] move
        pos(1) = x_offset + 3;
        pos(2) = y_offset + 4 + (0 - 4)/500 * (time - 1600);
        pos(3) = 0.5;
    elseif time < 2150 % [3; 0; 0] down
        pos(1) = x_offset + 3;
        pos(2) = y_offset + 0;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 2100);
    elseif time < 3150 % [0; 4; 0] write
        pos(1) = x_offset + 3 + (0 - 3)/1000 * (time - 2150);
        pos(2) = y_offset + 0 + (4 - 0)/1000 * (time - 2150);
        pos(3) = 0;
    elseif time <= 3200 % [0; 4; 0.5] up
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 4;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 3150);
    end
end

% letter 'Y'
function pos = drawY(x_offset, y_offset, prev_xyz, time)
    pos = [prev_xyz(1); prev_xyz(2); prev_xyz(3)];
    if time < 500 % [0; 4; 0.5] move
        pos(1) = prev_xyz(1) + (0 + x_offset - prev_xyz(1))/500 * time;
        pos(2) = prev_xyz(2) + (4 + y_offset - prev_xyz(2))/500 * time;
        pos(3) = prev_xyz(3) + (0.5 - prev_xyz(3))/500 * time;
    elseif time < 550 % [0; 4; 0] down
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 4;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 500);
    elseif time < 1550 % [1.5; 2; 0] write
        pos(1) = x_offset + 0 + (1.5 - 0)/1000 * (time - 550);
        pos(2) = y_offset + 4 + (2 - 4)/1000 * (time - 550);
        pos(3) = 0;
    elseif time < 2550 % [3; 4; 0] write
        pos(1) = x_offset + 1.5 + (3 - 1.5)/1000 * (time - 1550);
        pos(2) = y_offset + 2 + (4 - 2)/1000 * (time - 1550);
        pos(3) = 0;
    elseif time < 2600 % [3; 4; 0.5] up
        pos(1) = x_offset + 3;
        pos(2) = y_offset + 4;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 2550);
    elseif time < 3100 % [1.5; 2; 0.5] move
        pos(1) = x_offset + 3 + (1.5 - 3)/500 * (time - 2600);
        pos(2) = y_offset + 4 + (2 - 4)/500 * (time - 2600);
        pos(3) = 0.5;
    elseif time < 3150 % [1.5; 2; 0] down
        pos(1) = x_offset + 1.5;
        pos(2) = y_offset + 2;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 3100);
    elseif time < 4150 % [1.5; 0; 0] write
        pos(1) = x_offset + 1.5;
        pos(2) = y_offset + 2 + (0 - 2)/1000 * (time - 3150);
        pos(3) = 0;
    elseif time <= 4200 % [1.5; 0; 0.5] up
        pos(1) = x_offset + 1.5;
        pos(2) = y_offset + 0;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 4150);
    end
end

% letter 'Z'
function pos = drawZ(x_offset, y_offset, prev_xyz, time)
    pos = [prev_xyz(1); prev_xyz(2); prev_xyz(3)];
    if time < 500 % [0; 4; 0.5] move
        pos(1) = prev_xyz(1) + (0 + x_offset - prev_xyz(1))/500 * time;
        pos(2) = prev_xyz(2) + (4 + y_offset - prev_xyz(2))/500 * time;
        pos(3) = prev_xyz(3) + (0.5 - prev_xyz(3))/500 * time;
    elseif time < 550 % [0; 4; 0] down
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 4;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 500);
    elseif time < 1550 % [2; 4; 0] write
        pos(1) = x_offset + 0 + (2 - 0)/1000 * (time - 550);
        pos(2) = y_offset + 4;
        pos(3) = 0;
    elseif time < 2550 % [0; 0; 0] write
        pos(1) = x_offset + 2 + (0 - 2)/1000 * (time - 1550);
        pos(2) = y_offset + 4 + (0 - 4)/1000 * (time - 1550);
        pos(3) = 0;
    elseif time < 3550 % [3; 0; 0] write
        pos(1) = x_offset + 0 + (3 - 0)/1000 * (time - 2550);
        pos(2) = y_offset + 0;
        pos(3) = 0;
    elseif time <= 3600 % [3; 0; 0.5] up
        pos(1) = x_offset + 3;
        pos(2) = y_offset + 0;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 3550);
    end
end
%% Drawing Functions
