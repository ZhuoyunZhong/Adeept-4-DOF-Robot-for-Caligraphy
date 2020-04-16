%% RBE501 Project Alphabet Trajectory generation

% time lookup table
t_A = 3700; t_E = 5200; t_F = 4200; t_H = 4800;
t_I = 1600; t_K = 4200; t_L = 2600; t_M = 4600;
t_N = 3600; t_T = 3200; t_V = 2600; t_W = 4600;
t_X = 3200; t_Y = 4200; t_Z = 3600;

t_B = 5600; t_D = 3600; t_J = 2600;
t_P = 3600; t_R = 5200; t_S = 4850; t_U = 3850;

prev_pos = [0; 0; 0.5];
figure('Name','Draw String HKLM');
scatter3(0, 0, 0);
hold on
for t1 = 0: 25: t_J
    curpos = drawJ(0, 0, prev_pos, t1);
    scatter3(curpos(1),curpos(2),curpos(3));
    if t1 == t_J
        prev_pos = curpos;
    end
end
for t2 = 0: 25: t_S
    curpos = drawS(4, 0, prev_pos, t2);
    scatter3(curpos(1),curpos(2),curpos(3));
    if t2 == t_S
        prev_pos = curpos;
    end
end
for t3= 0: 25: t_U
    curpos = drawU(8, 0, prev_pos, t3);
    scatter3(curpos(1),curpos(2),curpos(3));
    if t3 == t_U
        prev_pos = curpos;
    end
end
for t4= 0: 25: t_R
    curpos = drawR(12, 0, prev_pos, t4);
    scatter3(curpos(1),curpos(2),curpos(3));
    if t4 == t_R
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

% letter 'B'
function pos = drawB(x_offset, y_offset, prev_xyz, time)
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
    elseif time < 2050 % [1; 4; 0] write
        pos(1) = x_offset + 0 + (1 - 0)/500 * (time - 1550);
        pos(2) = y_offset + 4;
        pos(3) = 0;
    elseif time < 3050 % [1; 2; 0] right-180 r = 1 c = (1,3)  write_curve
        pos(1) = x_offset + 1 + sin((time - 2050) * pi / 1000) * 1;
        pos(2) = y_offset + 3 + cos((time - 2050) * pi / 1000) * 1;
        pos(3) = 0;
    elseif time < 3550 % [0; 2; 0] write
        pos(1) = x_offset + 1 + (0 - 1)/500 * (time - 3050);
        pos(2) = y_offset + 2;
        pos(3) = 0;
    elseif time < 4050 % [1.5; 2; 0] write
        pos(1) = x_offset + 0 + (1.5 - 0)/500 * (time - 3550);
        pos(2) = y_offset + 2;
        pos(3) = 0;
    elseif time < 5050 % [1.5; 0; 0] right-180 r = 1 c = (1.5,1)  write_curve
        pos(1) = x_offset + 1.5 + sin((time - 4050) * pi / 1000) * 1;
        pos(2) = y_offset + 1 + cos((time - 4050) * pi / 1000) * 1;
        pos(3) = 0;
    elseif time < 5550 % [0; 0; 0] write
        pos(1) = x_offset + 1.5 + (0 - 1.5)/500 * (time - 5050);
        pos(2) = y_offset + 0;
        pos(3) = 0;
    elseif time <= 5600 % [0; 0; 0.5] up
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 0;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 5550);
    end
end

% letter 'D'
function pos = drawD(x_offset, y_offset, prev_xyz, time)
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
    elseif time < 1800 % [0.5; 4; 0] write
        pos(1) = x_offset + 0 + (0.5 - 0)/250 * (time - 1550);
        pos(2) = y_offset + 4;
        pos(3) = 0;
    elseif time < 3300 % [0.5; 0; 0] right-180 r = 2 c = (0.5, 2)  write_curve
        pos(1) = x_offset + 0.5 + sin((time - 1800) * pi / 1500) * 2;
        pos(2) = y_offset + 2 + cos((time - 1800) * pi / 1500) * 2;
        pos(3) = 0;
    elseif time < 3550 % [0; 0; 0] write
        pos(1) = x_offset + 0.5 + (0 - 0.5)/250 * (time - 3300);
        pos(2) = y_offset + 0;
        pos(3) = 0;
    elseif time <= 3600 % [0; 0; 0.5] up
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 0;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 3550);
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

% letter 'J'
function pos = drawJ(x_offset, y_offset, prev_xyz, time)
    pos = [prev_xyz(1); prev_xyz(2); prev_xyz(3)];
    if time < 500 % [2; 4; 0.5] move
        pos(1) = prev_xyz(1) + (2 + x_offset - prev_xyz(1))/500 * time;
        pos(2) = prev_xyz(2) + (4 + y_offset - prev_xyz(2))/500 * time;
        pos(3) = prev_xyz(3) + (0.5 - prev_xyz(3))/500 * time;
    elseif time < 550 % [2; 4; 0] down
        pos(1) = x_offset + 2;
        pos(2) = y_offset + 4;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 500);
    elseif time < 1550 % [2; 1; 0] write
        pos(1) = x_offset + 2;
        pos(2) = y_offset + 4 + (1 - 4)/1000 * (time - 550);
        pos(3) = 0;
    elseif time < 2550 % [0; 1; 0] bottom-180 r = 1 c = (1 , 1)  write_curve
        pos(1) = x_offset + 1 + cos((time - 1550) * pi / 1000) * 1;
        pos(2) = y_offset + 1 - sin((time - 1550) * pi / 1000) * 1;
        pos(3) = 0;
    elseif time <= 2600 % [0; 1; 0.5] up
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 1;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 2550);
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

% letter 'P'
function pos = drawP(x_offset, y_offset, prev_xyz, time)
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
    elseif time < 2050 % [1.5; 4; 0] write
        pos(1) = x_offset + 0 + (1.5 - 0)/500 * (time - 1550);
        pos(2) = y_offset + 4;
        pos(3) = 0;
    elseif time < 3050 % [1.5; 2; 0] right-180 r = 1 c = (1.5 ,3)  write_curve
        pos(1) = x_offset + 1.5 + sin((time - 2050) * pi / 1000) * 1;
        pos(2) = y_offset + 3 + cos((time - 2050) * pi / 1000) * 1;
        pos(3) = 0;
    elseif time < 3550 % [0; 2; 0] write
        pos(1) = x_offset + 1.5 + (0 - 1.5)/500 * (time - 3050);
        pos(2) = y_offset + 2;
        pos(3) = 0;
    elseif time <= 3600 % [0; 2; 0.5] up
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 2;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 3550);
    end
end

% letter 'R'
function pos = drawR(x_offset, y_offset, prev_xyz, time)
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
    elseif time < 2050 % [1; 4; 0] write
        pos(1) = x_offset + 0 + (1 - 0)/500 * (time - 1550);
        pos(2) = y_offset + 4;
        pos(3) = 0;
    elseif time < 3050 % [1; 2; 0] right-180 r = 1 c = (1 ,3)  write_curve
        pos(1) = x_offset + 1 + sin((time - 2050) * pi / 1000) * 1;
        pos(2) = y_offset + 3 + cos((time - 2050) * pi / 1000) * 1;
        pos(3) = 0;
    elseif time < 3550 % [0; 2; 0] write
        pos(1) = x_offset + 1 + (0 - 1)/500 * (time - 3050);
        pos(2) = y_offset + 2;
        pos(3) = 0;
    elseif time < 3600 % [0; 2; 0.5] up
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 2;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 3550);
    elseif time < 4100 % [1; 2; 0.5] move
        pos(1) = x_offset + 0 + (1 - 0)/500 * (time - 3600);
        pos(2) = y_offset + 2;
        pos(3) = 0.5;
    elseif time < 4150 % [1; 2; 0] down
        pos(1) = x_offset + 1;
        pos(2) = y_offset + 2;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 4100);
    elseif time < 5150 % [3; 0; 0] write
        pos(1) = x_offset + 1 + (3 - 1)/1000 * (time - 4150);
        pos(2) = y_offset + 2 + (0 - 2)/1000 * (time - 4150);
        pos(3) = 0;
    elseif time <= 5200% [3; 0; 0.5] up
        pos(1) = x_offset + 3;
        pos(2) = y_offset + 0;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 5150);
    end
end

% letter 'S'
function pos = drawS(x_offset, y_offset, prev_xyz, time)
    pos = [prev_xyz(1); prev_xyz(2); prev_xyz(3)];
    if time < 500 % [2.707; 3.707; 0.5] move
        pos(1) = prev_xyz(1) + (2.707 + x_offset - prev_xyz(1))/500 * time;
        pos(2) = prev_xyz(2) + (3.707 + y_offset - prev_xyz(2))/500 * time;
        pos(3) = prev_xyz(3) + (0.5 - prev_xyz(3))/500 * time;
    elseif time < 550 % [2.707; 3.707; 0] down
        pos(1) = x_offset + 2.707;
        pos(2) = y_offset + 3.707;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 500);
    elseif time < 800 % [2; 4; 0] right-45 r = 1 c = (2,3)  write_curve
        pos(1) = x_offset + 2 + cos(pi/4 + (time - 550) * pi / 1000) * 1;
        pos(2) = y_offset + 3 + sin(pi/4 + (time - 550) * pi / 1000) * 1;
        pos(3) = 0;
    elseif time < 1300 % [1; 4; 0] write
        pos(1) = x_offset + 2 + (1 - 2)/500 * (time - 800);
        pos(2) = y_offset + 4;
        pos(3) = 0;
    elseif time < 2300 % [1; 2; 0] left-180 r = 1 c = (1,3)  write_curve
        pos(1) = x_offset + 1 - sin((time - 1300) * pi / 1000) * 1;
        pos(2) = y_offset + 3 + cos((time - 1300) * pi / 1000) * 1;
        pos(3) = 0;
    elseif time < 2800 % [2; 2; 0] write
        pos(1) = x_offset + 1 + (2 - 1)/500 * (time - 2300);
        pos(2) = y_offset + 2;
        pos(3) = 0;
    elseif time < 3800 % [2; 0; 0] right-180 r = 1 c = (2,1)  write_curve
        pos(1) = x_offset + 2 + sin((time - 2800) * pi / 1000) * 1;
        pos(2) = y_offset + 1 + cos((time - 2800) * pi / 1000) * 1;
        pos(3) = 0;
    elseif time < 4300 % [1; 0; 0] write
        pos(1) = x_offset + 2 + (1 - 2)/500 * (time - 3800);
        pos(2) = y_offset + 0;
        pos(3) = 0;
    elseif time < 4800 % [0; 1; 0] left-90 r = 1 c = (1,1)  write_curve
        pos(1) = x_offset + 1 - sin((time - 4300) * pi / 1000) * 1;
        pos(2) = y_offset + 1 - cos((time - 4300) * pi / 1000) * 1;
        pos(3) = 0;
    elseif time <= 4850 % [0; 1; 0.5] up
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 1;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 4800);
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

% letter 'U'
function pos = drawU(x_offset, y_offset, prev_xyz, time)
    pos = [prev_xyz(1); prev_xyz(2); prev_xyz(3)];
    if time < 500 % [0; 4; 0.5] move
        pos(1) = prev_xyz(1) + (0 + x_offset - prev_xyz(1))/500 * time;
        pos(2) = prev_xyz(2) + (4 + y_offset - prev_xyz(2))/500 * time;
        pos(3) = prev_xyz(3) + (0.5 - prev_xyz(3))/500 * time;
    elseif time < 550 % [0; 4; 0] down
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 4;
        pos(3) = 0.5 + (0 - 0.5)/50 * (time - 500);
    elseif time < 1550 % [0; 1; 0] write
        pos(1) = x_offset + 0;
        pos(2) = y_offset + 4 + (1 - 4)/1000 * (time - 550);
        pos(3) = 0;
    elseif time < 2050 % [1; 0; 0] left-90 r = 1 c = (1,1)  write_curve
        pos(1) = x_offset + 1 - cos((time - 1550) * pi / 1000) * 1;
        pos(2) = y_offset + 1 - sin((time - 1550) * pi / 1000) * 1;
        pos(3) = 0;
    elseif time < 2300 % [1.5; 0; 0] write
        pos(1) = x_offset + 1 + (1.5 - 1)/250 * (time - 2050);
        pos(2) = y_offset + 0;
        pos(3) = 0;
    elseif time < 2800 % [2.5; 1; 0] right-90 r = 1 c = (1.5,1)  write_curve
        pos(1) = x_offset + 1.5 + sin((time - 2300) * pi / 1000) * 1;
        pos(2) = y_offset + 1 - cos((time - 2300) * pi / 1000) * 1;
        pos(3) = 0;
    elseif time < 3800 % [2.5; 4; 0] write
        pos(1) = x_offset + 2.5;
        pos(2) = y_offset + 1 + (4 - 1)/1000 * (time - 2800);
        pos(3) = 0;
    elseif time <= 3850 % [2.5; 4; 0.5] up
        pos(1) = x_offset + 2.5;
        pos(2) = y_offset + 4;
        pos(3) = 0 + (0.5 - 0)/50 * (time - 3800);
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
