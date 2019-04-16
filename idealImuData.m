% 该段程序用于实现，双轴RINS的理想 IMU 输出
clear all
syms wie L w t
%% 静止、转动 b系下输出
Wie = [0; wie*cos(L); wie*sin(L)];
Wzb = [0; wie*cos(L); wie*sin(L)+w]; W_zb = [0; wie*cos(L); wie*sin(L)-w];
Wyb = [0; wie*cos(L)+w; wie*sin(L)]; W_yb = [0; wie*cos(L)-w; wie*sin(L)];
%% C^s0_b
CA = diag([1,1,1]); CB = diag([-1,-1,1]);
CC = diag([1,-1,-1]); CD = diag([-1,1,-1]);
%% C^s_s0
Czs = [cos(w*t),  sin(w*t), 0;
       -sin(w*t), cos(w*t), 0;
       0,         0,        1]; % 绕 +Zs轴旋转，
C_zs = Czs'; % 绕 -Zs轴旋转
Cys = [cos(w*t), 0, -sin(w*t);
       0,        1, 0;
       sin(w*t), 0. cos(w*t)]; % 绕 +Ys轴旋转，
C_ys = Cys'; % 绕 -Ys轴旋转
%% 产生 各位置下 s系下的 IMU 理想输出
imuA = CA*Wie; % A位置 Ts 
imuB = CB*Wie; % B位置 Ts
imuC = CC*Wie; % C位置 Ts
imuD = CD*Wie; % D位置 Ts
%% 产生 各次序 各时刻 s系下的 IMU 理想输出
imu1 = Czs*CA*Wzb; % 次序1 A->B +Zb +Zs
imu2 = C_ys*CB*Wyb; % 次序2 B->C +Yb -Ys
imu3 = Czs*CC*W_zb; % 次序3 C->D -Zb +Zs
imu4 = C_ys*CD*W_yb; % 次序4 D->A -Yb -Ys
imu5 = C_ys*CA*W_yb; % 次序5 A->D -Yb -Ys
imu6 = Czs*CD*W_zb; % 次序6 D->C -Zb +Zs
imu7 = C_ys*CC*Wyb; % 次序7 C->B +Yb -Ys
imu8 = Czs*CB*Wzb; % 次序8 B->A +Zb +Zs

imu9 = C_zs*CA*W_zb; % 次序9 A->B -Zb -Zs
imu10 = Cys*CB*W_yb; % 次序10 B->C -Yb +Ys
imu11 = C_zs*CC*Wzb; % 次序11 C->D Zb -Zs
imu12 = Cys*CD*Wyb; % 次序12 D->A Yb Ys
imu13 = Cys*CA*Wyb; % 次序13 A->D Yb Ys
imu14 = C_zs*CD*Wzb; % 次序14 D->C Zb -Zs
imu15 = Cys*CC*W_yb; % 次序15 C->B -Yb +Ys
imu16 = C_zs*CB*W_zb; % 次序16 B->A -Zb -Zs