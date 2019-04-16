% �öγ�������ʵ�֣�˫��RINS������ IMU ���
clear all
syms wie L w t
%% ��ֹ��ת�� bϵ�����
Wie = [0; wie*cos(L); wie*sin(L)];
Wzb = [0; wie*cos(L); wie*sin(L)+w]; W_zb = [0; wie*cos(L); wie*sin(L)-w];
Wyb = [0; wie*cos(L)+w; wie*sin(L)]; W_yb = [0; wie*cos(L)-w; wie*sin(L)];
%% C^s0_b
CA = diag([1,1,1]); CB = diag([-1,-1,1]);
CC = diag([1,-1,-1]); CD = diag([-1,1,-1]);
%% C^s_s0
Czs = [cos(w*t),  sin(w*t), 0;
       -sin(w*t), cos(w*t), 0;
       0,         0,        1]; % �� +Zs����ת��
C_zs = Czs'; % �� -Zs����ת
Cys = [cos(w*t), 0, -sin(w*t);
       0,        1, 0;
       sin(w*t), 0. cos(w*t)]; % �� +Ys����ת��
C_ys = Cys'; % �� -Ys����ת
%% ���� ��λ���� sϵ�µ� IMU �������
imuA = CA*Wie; % Aλ�� Ts 
imuB = CB*Wie; % Bλ�� Ts
imuC = CC*Wie; % Cλ�� Ts
imuD = CD*Wie; % Dλ�� Ts
%% ���� ������ ��ʱ�� sϵ�µ� IMU �������
imu1 = Czs*CA*Wzb; % ����1 A->B +Zb +Zs
imu2 = C_ys*CB*Wyb; % ����2 B->C +Yb -Ys
imu3 = Czs*CC*W_zb; % ����3 C->D -Zb +Zs
imu4 = C_ys*CD*W_yb; % ����4 D->A -Yb -Ys
imu5 = C_ys*CA*W_yb; % ����5 A->D -Yb -Ys
imu6 = Czs*CD*W_zb; % ����6 D->C -Zb +Zs
imu7 = C_ys*CC*Wyb; % ����7 C->B +Yb -Ys
imu8 = Czs*CB*Wzb; % ����8 B->A +Zb +Zs

imu9 = C_zs*CA*W_zb; % ����9 A->B -Zb -Zs
imu10 = Cys*CB*W_yb; % ����10 B->C -Yb +Ys
imu11 = C_zs*CC*Wzb; % ����11 C->D Zb -Zs
imu12 = Cys*CD*Wyb; % ����12 D->A Yb Ys
imu13 = Cys*CA*Wyb; % ����13 A->D Yb Ys
imu14 = C_zs*CD*Wzb; % ����14 D->C Zb -Zs
imu15 = Cys*CC*W_yb; % ����15 C->B -Yb +Ys
imu16 = C_zs*CB*W_zb; % ����16 B->A -Zb -Zs