%% ��������� IMU ���ݣ����ż���
clear all
syms wie L w t Kxy Kxz Kyx Kyz Kzx Kzy theta %theta = w*t

%% ��װ��ʽ�Ƶ� 
SA = [0, Kxy, Kxz;
      Kyx, 0, Kyz;
      Kzx, Kzy, 0];
% ͬ�������� ���������ʽ�Ƶ���ֻ����KA���SA��
KA = [Kx, 0, 0;
      0, Ky, 0;
      0, 0, Kz];
%% C^s0_b
CA = diag([1,1,1]);
CB = diag([-1,-1,1]);
CC = diag([1,-1,-1]);
CD = diag([-1,1,-1]);
%% C^s_s0
Czs = [cos(theta),  sin(theta), 0;
       -sin(theta), cos(theta), 0;
       0,         0,        1]; % �� +Zs����ת��
C_zs = Czs.'; % �� -Zs����ת
Cys = [cos(theta), 0, -sin(theta);
       0,        1, 0;
       sin(theta), 0. cos(theta)]; % �� +Ys����ת��
C_ys = Cys.';  % �� -Ys����ת
%% Wzb
Wzb = [0; wie*cos(L); wie*sin(L)+w];
W_zb = [0; wie*cos(L); wie*sin(L)-w];
Wyb = [0; wie*cos(L)+w; wie*sin(L)];
W_yb = [0; wie*cos(L)-w; wie*sin(L)];
%% ���㰲װ���
dw1 = CA*Czs.'*SA*Czs*CA*Wzb;
dw2 = CB*C_ys.'*SA*C_ys*CB*Wyb;
dw3 = CC*Czs.'*SA*Czs*CC*W_zb;
dw4 = CD*C_ys.'*SA*C_ys*CD*W_yb;
dw5 = CA*C_ys.'*SA*C_ys*CA*W_yb;
dw6 = CD*Czs.'*SA*Czs*CD*W_zb;
dw7 = CC*C_ys.'*SA*C_ys*CC*Wyb;
dw8 = CB*Czs.'*SA*Czs*CB*Wzb;
%
dw9 = CA*C_zs.'*SA*C_zs*CA*W_zb;
dw10 = CB*Cys.'*SA*Cys*CB*W_yb;
dw11 = CC*C_zs.'*SA*C_zs*CC*Wzb;
dw12 = CD*Cys.'*SA*Cys*CD*Wyb;
dw13 = CA*Cys.'*SA*Cys*CA*Wyb;
dw14 = CD*C_zs.'*SA*C_zs*CD*Wzb;
dw15 = CC*Cys.'*SA*Cys*CC*W_yb;
dw16 = CB*C_zs.'*SA*C_zs*CB*W_zb;