%% 产生定点的 IMU 数据，符号计算
clear all
syms wie L w t Kxy Kxz Kyx Kyz Kzx Kzy theta %theta = w*t

%% 安装误差公式推导 
SA = [0, Kxy, Kxz;
      Kyx, 0, Kyz;
      Kzx, Kzy, 0];
% 同样适用于 标度因数误差公式推导（只需以KA替代SA）
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
       0,         0,        1]; % 绕 +Zs轴旋转，
C_zs = Czs.'; % 绕 -Zs轴旋转
Cys = [cos(theta), 0, -sin(theta);
       0,        1, 0;
       sin(theta), 0. cos(theta)]; % 绕 +Ys轴旋转，
C_ys = Cys.';  % 绕 -Ys轴旋转
%% Wzb
Wzb = [0; wie*cos(L); wie*sin(L)+w];
W_zb = [0; wie*cos(L); wie*sin(L)-w];
Wyb = [0; wie*cos(L)+w; wie*sin(L)];
W_yb = [0; wie*cos(L)-w; wie*sin(L)];
%% 计算安装误差
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