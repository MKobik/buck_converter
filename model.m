clc, clear, close all;

% This code simulates the control of a buck converter using different methods:
% 1. PID control:
%    The system is modeled using a transfer function (TF) representation of the buck converter,
%    and a PID controller is used to regulate its output.
%    PID controller and it's step response is modeled in simulink file "PID".
% 2. State space control:
%    The system is also represented in state space form, and two control strategies are employed:
%    a) LQR (Linear Quadratic Regulator) control, and
%    b) Pole placement control.

% buck converter parameters
Vi = 100;
R = 1;
L = 10e-3;
C = 50e-3;

% transfer function of the buck converter (plant)
TF_num = [Vi./(L*C)];
TF_den = [1, 1/(R*C), 1/(L*C)];

% state space matricies of the buck converter
A = [0, -1/L; 1/C, -1/R*C];
B = [Vi/L; 0];
C_mat = [0, 1];
D = 0;

% check poles for stability
eig(A);

% state space model
sys = ss(A, B, C_mat, D); 

%____________________________________________

% LQR

% cost function matricies
Q = [1000 0;  % penalize inductor current
     0 10];   % penalize output voltage error
R = [0.001];  % penalize duty cycle

% find K based on LQR
K = lqr(A, B, Q, R);

% new closed loop system
Acl = A - B*K;
sys_cl = ss(Acl, B, C_mat ,D);

% solve for Kr so the system has gain of 1
Kdc = dcgain(sys_cl);
Kr = 1/Kdc;

% LQR system
sys_cl_scaled = ss(Acl, B*Kr, C_mat, D);
step(sys_cl_scaled)
title('Step Response with LQR')

%____________________________________________

% % POLE PLACEMENT (uncomment before running)
% 
% % place poles
% p = [-10, -20]; 
% 
% % create K matrix with desired poles
% K = place(A, B, p); 
% 
% % new closed loop A matrix
% Acl = A - B*K;
% 
% % new closed loop system
% sys_cl = ss(Acl, B, C_mat, D);
% %step(sys_cl);
% 
% % solve for Kr so the system has gain of 1
% Kdc = dcgain(sys_cl);
% Kr = 1/Kdc;
% 
% % pole placement system
% sys_cl_scaled = ss(Acl, B*Kr, C_mat, D);
% step(sys_cl_scaled)
% title('Step Response with pole placement')