clc; clear;

%% info
% http://www.bylobvgren.com/projects/mathematic-modelling-of-a-dc-motor/
% https://leo.technion.ac.il/courses/IC/Lectures/lect02handout.pdf
% https://www.irjet.net/archives/V6/i7/IRJET-V6I7157.pdf
% 1 rpm = 1 rot/min = 2*pi rad/min = 2*pi/60 rad/s = 1/60 rps
% 1 V/rpm = 60 V/rps = 60/(2*pi) V/(rad/s)

% to do: https://www.mdpi.com/2076-3417/10/12/4329/pdf?version=1593424115
%        https://www.youtube.com/watch?v=IolG5V_skv8

%% properties simulation
dt = 0.01;

%% properties system
% motor: Faulhaber 2342-024-CR
% datasheet: https://www.faulhaber.com/fileadmin/Import/Media/EN_2342_CR_DFF.pdf

% electric properties
Ra = 7.1;                   % [ohm] terminal resistance (resistance in coil armature)
La = 265e-6;                % [H] rotor inductance
Km = 26.1e-3;               % [Nm/A] torque constant (Tm=Km*i)
Kb = 2.73e-3*(60/(2*pi));   % [V*s/rad] back-EMF constant

% mechanical properties
J = 5.8e-7;                 % [N*m*s^2/rad] rotor inertia (without load)
b = 5.8/6*1e-4;             % [N*m*s/rad] coulomb friction

% time constants
tc_arm = La/Ra;             % [s] armature time const.
tc_mech = J/b;              % [s] mechanical time const.

% limitations
ddth_max = 150e3;           % [rad/s^2] max. angular acceleration

% if tc_arm<<tc_mech
simpDC_gain = Km/(Ra*b+Kb*Km);
simpDC_timeconst = (Ra*J/(Ra*b+Kb*Km));

%% controller motor current: PI (complex dc motor)
% Y = I(s), Y_ref = I_ref(s)
% Gp = I(s)/V_in(s) = 1/(L*s + R) = K/(T*s + 1)
% Gc = V_in(s)/(I_des-I) = K_P + K_I/s
% Y/Y_ref = Gp*Gc / (1 + Gp*Gc) = Gpn*Gc / (Gpd + Gpn*Gc)
%   = (K_P + K_I/s)*K / ((T*s + 1) + (K_P + K_I/s)*K)
%   = (K_P*s + K_I)*K / (T*s^2 + (1 + K_P*K)*s + K_I*K)
%   -> so Y(0)/Y_ref(0) = 1 
%   wn^2 = K_I*K/T -> K_I = f(wn)
%   (1 + K_P*K) = 2 * zeta * wn * T -> K_P = f(wn, zeta)
K = 1/Ra;
T = tc_arm;
wn_m_cur = 20000;           % should be order size 1/tc_arm
zeta_m_cur = 1;             % [-] relative damping (1=critical damping)
K_I_cur = wn_m_cur^2 * T/K;
K_P_cur = (2*zeta_m_cur*wn_m_cur*T - 1)/K;
if K_P_cur < 0
    warning("K_P_cur < 0 -> try choosing a larger wn")
end

%% controller motor velocity: PI (simple dc motor)
% Y = dth(s, Y_ref = dth_des(s)
% Gp = dth(s)/V_in(s) = K/(T*s + 1)
% Gc = V_in(s)/(dth_des-dth) = K_P + K_I/s
% Y/Y_ref = Gp*Gc / (1 + Gp*Gc) = Gpn*Gc / (Gpd + Gpn*Gc)
%   = (K_P + K_I/s)*K / ((T*s + 1) + (K_P + K_I/s)*K)
%   = (K_P*s + K_I)*K / (T*s^2 + (1 + K_P*K)*s + K_I*K)
%   -> so Y(0)/Y_ref(0) = 1 
%   wn^2 = K_I*K/T -> K_I = f(wn)
%   (1 + K_P*K) = 2 * zeta * wn * T -> K_P = f(wn, zeta)
K = simpDC_gain;
T = simpDC_timeconst;
wn_m_vel = 500;             % should be order size 1/simpDC_timeconst
zeta_m_vel = 1;             % [-] relative damping (1=critical damping)
K_I_vel = wn_m_vel^2 * T/K;
K_P_vel = (2*zeta_m_vel*wn_m_vel*T - 1)/K;
if K_P_vel < 0
    warning("K_P_cur < 0 -> try choosing a larger wn")
end

%% controller 1 motor position: PD (simple dc motor)
% Y = th(s, Y_ref = th_des(s)
% Gp = th(s)/V_in(s) = 1/s * K/(T*s + 1)
% Gc = V_in(s)/(th_des-th) = K_P + K_I/s + K_D/s
% Y/Y_ref = Gp*Gc / (1 + Gp*Gc) = Gpn*Gc / (Gpd + Gpn*Gc)
%   = (K_P + K_I/s + K_D*s)*K / ((T*s^2 + s) + (K_P + K_I/s + K_D*s)*K)
%   = (K_P + K_I/s + K_D*s)*K / (T*s^2 + (1 + K_D*K)*s+ K_P*K + K*K_I/s)
%   -> K_I should be zero, such that the denominator is polynome of order 2
%   = (K_P + K_D*s)*K / (T*s^2 + (1 + K_D*K)*s+ K_P*K)
%   = (K_P + K_D*s) / (T/K*s^2 + (1/K + K_D)*s+ K_P)
%   -> if t-> inf, then s=0 -> Y(0)/Y_ref(0) = 1 
%   wn^2 = K_P*K/T -> K_P = wn^2*T/K
%   (1 + K_D*K) = 2 * zeta * wn * T -> K_D = (2*zeta*wn*T - 1)/K
% note: if you rewrite the closed-loop controller: 
%        then: 1*(ddth-ddth_d) + K_D*dth + wn^2*(th-th_d) = 0
K = simpDC_gain;
T = simpDC_timeconst;
wn_m_pos = 250;             % should be order size 1/simpDC_timeconst
zeta_m_pos = 1;             % [-] relative damping (1=critical damping)
K_P_pos = wn_m_pos^2 * T/K;
K_I_pos = 0;
K_D_pos = (2*zeta_m_pos*wn_m_pos*T - 1)/K;
if K_D_pos < 0
    warning("K_D_pos < 0 -> try choosing a larger wn")
end

%% controller 2 motor position: passivity (simple dc motor)
% th(s)/V_in = 1/s * w(s)/V_in = 1/s * K/(T*s + 1)
% open-loop: T*ddth + dth = K*V_in -> now V_in has to be defined
% control law: V_in = T/K*(+ddth_d + 1/T*dth - K_D*(dth-dth_d) - K_P*(th-th_d))
% closed-loop: 1*(ddth-ddth_d) + K_D*(dth-dth_d) + K_P*(th-th_d) = 0
%   wn^2 = K_P -> K_P = wn^2
%   K_D = (2*zeta*wn*1) -> K_D = 2*zeta*wn
K = simpDC_gain;
T = simpDC_timeconst;
wn_m_pos = 100;             
zeta_m_pos = 1;             % [-] relative damping (1=critical damping)
K_P_pos_pass = wn_m_pos^2;
K_D_pos_pass = 2*zeta_m_pos*wn_m_pos;

%% controller 3 motor position: admittance control (simple dc motor)
% M*ddx + D*dx + K*x = F
% wn_OpenLoop = sqrt(K/M)
% 2*zeta_OpenLoop*wn_OpenLoop*M = D
zeta_adm = 1;
wn_adm = 10;
M_adm = 1;
D_adm = 2*zeta_adm*wn_adm*M_adm;
K_adm = wn_adm^2*M_adm;

%%
% load
m = 1;                      % [kg] mass pendulum
L = 1;                      % [m] length pendulum
g = 9.807;                  % [m/s^2]   grav. const.

%% trajectory
dt_traj = 3;
x_des = (0:dt:dt_traj).^2;
dx_des = 2 * (0:dt:dt_traj);
ddx_des = 2;