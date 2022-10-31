clc; clear;

%% properties simulation
dt = 0.01;

%% properties system

% mass
m = 0.5;  %[kg]

% friction
Cd = 0.5;           % [-]        air drag fric. coef.
rho_air = 1.225;    % [kg/m^3]  air density 
S = pi*0.2^2;       % [m^2]     surface area

% gravity
g = 9.807;     % [m/s^2]   grav. const.

%% controller

% impedance controller 
wn_imp = 50;
zeta_imp = 1;
K_imp = wn_imp^2 * m;
D_imp = 2*zeta_imp*wn_imp*m;

% admittance controller
wn_adm = 5;
zeta_adm = 1;
M_adm = 0.1;
K_adm = wn_adm^2 * M_adm;
D_adm = 2*zeta_adm*wn_adm*M_adm;

%% trajectory
dt_traj = 3;
x_des = (0:dt:dt_traj).^2;
dx_des = 2 * (0:dt:dt_traj);
ddx_des = 2;