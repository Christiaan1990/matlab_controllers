%% info
% the goal of this script is to find the best model for some data
% method: Auto-Regressive model with eXogenous input (ARX)

%% clear
clc; clear;

%% create actual model (cont. transfer func.)
K = 100; T = 2; Gp_c = tf(K,[T 1]);
Ts = 0.1; Gp_d = c2d(Gp_c, Ts);
b_true = K*Ts/T; a_true = Ts/T-1; %https://nl.mathworks.com/help/sps/ref/lowpassfilterdiscreteorcontinuous.html

%% create data
t_max = 4*T;
t = (0:Ts:t_max)'; N = length(t);
u = ones(N,1);
y_true = lsim(Gp_c,u,t,0); %y_true = step(Gp_c,t);
y = y_true + 1*randn(N,1)*sqrt(K)*0.1;

%% apply ARX

na_vec = 1:5;
nb_vec = 0:5;
[NA,NB] = meshgrid(na_vec,nb_vec);
NLL_arx = zeros(size(NA));
for grid_row_i=1:length(nb_vec)
    for grid_col_i=1:length(na_vec)
        
        % create regression matrix
        na = NA(grid_row_i,grid_col_i);
        nb = NB(grid_row_i,grid_col_i);
        M_arx = zeros(length(u)-max(na,nb),na+nb);
        for col_i = 1:(na+nb)
            if col_i<=na
                M_arx(:,col_i) = -y(max(na,nb)+1-col_i:end-col_i,1);
            else
                M_arx(:,col_i) = u(max(na,nb)+1-col_i+na:end-col_i+na,1);
            end
        end

        % calculate the estimated parameters a and b via least squares
        y_arx = y(max(na,nb)+1:end,1);
        th_arx = pinv(M_arx'*M_arx)*M_arx'*y_arx;
        a_arx = th_arx(1:na); b_arx = th_arx(na+1:na+nb);
        y_arx_1 = [y(1:max(na,nb)) ; M_arx * th_arx];
        NLL_arx(grid_row_i,grid_col_i) = 0.5*(y_arx-M_arx*th_arx)'*(y_arx-M_arx*th_arx);

    end
end

%% plot the data
surf(NA,NB,NLL_arx)
legend('models');
xlabel('na [-]'); ylabel('nb [-]'); zlabel('NLL [-]')
hold off;