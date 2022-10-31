%% info
% this is parameter estimation of a TF in the laplace domain
% method: Auto-Regressive model with eXogenous input (ARX)
% A(z) = 1 + a_1*z^(-1) + ... + a_na*z^(-na)        -> eq(1)
% B(z) = 0 + b_1*z^(-nk) + ... + b_nb*z^(-nb-nk+1)  -> eq(2) (nk = input lag)
% y(k) = 1/A(z) * ( B(z)*u(k) + e(k))               -> eq(3)
% theta = [a_1 ... a_na b_1 ... b_nb]'              -> eq(4)
% rewrite eq(3) as: A*y(k) = B(z)*u(k)) + e(k)      -> eq(5)
% now insert eq(1) and eq(2) in eq(5)
%   [1 + a_1*z^(-1) + ... + a_na*z^(-na)] * y(k) = [b_1*z^(-nk) + ... + b_nb*z^(-nb-nk+1)]*u(k)) + e(k)
% y(k) + a_1*y(k-1) + ... + a_na*y(k-na)         = [b_1*u(k-nk) + ... + b_nb*u(k-nb-nk+1)] + e(k)
% y(k) = -a_1*y(k-1) - ... - a_na*y(k-na) + b_1*u(k-nk) + ... + b_nb*u(k-nb-nk+1) + e(k)
% y_hat(k) = -a_1_hat*y(k-1) - ... - a_na_hat*y(k-na) + b_1_hat*u(k-1) + ... + b_nb_hat*u(k-nb)
% [y(N-1)] = [-y(N-2) ... -y(N-na-1) u(N-1)  ... u(N-nb - 1) ]*[a_1 ... a_na b_1 ... b_nb]'
% [y(N)]   = [-y(N-1) ... -y(N-na)   u(N-nk) ... u(N-nb-nk+1)]*[a_1 ... a_na b_1 ... b_nb]'
% Gp_d_arx(z) = tf([b_1 ... b_nb],[1 a_1 ... a_na],Ts);
% note: G(Z) = GN(z)/GD(z) = (z^(-p)*GN(z))/(z^(-p)*GD(z)) = G(z^-1)
% https://www.ni.com/nl-nl/support/documentation/supplemental/06/selecting-a-model-structure-in-the-system-identification-process.html
% https://www.youtube.com/watch?v=JRWjO2oiy3Y

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

% create the observation matrix M_arx
na = 1; nb = 1; n_k = 1;
M_arx = zeros(N-max(na,nb+n_k-1),na+nb);
for col_i = 1:size(M_arx,2)
    if col_i<=na
        M_arx(:,col_i) = -y(max(na,nb+n_k-1)+1-col_i:end-col_i,1);
    else
        M_arx(:,col_i) = u(max(na,nb+n_k-1)+1-col_i+na+1-n_k:end-col_i+na+1-n_k,1);
    end
end

% calculate the estimated parameters a and b via least squares
y_arx = y(max(na,nb+n_k-1)+1:end,1);
th_arx = pinv(M_arx'*M_arx)*M_arx'*y_arx;
a_arx = th_arx(1:na); b_arx = th_arx(na+1:na+nb);
y_arx_1 = [y(1:max(na,nb+n_k-1)) ; M_arx * th_arx];
NLL_arx = 0.5*(y_arx-M_arx*th_arx)'*(y_arx-M_arx*th_arx);
COVAR_th = pinv(M_arx'*M_arx)*1/(length(y_arx)-1)*sum((y_arx-M_arx*th_arx).^2);

% get the transfer function and perform a check
Gp_d_arx = tf(b_arx',[1 a_arx'],Ts);
y_arx_2 = lsim(Gp_d_arx,u,t,0);

% check the results via a simulation
y_arx_3 = zeros(N,1);
for ti=1:N
    if ti<=max(na,nb)
        y_arx_3(ti) = y(ti);
    else
        if isempty(b_arx)~=1
            y_arx_3(ti) = -a_arx*y_arx_3(ti-1) + b_arx*1;
        else
            y_arx_3(ti) = -a_arx*y_arx_3(ti-1);
        end
    end
end

%% plot the data
plot(t,y,'o','Color','k'); hold on;
plot(t,y_arx_1,'-','Color','r'); hold on;
plot(t,y_arx_2,'-','Color','g'); hold on;
plot(t,y_arx_3,'-.','Color','b'); hold off;
legend('data','arx where y = M*th','arx with tf','arx with sim.','location','southeast');
xlabel('time [s]'); ylabel('y [unit]')
xlim([0 t(end)]); ylim([0 max(y_true)]); grid on;
hold off;