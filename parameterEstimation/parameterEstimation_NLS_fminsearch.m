%% info
% this is parameter estimation of a nonlinear model on a time-domain
% the method is non-linear least squares (see function)

%% clear
clc; clear; close;

%% create actual model
K = 10;
T = 5;
Gp = tf(K,[T 1]);

%% create data
[y_true, t] = step ( Gp , 4*T);
y_meas = y_true + randn(length(t),1)*sqrt(K)*0.1;

%% plot the data
plot(t,y_meas,'o','Color','k');
hold on;
h = plot(t,y_meas,'r-');
hold off;
xlabel('time [s]'); ylabel('ang. vel. [rad/s]')
xlim([0 t(end)]); ylim([0 K]); grid on;

%% solver
x0 = [1,1];
%options = optimset('TolX',0.1);
p_est = fminsearch(@(p) myfitfun(p,t,y_meas,h),x0)
%p_est = fminsearch(@(p) myfitfun(p,t,y_meas,h),x0,options)

%% functions
function err = myfitfun(p, t, y_meas, handle)
    % y(t) => f(t, th) where th = [param 1, ..., param n]
    % y_meas(ti) = f(ti, th*) = f(ti,th_now) + (df/dth1)*(th1*-th1_now) + ... 
    % res(ti) = y_meas(ti) - f(ti,th_now) = -(df/dth1)*(th1*-th1_now) + ...
    % note: (df/dthj) have to be evaluated for all parameters j at time t
    % -> find the best delta_th for multiple data points r = A*x -> x = inv(A'A)A'*r
    % -> repeat the process (because evaluatations of part. der. change)

    z = p(1)*(1-exp(-t/p(2)));
    err = norm(z - y_meas, 2);
    %set(gcf,'DoubleBuffer','on');
    set(handle,'ydata',z); drawnow; %pause(0.00)
end
