%% info
% method: Autoregressive Moving Average with Extra Input (ARMAX)
% adv. of ARMAX vs ARX is that the disturbance model can be modelled as well
% https://nl.mathworks.com/help/ident/ref/armax.html
% https://www.youtube.com/watch?v=JRWjO2oiy3Y
% A(z) = 1 + a_1*z^(-1) + ... + a_na*z^(-na)        -> eq(1)
% B(z) = 0 + b_1*z^(-nk) + ... + b_nb*z^(-nb-nk+1)  -> eq(2) (nk = input lag)
% C(z) = 1 + c_1*z^(-1) + ... + c_nc*z^(-nc)        -> eq(3)
% theta = [a_1 ... a_na b_1 ... b_nb c_1 ... c_nc]'
% y(k) = 1/A(z) * ( B(z)*u(k) + C(z)*e(k))          -> eq(4)
% rewrite eq(4) as: A*y(k) = B(z)*u(k)) + C(z)*e(k) -> eq(5)
% now insert eq(1) and eq(2) in eq(4)
% y(k) = -a_1*y(k-1) - ... - a_na*y(k-na) + b_1*u(k) + b(2)*u(k-1) + ... + b_nb*u^(k-nb+1) + c_1*e(k-1) + ... + c_nc*e(k-nc)
% the next step would be to make an explicit expression, such that e terms are not in the step predictions
