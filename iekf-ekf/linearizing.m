% linearizing g(x_t-1, u_t)
syms x;
k=1;

% motion model
g_func = (1/2)*x + (25*x)/(1+(x^2)) + 8*cos(1.2*k);
G = jacobian(g_func, x);
disp(G);
% Motion Model linearized (G)
% 25/(x^2 + 1) - (50*x^2)/(x^2 + 1)^2 + 1/2


% measurement model
h_func = (x^2)/20;
H = jacobian(h_func, x);
disp(H);
% Motion Model linearized (H)
% x/10