% Extended Kalman filter example
clc;
% clear all;
close all;

% Variables
K = 0:100;
disp(length(K));

% Should this be squared?
var_motion = 10;
var_sensor = 1;

R = var_motion;
Q = var_sensor;

% Initial State
x0 = normrnd(0,1);

% Prior
mu = 0; % mean (mu)
S = 1;% covariance (Sigma)

% Simulation Initializations
x = zeros(1,length(K));
x_ideal = zeros(1,length(K));

x(1) = x0;
x_ideal(1) = x0;

z = zeros(1,length(K));

mu_S = zeros(1,length(K));
mu_SI = zeros(1,length(K));

rmse_ekf = zeros(1,length(K));
rmse_iekf = zeros(1,length(K));

rmse_ekf(1) = 0;
rmse_iekf(1) = 0;


for k=2:length(K)
% So the first iteration (when k=1) has a value of 1
% And the last iteration (when k=100) has a value of 100

    v = normrnd(0,var_motion);
    n = normrnd(0,var_sensor);

    x_ideal(k) = motion_model(x_ideal(k-1),k-1);
    x(k) = motion_model(x(k-1),k-1) + v;
    g = x_ideal(k);
%     disp(x_ideal(k));
    
    Gt = linearize_motion_model(x_ideal(k-1));
    Ht = linearize_sensor_model(x_ideal(k));
    
    z(k) = sensor_model(x(k)) + n;
    Y=z(k);

    
    %% Extended Kalman Filter Estimation
    [mu,S] = EKF(g,Gt,Ht,S,Y,@flight_sensor_model,R,Q);
    [mu_I,S_I] = IEKF(g,Gt,S_I,Y,@flight_sensor_model,R,Q);
    
    %% Store Results
    mu_S(k) = mu;
    mu_SI(k) = mu_I;
    
    
    %% Metric (RMSE) 
    rmse_ekf(k) = sqrt(mean((mu_S(k)-x_ideal(k)).^2));
    rmse_iekf(k) = sqrt(mean((mu_SI(k)-x_ideal(k)).^2));

    %% Plot
    figure(1);clf; hold on;
    plot(K(1:k),x_ideal(1:k),'ro--')
    plot(K(1:k),mu_S(1:k),'bx--')
    
    figure(2);clf; hold on;
    plot(K(1:k),rmse_ekf(1:k),'bs--')

    figure(3);clf; hold on;
    plot(K(1:k),x_ideal(1:k),'ro--')
    plot(K(1:k),mu_S(1:k),'bx--')
    plot(K(1:k),mu_SI(1:k), 'x--', 'Color', '#208524'); %iterative EKF prediction
    
    figure(4);clf; hold on;
    plot(K(1:k),rmse_ekf(1:k),'bs--')
    plot(K(1:k),rmse_iekf(1:k), 's--', 'Color', '#208524'); %iterative EKF prediction



end