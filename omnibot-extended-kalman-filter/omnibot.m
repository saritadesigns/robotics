% Extended Kalman filter example
clc;
% clear all;
close all;

% Variables
dt = 0.1; %Discrete timestep (10Hz)
r=0.25; %25cm radius of wheels
L=0.3; %30cm distance to wheel
u = [-1.5;2;1]; %input wheel rotations
decl = [0; 0; -9.7*pi/180];

%% Linearizing g(x_t-1, u_t)
syms x1 x2 x3
x_arr = [x1;x2;x3];
v_x = (r*2/3) * (-u(1)*cos(x3) + u(2)*cos(pi/3-x3) + u(3)*cos(pi/3+x3));
v_y = (r*2/3) * (u(1)*sin(x3) + u(2)*sin(pi/3-x3) - u(3)*sin(pi/3+x3));
omega = r/(3*L) * (u(1)+u(2)+u(3));
g_func = x_arr + [ v_x * dt; v_y * dt; omega * dt];
G = jacobian(g_func, [x1; x2; x3]);
disp(G);

%% Init
% Initial State
x0 = [0 0 0]'; % 3x1

% Prior
mu = [0 0 0]'; % mean (mu)
S = 1*eye(3);% covariance (Sigma)

% Errors 
% R=diag([0.01^2 0.01^2 0.1*(pi/180)^2]);
% [RE, Re] = eig (R);
% Q=diag([0.5^2 0.5^2 10*(pi/180)^2]);

omega_std = 0.1 * pi / 180;
R = [0.01 0 0; 0 0.01 0; 0 0 (omega_std)].^2;
Q = [0.5 0 0; 0 0.5 0; 0 0 (10*pi/180)].^2;
[RE, Re] = eig (R);

% Simulation Initializations
Tf = 15; %15 seconds
T = 0:dt:Tf;
n = length(mu); %returns largest dimension (in this case: 3)
x = zeros(n,length(T));
x_ideal = zeros(n,length(T));
x(:,1) = x0;
x_ideal(:,1) = x0;
m = length(Q(:,1));
y = zeros(m,length(T));
mup_S = zeros(n,length(T));
mu_S = zeros(n,length(T));
v = zeros(n,length(T)); % [v_x; v_y; w]
v(:,1) = find_velocities(u,x(3,1));

%% Main loop
for t=2:length(T)
    %% Simulation
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    
    % Update state
    x_ideal(:,t) = omnibot_motion_model(x_ideal(:,t-1),v(:,t-1),dt);
    x(:,t) = omnibot_motion_model(x(:,t-1),v(:,t-1),dt) + e;
    g = x_ideal(:,t);
    Gt = omnibot_linearize_motion_model(x_ideal(3,t));
    Ht = omnibot_linearize_sensor_model(x_ideal(:,t));
    
    %Calculate (linear and angular) velocity for next time step
    v(:,t) = find_velocities(u,x(3,t));
    
    % Determine measurement; with sensor noise
    d = sqrt(Q)*randn(m,1);
    y(:,t) = omnibot_sensor_model(x_ideal(:,t)) + decl + d;
    Y=y(:,t);
    
    %% Extended Kalman Filter Estimation
    [mu,S,K,mup] = EKF(g,Gt,Ht,S,Y,@omnibot_sensor_model,R,Q);
    
    % Store results
    mup_S(:,t) = mup;
    mu_S(:,t) = mu;
    
    %% Plot results
    figure(1);clf; hold on;
    plot(x(1,2:t),x(2,2:t), 'ro--') %state x and y (directions) for timesteps
    plot(y(1,2:t), y(2,2:t), 'x--', 'Color', '#329E2B') %measurement
    plot(mu_S(1,2:t),mu_S(2,2:t), 'bx--') %BEST PREDICTION OF state x and y (directions) for timesteps
    mu_pos = [mu(1) mu(2)];
    S_pos = [S(1,1) S(1,2); S(2,1) S(2,2)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);

    %for printing title and legend in video
    text(-2,1.6,'True state, measurement and EKF Estimate');
    text(-2,1.4,'True Path','Color','red');  %printing text is faster than printing legend
    text(-2,1.2,'Measurement','Color','#329E2B');  %printing text is faster than printing legend
    text(-2,1,'EKF path','Color','blue');
    axis([-3 3 -3 2])


% % % X-Subplot
    plot(T(2:t),x(1,2:t),'ro--')
    plot(T(2:t),mu_S(1,2:t),'bx--')
    axis([0 Tf 0 10])
    text(7,5,'X-position True state and belief');
    text(7,4.5,'True Path (x)','Color','red');  %printing text is faster than printing legend
    text(7,4,'EKF path (x)','Color','blue');
    
% % % Y-Subplot
    plot(T(2:t),x(2,2:t), 'ro--')
    plot(T(2:t),mu_S(2,2:t), 'bx--')
    axis([0 Tf 0 10])
    text(7,3,'Y-position True state and belief');
    text(7,2.5,'True Path (y)','Color','red');  %printing text is faster than printing legend
    text(7,2,'EKF path (y)','Color','blue');

% % % Theta-Subplot
    plot(T(2:t),x(3,2:t), 'ro--')
    plot(T(2:t),mu_S(3,2:t), 'bx--')
    axis([0 Tf 0 10])
    text(2,7,'Theta-position True state and belief');
    text(2,6.5,'True Path (theta)','Color','red');  %printing text is faster than printing legend
    text(2,6,'EKF path (theta)','Color','blue');
    
    axis equal
    pause(0.0001);
        
end