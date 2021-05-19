%% Linearize Motion Model
dt = 0.1; %timestep
Tf = 15; %15 seconds
T = 0:dt:Tf; 
r = 0.4; %radius
w = 5; %angular velocity

syms x1 x2 x3

    v_x = r*w*cos(x3);
    v_y = r*w*sin(x3);

    X1 = x1 + v_x*dt; 
    X2 = x2 + v_y*dt;
    X3 = x3 + w*dt;
    
    g = [X1;X2];
    Gt=jacobian(g,[x1;x2]);
%     disp(Gt);

    %% Linearize Measurement Model 
    syms x1 x2 x3 x4 x5 x6
    %Poly fit
%     Y1 = 4.1136*x1^2 -4.7153*x1 + 9.9209;
%     Y2 = 2.3153*x2^2 -3.0747*x2 + 9.6963;

    %Inverse fit
    Y1 = (8.3741*x1+0.2395)/(x1+0.0123); %x-accelerometer
    Y2 = (8.3558*x2+1.3344)/(x2+0.1294); %y-accelerometer
    Y3 = 0.6127*x3^2 - 21.343*x3 + 189.15; %john's z-gyroscope
    %Power fit
%     Y1 = 8.2486*x1^-0.0619;
%     Y2 = 8.4711*x2^-0.0472;
    
    h = [Y1;Y2;Y3];                
    Ht=jacobian(h,[x1,x2,x3,x4,x5,x6]);
    disp(Ht);
    
    %% Linearize Motion Model: Line
%     syms x y dx dy dt
%     X1 = x +dx*dt;
%     X2 = y +dy*dt;
%     X3 = dx;
%     X4 = dy;
%     
%     g = [X1;X2;X3;X4];
%     Gt = jacobian(g,[x;y;dx;dy]);
%     disp(Gt);
%     
    
    