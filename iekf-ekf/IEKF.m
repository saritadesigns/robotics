function [mu,S] = IEKF(g,Gt,S,Y,sensor_model,R,Q)
%% Extended Kalman Filter Estimation 
    % Prediction
    mup = g;
    Sp = Gt*S*Gt' + R;
    
    
    %IEKF: iterate Ht, K, mu
    m = 5; %tested as optimal m-bar
    mu_iter = zeros(1,m);
    K_iter = zeros(1,m);
    S_iter = zeros(1,m);

    mu_iter(1) = mup;
    S_iter(1) = Sp;
    
    for n=2:m
        Ht = mu_iter(n-1)/10;

        K_iter(n-1) = Sp*Ht'*inv(Ht*Sp*Ht'+Q);
        
        v1 = (Y-sensor_model(mu_iter(n-1)));
        v2 = Ht*(mup-mu_iter(n-1));
        mu_iter(n) = mup + K_iter(n-1)*(v1-v2);
        
        S_iter(n) = (eye(1)-K_iter(n-1)*Ht)*Sp;
    end 
    
    mu = mu_iter(m);
    S = S_iter(m);
    K = K_iter(m);
    
    
end