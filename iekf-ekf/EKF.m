function [mu,S] = EKF(g,Gt,Ht,S,z,sensor_model,R,Q)
%% Extended Kalman Filter Estimation
    % Prediction
    mup = g;
    Sp = Gt*S*Gt + R;
    
    % update
    K = Sp*Ht'*inv(Ht*Sp*Ht'+Q);
    mu = mup + K*(z-sensor_model(mup));
    S = (eye(1)-K*Ht)*Sp;
end 