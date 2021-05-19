function [rotationDecision,probabilityRotation] = decision_fusion(trajectory,y,EKF_orientation,K_gyro)

% get mu = [notRotating;rotating] matrices for all inputs, then fuze
[mu_xAccel, mu_yAccel] = fuzzy_accelerometer(trajectory,y(1),y(2));
mu_EKF = fuzzy_EKF(EKF_orientation);
mu_KGyro = fuzzy_KalmanGain(K_gyro);
mu_gyro = fuzzy_gyroscope(y(3));

if trajectory==3
    w_EKF = 0.4;
    w_K = 0.4;
    w_gyro = 0.2;
    mu_weighted = (mu_EKF*w_EKF + mu_KGyro*w_K + mu_gyro*w_gyro);
else
    w_xA = 0.1;
    w_yA = 0.1;
    w_EKF = 0.1;
    w_K = 0.3;
    w_gyro = 0.4
    mu_weighted = (mu_xAccel*w_xA + mu_yAccel*w_yA + mu_EKF*w_EKF + mu_KGyro*w_K + mu_gyro*w_gyro);
end


probabilityRotation = mu_weighted(2);

[maxVal,i] = max(mu_weighted);
if i==1
    %notRotating
    rotationDecision=0;
else
    %rotating
    rotationDecision=1;
end

