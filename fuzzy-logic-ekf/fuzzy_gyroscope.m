function [mu] = fuzzy_gyroscope(gyro)
low = 62.3;
high = 124.6;

if gyro<low
    mu_notrotating = 1;
    mu_rotating = 0;
elseif gyro>low && gyro<high
    mu_notrotating = (high-gyro)/(high-low);
    mu_rotating = (gyro-low)/(high-low);
elseif gyro>high
    mu_notrotating = 0;
    mu_rotating = 1;
end
    
not_rotating = mu_notrotating;
rotating = mu_rotating;
mu = [not_rotating;rotating];
end

