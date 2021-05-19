function [mu] = fuzzy_KalmanGain(K)
low = -0.034;
high = -0.017;

if K<low
    mu_notrotating = 0;
    mu_rotating = 1;
elseif K>low && K<high
    mu_notrotating = (K-low)/(high-low);
    mu_rotating = (high-K)/(high-low);
elseif K>high
    mu_notrotating = 1;
    mu_rotating = 0;
end
    
not_rotating = mu_notrotating;
rotating = mu_rotating;
mu = [not_rotating;rotating];
end



