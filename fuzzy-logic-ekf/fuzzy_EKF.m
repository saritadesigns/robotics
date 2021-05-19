function [mu] = fuzzy_EKF(orientation)
low = 0.375;
mid = 0.75;
high = 1.125;

mu_original = fuzzy_lowmid(orientation,low,mid);
mu_rotating = fuzzy_lowhigh(orientation,low,mid,high);
mu_rotated = fuzzy_midhigh(orientation,mid,high);

not_rotating = mu_original + mu_rotated;
rotating = mu_rotating;
mu = [not_rotating;rotating];
end

