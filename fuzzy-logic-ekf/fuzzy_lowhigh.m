function [mu] = fuzzy_lowhigh(val,low,mid,high)
if val < low || val > high
    mu = 0;
elseif val > low && val < mid
    mu = (val-low)/(high-mid);
elseif val > mid && val < high
    mu = (high-val)/(high-mid);
end
end

