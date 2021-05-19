function [mu] = fuzzy_midhigh(val,mid,high)
if val < mid
    mu = 0;
elseif val > mid && val < high
    mu = (val-mid)/(high-mid);
elseif val > high
    mu = 1;
end
end

