 function [mu] = fuzzy_lowmid(val,low,mid)
if val < low
    mu = 1;
elseif val > low && val < mid
    mu = (mid-val)/(mid-low);
elseif val > mid
    mu = 0;
end
end

