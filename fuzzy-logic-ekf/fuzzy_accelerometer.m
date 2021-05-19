function [Xmu,Ymu] = fuzzy_accelerometer(trajectory,xAccel,yAccel)

Xlow = 11;
Xmid = 14;
Xhigh = 17;

Ylow = 8.8;
Ymid = 9.3;
Yhigh = 9.8;

if trajectory==1    
    %mu_original
    Xmu_original = fuzzy_lowmid(xAccel,Xlow,Xmid);
    Ymu_original = fuzzy_midhigh(yAccel,Ymid,Yhigh);
    
    %mu_rotating
    Xmu_rotating = fuzzy_lowhigh(xAccel,Xlow,Xmid,Xhigh);
    Ymu_rotating = fuzzy_lowhigh(yAccel,Ylow,Ymid,Yhigh);
        
    %mu_rotated
    Xmu_rotated = fuzzy_midhigh(xAccel,Xmid,Xhigh);
    Ymu_rotated = fuzzy_lowmid(yAccel,Ylow,Ymid);

elseif trajectory==2
    %mu_original
    Xmu_original = fuzzy_midhigh(xAccel,Xmid,Xhigh);
    Ymu_original = fuzzy_lowmid(yAccel,Ylow,Ymid);
    
    %mu_rotating
    Xmu_rotating = fuzzy_lowhigh(xAccel,Xlow,Xmid,Xhigh);
    Ymu_rotating = fuzzy_lowhigh(yAccel,Ylow,Ymid,Yhigh);
        
    %mu_rotated
    Xmu_rotated = fuzzy_lowmid(xAccel,Xlow,Xmid);
    Ymu_rotated = fuzzy_midhigh(yAccel,Ymid,Yhigh);
    
else
    %mu_original
    Xmu_original = 0;
    Ymu_original = 0;
    
    %mu_rotating
    Xmu_rotating = 0;
    Ymu_rotating = 0;
        
    %mu_rotated
    Xmu_rotated = 0;
    Ymu_rotated = 0;
end

Xnot_rotating = Xmu_original + Xmu_rotated;
Xrotating = Xmu_rotating;

Ynot_rotating = Ymu_original + Ymu_rotated;
Yrotating = Ymu_rotating;

%mu: [notRotating;rotating]
Xmu = [Xnot_rotating;Xrotating];
Ymu = [Ynot_rotating;Yrotating];

end

