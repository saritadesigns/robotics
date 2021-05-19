function mup = motion_model(mu,k)
    mup = (mu/2) + (25*mu)/(1+(mu^2)) + 8*cos(1.2*k);
end