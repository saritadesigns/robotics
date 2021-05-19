function G = linearize_motion_model(mu)
    G = 25/(mu^2 + 1) - (50*mu^2)/((mu^2 + 1)^2) + 1/2;
end