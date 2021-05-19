function Gt = omnibot_linearize_motion_model(x3)
% This was found using syms and Jacobian
    Gt = [1 0 (- sin(x3)/40 - sin(x3 - pi/3)/30 - sin(x3 + pi/3)/60);
        0 1 (- cos(x3)/40 - cos(x3 - pi/3)/30 - cos(x3 + pi/3)/60);
        0 0 1];
end