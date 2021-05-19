function mup = omnibot_motion_model(mu,v,dt)
    % mup = [x +vcos(theta)dt; y + vsin(theta)dt; theta +omega*dt]
    % note: v(1) = v_x... v(2) = v_y.... v(3) = w
    mup=[mu(1)+v(1)*dt; mu(2)+v(2)*dt; mu(3)+v(3)*dt];
    
end