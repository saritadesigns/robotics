function v = find_velocities(u,x3)
r=0.25; %25cm radius of wheels
L=0.3; %30cm distance to wheel
v_x = (r*2/3) * (-u(1)*cos(x3) + u(2)*cos(pi/3-x3) + u(3)*cos(pi/3+x3));
v_y = (r*2/3) * (u(1)*sin(x3) + u(2)*sin(pi/3-x3) - u(3)*sin(pi/3+x3));
w = r/(3*L) * (u(1)+u(2)+u(3));
v = [v_x; v_y; w];
end

