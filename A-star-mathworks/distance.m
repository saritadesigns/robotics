function dist = distance(x1,y1,x2,y2)
%This function calculates the distance between any two cartesian 
%coordinates.

dx = abs(x1-x2);
dy = abs(y1-y2);

% Euclidean distance (Pythagoras) 
% dist=sqrt((dx)^2 + (dy)^2);

% Manhattan distance
% dist=dx + dy;

% Chebyshev distance
% dist=max(dx,dy);

% Heuristic
% dist = min(dx, dy) * sqrt(2) + abs(dx - dy);

% %Dijkstra
dist=0;
