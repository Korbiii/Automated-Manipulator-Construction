%%  [dist] = distPointLine(PL_line,point)
%	=== INPUT PARAMETERS ===
%	PL_line:    PL of straight line
%	point:      2x1 point you want the distance to the line
%	=== OUTPUT RESULTS ======
%	dist:       shortest distance from point to line
function [dist] = distPointLine(PL_line,point)

v1 = [PL_line(1,:) 0]';
v2 = [PL_line(2,:) 0]';
pt = [point 0]';
a = v1 - v2;
b = pt - v2;
dist = norm(cross(a,b)) / norm(a);

end