%%  [dist] = distLinePoint((PL_line,points)
%	=== INPUT PARAMETERS ===
%	PL_line:    PL of straight line
%	points:     nx2 points you want the distance to the line
%	=== OUTPUT RESULTS ======
%	dist:       shortest distance from point to line
function [dist] = distLinePoint(PL_line,points)
x1 =  PL_line(1,1);
x2 =  PL_line(2,1);
y1 =  PL_line(1,2);
y2 = PL_line(2,2);
x0 = points(:,1);
y0 = points(:,2);

nom = (y2-y1)*x0-(x2-x1)*y0+x2*y1-y2*x1;
denom = (y2-y1)^2+(x2-x1)^2;

dist = abs(nom)/sqrt(denom);


end