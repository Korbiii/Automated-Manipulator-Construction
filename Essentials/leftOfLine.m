%%  [index] = leftOfLine(PL_line,points)
%	=== INPUT PARAMETERS ===
%	PL_line:    PL of straight line
%	points:     Point List
%	=== OUTPUT RESULTS ======
%	index:      -1 right of, 0 on 1 left
function [index] = leftOfLine(PL_line,points)

d = (points(:,1)-PL_line(1,1))*(PL_line(2,2)-PL_line(1,2))-(points(:,2)-PL_line(1,2))*(PL_line(2,1)-PL_line(1,1));
d= d./abs(d);

end