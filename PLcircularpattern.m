%%  [CPL] = PLcircularpattern(PL,radius,angle,num)
%	=== INPUT PARAMETERS ===
%	PL:     Contour of PL you want to pattern
%   radius: Radius of pattern
%   angle:  angle between elements
%   num:    number of elements
%	=== OUTPUT RESULTS ======
%   CPL: 	CPL of copied elements
function [CPL] = PLcircularpattern(PL,radius,angle,num)
PL = PLtrans(PL,[radius 0]);
CPL = PL;
for i=2:num
    CPL = [CPL;NaN NaN;PLtransC(PL,[0 0],angle*(i-1))];
end
end
