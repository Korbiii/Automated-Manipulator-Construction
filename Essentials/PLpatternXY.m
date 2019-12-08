%%   [CPL] = PLpatternXY(PL,x_num,y_num,x_dis,y_dis)
%	=== INPUT PARAMETERS ===
%	PL:     Contour of PL you want to pattern
%	x_num:	Number of copies in x-direction
%	y_num:	Number of copies in y-direction
%   x_dis:  Distance between copies in x-direction
%   y_dis:  Distance between copies in y-direction
%	=== OUTPUT RESULTS ======
%   CPL: 	CPL of copied elements
function [CPL] = PLpatternXY(PL,x_num,y_num,x_dis,y_dis)
CPL = PL;
for i=1:x_num-1
    CPL = [CPL;NaN NaN;PLtrans(PL,[x_dis*i 0])];
end
PL_2 = CPL;
for i=1:y_num-1
    CPL = [CPL;NaN NaN;PLtrans(PL_2,[0 y_dis*i])];
end
CPL = PLtrans0(CPL);
end
