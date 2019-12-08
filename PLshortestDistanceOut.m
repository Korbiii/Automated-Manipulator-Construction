%%   [e_dir] = PLshortestDistanceOut(CPL,point)
%	=== INPUT PARAMETERS ===
%	CPL:	Contour in which point sits
%	point:	Point from which to find shortest distance to CPL
%	=== OUTPUT RESULTS ======
%   e_dir:  Direction from point
function [e_dir] = PLshortestDistanceOut(CPL,point)
line = [point;point+500];
distance = 500;
closest_point = [];
deg = 1;
for i=1:deg:360
    cp = PLcrossCPLLine2(line,CPL);
    new_dis = pdist2(cp,point);
    if new_dis < distance
        closest_point = cp;
        distance = new_dis;
    end
    line = PLtransC(line,point,rad2deg(deg));
end
e_dir = [point;closest_point];
end