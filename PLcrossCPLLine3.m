%%  [CP] = PLcrossCPLLine2(CPL_line,CPL)
%	=== INPUT PARAMETERS ===
%	CPL_line:   CPL of line to cross CPL
%	CPL:        CPL of CPL to cross CPL_line
%	M_paras:   	3xn Vector of DoFs [direction_angle total_angle offset]
%	=== OUTPUT RESULTS ======
%	CP:         List of all crosspoints
function [CP] = PLcrossCPLLine3(CPL_line,CPL)
pgon = polyshape(CPL(:,1),CPL(:,2));
lineseg = [CPL_line(1,1),CPL_line(1,2);CPL_line(2,1),CPL_line(2,2)];
[CP,~] = intersect(pgon,lineseg);
ind = ~isnan(CP(:,1));
CP = CP(ind,:);
end

