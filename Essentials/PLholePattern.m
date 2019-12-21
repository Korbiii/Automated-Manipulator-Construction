%%   [CPL] = screw_holes(radius,number,hole_radius)
%	=== INPUT PARAMETERS ===
%	radius:         radius of circular hole pattern
%	number:         number of holes
%   hole_radius:    radius of holes
%	=== OUTPUT RESULTS ======
%   CPL:            CPL of hole pattern
%
%   Same as CPLcopyradial
%
function [CPL] = PLholePattern(radius,number,hole_radius)
PL_hole = PLtrans(PLcircle(hole_radius),[0 radius]);
CPL = PL_hole;
for i=0:number
    CPL = CPLbool('+',CPL,PLtransC(PL_hole,[0 0],((2*pi)/number)*i));
end
end