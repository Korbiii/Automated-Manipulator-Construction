%%   [SG] = SGofCPLy(CPL,x)
%	=== INPUT PARAMETERS ===
%   CPL:    CPL for Extrusion of CPL
%   y:      Distance of extrusion in y-direction
%	=== OUTPUT RESULTS ======
function [SG] = SGofCPLy(CPL,y)
SG = SGofCPLz(CPL,y);
SG = SGtransR(SG,rot(pi/2,0,0));
end

