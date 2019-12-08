%%  [SG] = SGofCPLx(CPL,x)
%	=== INPUT PARAMETERS ===
%   CPL:    CPL for Extrusion of CPL
%   x:      Distance of extrusion in x-direction
%	=== OUTPUT RESULTS ======
function [SG] = SGofCPLx(CPL,x)
SG = SGofCPLz(CPL,x);
SG = SGtransR(SG,rot(0,pi/2,0));
end

