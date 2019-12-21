%%  [PL] = PLcircarc2(PL,radius)
%	=== INPUT PARAMETERS ===

%	=== OUTPUT RESULTS ======
function [PL] = PLcircarc2(PL,radius)
v1 = PL(1,:)-PL(2,:);
v1 = v1/norm(v1);
v2 = PL(3,:)-PL(2,:);
v2 = v2/norm(v2);
phi = acos(dot(v1,v2)/(norm(v1)*norm(v2)));

PL = PLcircseg(radius,'',0,phi);


end