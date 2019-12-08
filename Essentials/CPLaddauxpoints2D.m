%%  [CPL] = CPLaddauxpoints2D(CPL,d)
%	=== INPUT PARAMETERS ===
%	CPL:	CPL to which you want to add more points
%	d:		distance between new points
%	=== OUTPUT RESULTS ======
%	CPL:    CPL with added points
function [CPL] = CPLaddauxpoints2D(CPL,d)
[GPL,k] =GPLauxgridpointsCPS(CPL,d);
positions = find(k==1);
for i=1:size(positions,1)
    CPL = [CPL;NaN NaN;GPL(positions(i),:)];
end
end