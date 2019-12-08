%%  [SG] = SGsplitandarrange(SG)
%	=== INPUT PARAMETERS ===
%	SG:    	SG out of multiple parts 
%	=== OUTPUT RESULTS ======
%	SG:         Newly arranged SG
function [SG] = SGsplitandarrange(SG)
SGs = SGanalyzeGroupParts(SG);
SG = SGarrangeSG(SGs);
end