%%  [indices] = PLfindcorners(PL)
%	=== INPUT PARAMETERS ===
%	PL:     Contour of PL you to search throguh
%	=== OUTPUT RESULTS ======
%
function [indices] = PLfindcorners(PL)
indices = [];
for i=1:size(PL,1)
    if i == size(PL,1)
        v1 = PL(1,:)-PL(i,:);
    else        
        v1 = PL(i+1,:)-PL(i,:);
    end
    if i == 1
        v2 = PL(size(PL,1),:)-PL(i,:);
    else        
        v2 = PL(i-1,:)-PL(i,:);
    end    
    phi = acosd(dot(v1,v2)/(norm(v1)*norm(v2)));
    if phi ~= 180
        indices = [indices i];
    end
end
end
