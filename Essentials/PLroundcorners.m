%%  [PL] = PLroundcorners(PL,indices,varargin)
%	=== INPUT PARAMETERS ===
%	PL:     Contour of PL you to search throguh
%	=== OUTPUT RESULTS ======
function [PL] = PLroundcorners(PL,corner_numbers,varargin)
radius = ones(1,size(corner_numbers,2)); if nargin>=3 && ~isempty(varargin{1}); radius=varargin{1}; end
if(size(radius,1)==1)
    radius = repmat(radius,1,size(corner_numbers,2));
end
PL_save = CPLselectinout(PL,1);
PL = CPLselectinout(PL,0);
corners = {};
for i=1:size(corner_numbers,2)
    if corner_numbers(i) == 1
        v1 = PL(corner_numbers(i)+1,:)-PL(corner_numbers(i),:);
        v1 = v1/norm(v1);
        v2 = PL(size(PL,1),:)-PL(corner_numbers(i),:);
        v2 = v2/norm(v2);
    elseif corner_numbers(i) == size(PL,1)
        v1 = PL(1,:)-PL(corner_numbers(i),:);
        v1 = v1/norm(v1);
        v2 = PL(corner_numbers(i)-1,:)-PL(corner_numbers(i),:);
        v2 = v2/norm(v2);
    else
        v1 = PL(corner_numbers(i)+1,:)-PL(corner_numbers(i),:);
        v1 = v1/norm(v1);
        v2 = PL(corner_numbers(i)-1,:)-PL(corner_numbers(i),:);
        v2 = v2/norm(v2);
    end
    following_point = PL(corner_numbers(i),:)+(v1*abs(radius(i)));
    trailing_point = PL(corner_numbers(i),:)+(v2*abs(radius(i)));
%     corners{end+1} = CPLradialEdges([trailing_point;PL(corner_numbers(i),:);following_point],radius);
    corners{end+1} = PLcircarc2([trailing_point;PL(corner_numbers(i),:);following_point]);
    if radius(i)<0
        corners{end} = PLmirror0(corners{end},'y',1);
    end
end
for i=1:size(corner_numbers,2)
    if corner_numbers(i) == 1
        PL = [corners{i};PL(corner_numbers(i)+1:end,:)];
    elseif corner_numbers(i)==size(PL,1)
        PL = [PL(1:corner_numbers(i)-1,:);corners{i}];
    else
        PL = [PL(1:corner_numbers(i)-1,:);corners{i};PL(corner_numbers(i)+1:end,:)];
    end
    corner_numbers = corner_numbers + size(corners{i},1)-1;
    
end
if ~isempty(PL_save)
    PL = [PL;NaN NaN;PL_save];
end
end