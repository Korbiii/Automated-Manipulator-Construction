%%  [CP] = PLcrossCPLLine2(CPL_line,CPL)
%	=== INPUT PARAMETERS ===
%	CPL_line:   CPL of line to cross CPL
%	CPL:        CPL of CPL to cross CPL_line
%	M_paras:   	3xn Vector of DoFs [direction_angle total_angle offset]
%	=== OUTPUT RESULTS ======
%	CP:         List of all crosspoints
function [CP] = PLcrossCPLLine2(CPL_line,CPL)
%%Initializing
sep = [0 find(isnan(CPL(:,1)),15)' size(CPL,1)+1];
CPLs = {};
CP = [];
%% Seperating CPL in its seperate CPLs
for i=1:size(sep,2)-1
    if size(sep,2)>2
        CPLs{end+1} = CPL(sep(i)+1:sep(i+1)-1,:);
    else
        CPLs{end+1} = CPL;
    end
end
%%Finding crosspoints
for k=1:size(CPLs,2)
    for n=1:size(CPLs{1,k},1)
        if n==size(CPLs{1,k},1) %% Wraparound for last to first point
            CP_temp = PLcrossCPLline(CPL_line,CPLs{1,k}(n,:),CPLs{1,k}(1,:));
            CP_temp_2 = PLcrossCPLline(CPL_line,CPLs{1,k}(n,:),CPLs{1,k}(1,:),true);
        else
            CP_temp = PLcrossCPLline(CPL_line,CPLs{1,k}(n,:),CPLs{1,k}(n+1,:));
            CP_temp_2 = PLcrossCPLline(CPL_line,CPLs{1,k}(n,:),CPLs{1,k}(n+1,:),true);
            
        end
               
        if isempty(CP_temp)
            if ~isempty(CP_temp_2)
                if sum(CP_temp_2 == CPL_line(1,:)) == 2 || sum(CP_temp_2 == CPL_line(2,:)) == 2
                    CP_temp_2 = [];
                end
            end
            CP_temp = CP_temp_2;
        end
        CP = [CP;CP_temp];
        
    end
end
  CP = unique(CP,'rows','stable');
end

