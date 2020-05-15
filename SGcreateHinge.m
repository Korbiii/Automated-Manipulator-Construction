%%  [SG] = SGcreateHinge(CPL,SG_hinge,hinge_dir)
%	=== INPUT PARAMETERS ===
%	CPL:        CPL of element you want to create a hinge for
%	SG_hinge:   SG of your hinge
%	hinge_dir:  Direction of hinge
%   hinge_opti: 0 no opti; 1 opti in pos -1 in negative
%	=== OUTPUT RESULTS ======
%	SG:         SG of connector element
%   offset:     Calculated offset in positive y direction
function [SG,offset] = SGcreateHinge(CPL,SG_hinge,hinge_dir,hinge_opti,hinge_width,min_len,h_height)
%% Check if same hinge was already created once

global hinges;
already_existing = 0;
for i=1:size(hinges,2)
    if isequalwithequalnans(hinges{i}{3},CPL) && isequal(hinges{i}{4},[hinge_dir,hinge_opti,hinge_width,min_len])
        SG =  hinges{i}{1};
        offset = hinges{i}{2};
        already_existing = 1;
        break;
    end
end

if ~already_existing
    
    %% Initializing and generating general values
    offset = 0;
    max_dim = max(sizeVL(CPL))+1;
    middle_axis = PLtransR([-max_dim 0;max_dim 0],rot(deg2rad(hinge_dir)));
    e_dir = [sind(hinge_dir) -cosd(hinge_dir)];
    PL_offsetline = middle_axis;
    middle_axis = PLtransR(middle_axis,rot(pi/2));
    pos_plane = [flip(middle_axis);PLtrans(middle_axis,e_dir*rot(-pi/2)*max_dim*2)]; % Plane for finding points in positive area
    hinge_width = hinge_width+(2*h_height);
%     hinge_width = hinge_width+1;
    
    %% Calculating best offset
    if abs(hinge_opti) == 1
        PL_offsetline = PLtrans(PL_offsetline,-e_dir*max_dim*hinge_opti);
        size_h = 0; res = 0.3; offset = max_dim;
        while size_h < min_len && offset > 0
            size_h = 0;
            PL_offsetline = PLtrans(PL_offsetline,e_dir*res*hinge_opti);
            offset = offset-res;
            c_p = PLcrossCPLLine3(PL_offsetline,CPL);
            if ~isempty(c_p)
                c_p = sortrows(c_p);
                ex_1 = c_p(1,:);
                ex_2 = c_p(end,:);
                ex_1_n = floor(distPointLine(middle_axis,ex_1)/res)-1;
                ex_2_n = floor(distPointLine(middle_axis,ex_2)/res)-1;
                PL_ex_1 = [ex_1;PLtrans(ex_1,e_dir*rot(pi/2)*hinge_width)];
                PL_ex_2 = [ex_2;PLtrans(ex_2,-e_dir*rot(pi/2)*hinge_width)];
                counting = 0;
                for k=0:ex_1_n
                    inside_ex_1 = inside2C(CPL,PL_ex_1);
                    if (inside_ex_1(1)+inside_ex_1(3)) == size(PL_ex_1,1) && inside_ex_1(4) == 0  && counting
                        size_h = size_h + res;
                    elseif (inside_ex_1(1)+inside_ex_1(3)) == size(PL_ex_1,1) && inside_ex_1(4) == 0  && ~counting
                        counting = 1;
                    else
                        counting = 0;
                    end
                    PL_ex_1 = PLtrans(PL_ex_1,e_dir*res);
                end
                counting = 0;
                for k=0:ex_2_n
                    inside_ex_2 = inside2C(CPL,PL_ex_2);
                    if (inside_ex_2(1)+inside_ex_2(3)) == size(PL_ex_2,1) && inside_ex_2(4) == 0  && counting
                        size_h = size_h + res;
                    elseif (inside_ex_2(1)+inside_ex_2(3)) == size(PL_ex_2,1) && inside_ex_2(4) == 0  && ~counting
                        counting = 1;
                    else
                        counting = 0;
                    end
                    PL_ex_2 = PLtrans(PL_ex_2,-e_dir*res);
                end
            end
        end
        if offset>0
            PL_offsetline_mid_hinge = PLtrans(PL_offsetline,e_dir*rot(pi/2)*((hinge_width/2)));
            offset = (offset-(hinge_width/2));
            SG_hinge = SGtrans(SG_hinge,[-e_dir*offset*hinge_opti 0]);
            leftOf = leftOfLine(PL_offsetline_mid_hinge,[0 0]);
            if leftOf
                offset = -offset;
            end
        else
            error("Not enough space for hinge");
        end
    end
    SG_hinge = SGtrans(SG_hinge,[e_dir*rot(-pi/2)*max_dim 0]);
    VL_hinge = SG_hinge.VL;
    
    SG =[];
    proj_points = {};
    %%  Generating all cross points between CPL and hingepoint projections
    
    unique_points = unique(VL_hinge(:,1:2),'rows');
    unique_cp = {};
    for o=1:size(unique_points,1)
        PL_cross_line = [unique_points(o,1:2);unique_points(o,1:2)+(e_dir*rot(pi/2)*max_dim*2)];
        c_p =  PLcrossCPLLine3(PL_cross_line,CPL);
        c_p(:,3) = pdist2(c_p,unique_points(o,1:2));
        c_p = sortrows(c_p,3);
        c_p = c_p(:,1:2);
        unique_cp{end+1} = c_p;
    end
    for o=1:size(VL_hinge,1)
        [~,index] = ismember(VL_hinge(o,1:2),unique_points,'rows');
        proj_points{end+1} = unique_cp{index};
    end
    
    %% Getting number of hinge elements below and above axis through [0 0]
    [proj_points_size, ~] = max(cellfun('size', proj_points, 1));
    proj_points_max_values = cellfun('size', proj_points, 1) == proj_points_size;
    
    max_axis = proj_points(proj_points_max_values);
    origin_axis = PLtrans(middle_axis,e_dir(1,:)*rot(pi/2)*max_dim);
    
    plains = {[e_dir;-e_dir]};
    plain_offsets = [distLinePoint(origin_axis,[0 0]) distLinePoint(origin_axis,[0 0])];    
    for j=1:size(max_axis,2)
        for k=1:2:proj_points_size-2
            axis_offset = [mean(max_axis{j}(k+1:k+2,1)),mean(max_axis{j}(k+1:k+2,2))];
            plain_temp = [axis_offset;axis_offset+(e_dir)];
            dis_temp =  distPointLine(origin_axis,plain_temp(1,:));
            if min(abs(plain_offsets-dis_temp))>0.1
                plains{end+1} = plain_temp;
                plain_offsets = [plain_offsets dis_temp dis_temp];
            end
        end
    end
    if size(plains,2) >1
        plains = sortrows([cell2mat(plains') plain_offsets'],3,'descend');
        plains = mat2cell(plains(:,1:2),repmat(2,1,size(plain_offsets,2)/2));
    end
    positive_gl = 1;
    negative_gl = 1;
    len = pdist2(PL_offsetline(1,:),PL_offsetline(2,:))/2;
    for o=1:size(plains,1)
        dist = distLinePoint(plains{o},PL_offsetline(2,:));
        if round(dist,1) < round(len,1)
            positive_gl = positive_gl+1;
        elseif round(dist,1) > round(len,1)
            negative_gl = negative_gl+1;
        end
    end
    positive_gl = positive_gl*2;
    negative_gl = negative_gl*2;
    proj_points_size = (size(plains,1)+1)*2;
    %% Filling up points list in a way that it can be iterated through easily afterwards to generate hinge elements
    for k=1:size(proj_points,2)
        if size(proj_points{k},1) < proj_points_size
            if size(proj_points{k},1) == 2
                proj_points{k}=[proj_points{k};repmat(proj_points{k}(end-1:end,:),(proj_points_size-size(proj_points{k},1))/2,1)];
            else
                num_pos_neg = proj_points{k};
                in_out = leftOfLine(middle_axis,num_pos_neg);
                positive = length(find(in_out>0));
                negative = length(find(in_out<0));
                positive_v = proj_points{k}(1:positive,:);
                negative_v = proj_points{k}(positive+1:end,:);
                if mod(positive,2) ~= 0
                    negative_temp = negative_v(1,:);
                    positive_temp = positive_v(end,:);
                    positive_v = [positive_v;negative_temp];
                    negative_v = [positive_temp;negative_v];
                    negative = negative+1;
                    positive = positive+1;
                end
                positive_v = [positive_v;repmat(positive_v(end-1:end,:),floor((positive_gl-positive)/2),1)];
                negative_v = [negative_v;repmat(negative_v(end-1:end,:),floor((negative_gl-negative)/2),1)];
                proj_points{k} = [positive_v;negative_v];
            end
        end
    end
    %% Replacing values to minimize overlapping triangles
    for u=1:size(proj_points,2)
        for j=1:size(plains,1)
            or_point = proj_points{u}(j*2-1,:);
            curr_point = proj_points{u}(j*2,:);
            distance_point_plain = distLinePoint(plains{j},or_point);
            if pdist2(or_point,curr_point)>distance_point_plain
                new_point = or_point+distance_point_plain*e_dir*rot(pi/2);
                proj_points{u}(j*2,:) = new_point;       
            end
            or_point_2 = proj_points{u}(j*2+2,:);
            curr_point_2 = proj_points{u}(j*2+1,:);
            distance_point_plain_2 = distPointLine(plains{j},or_point_2);
            if pdist2(or_point_2,curr_point_2)>distance_point_plain_2
                new_point = or_point+distance_point_plain*e_dir*rot(pi/2);
                proj_points{u}(j*2+1,:) = new_point;
            end
        end
    end
    %% Replacing values in hinge VLs with projected values
    for m=1:2:proj_points_size
        SG_hinge_new = SG_hinge;
        for n=1:size(SG_hinge.VL,1)/2
            SG_hinge_new.VL(n,1:2) = proj_points{n}(m+1,1:2);
        end
        for n=size(SG_hinge.VL,1)/2+1:size(SG_hinge.VL,1)
            SG_hinge_new.VL(n,1:2) = proj_points{n}(m,1:2);
        end
        SG = SGcat(SG,SG_hinge_new);
    end
    hinges{end+1} = {SG;offset;CPL;[hinge_dir,hinge_opti,hinge_width-1,min_len]};
    
end
end