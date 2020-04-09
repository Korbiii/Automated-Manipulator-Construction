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
%% Initializing and generating general values
offset = 0;
max_dim = max(sizeVL(CPL))+1;
middle_axis = PLtransR([-max_dim 0;max_dim 0],rot(deg2rad(hinge_dir)));
e_dir = middle_axis/norm(middle_axis);
e_dir_p = e_dir(1,:)/norm(e_dir(1,:));
e_dir_n = e_dir(2,:)/norm(e_dir(2,:));
PL_offsetline = middle_axis;
middle_axis = PLtransR(middle_axis,rot(pi/2));
pos_plane = [flip(middle_axis);PLtrans(middle_axis,e_dir_n*50)]; % Plane for finding points in positive area
% hinge_width = hinge_width+(2*h_height);
hinge_width = hinge_width+1;

%% Calculating best offset
if abs(hinge_opti) == 1
    e_dir_ = [e_dir_p;e_dir_n];
    if hinge_opti <= 0; e_dir_ =  flip(e_dir_); end
    PL_offsetline = PLtrans(PL_offsetline,e_dir_(1,:)*rot(pi/2)*max_dim);
    size_h = 0; res = 0.3; offset = max_dim;
    while size_h < min_len && offset > 0
        size_h = 0;
        PL_offsetline = PLtrans(PL_offsetline,e_dir_(2,:)*rot(pi/2)*res);
        offset = offset-res;
        c_p = PLcrossCPLLine3(PL_offsetline,CPL);
        if ~isempty(c_p)
            c_p = sortrows(c_p);
            ex_1 = c_p(1,:);
            ex_2 = c_p(end,:);
            ex_1_n = floor(distPointLine(middle_axis,ex_1)/res)-1;
            ex_2_n = floor(distPointLine(middle_axis,ex_2)/res)-1;
            PL_ex_1 = [ex_1;PLtrans(ex_1,e_dir_(2,:)*rot(pi/2)*hinge_width)];
            PL_ex_2 = [ex_2;PLtrans(ex_2,e_dir_(2,:)*rot(pi/2)*hinge_width)];
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
                PL_ex_1 = PLtrans(PL_ex_1,e_dir_(2,:)*res);
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
                PL_ex_2 = PLtrans(PL_ex_2,e_dir_(1,:)*res);
            end
        end
    end
    if offset>0
        offset = (offset-(hinge_width/2));
        SG_hinge = SGtrans(SG_hinge,[e_dir_(1,:)*rot(pi/2)*offset 0]);
        if hinge_opti > 0; offset = -offset; end
    else
        error("Not enough space for hinge");
    end
end
SG_hinge_zero = SG_hinge;
SG_hinge = SGtrans(SG_hinge,[e_dir(2,:)*15 0]);
VL_hinge = SG_hinge.VL;

SG =[];
proj_points = {};
to_side = 1;
if to_side
%%  Generating all cross points between CPL and hingepoint projections
for left=1:size(VL_hinge,1)
    PL_cross_line = [VL_hinge(left,1:2);VL_hinge(left,1:2)+(e_dir*50)];
    c_p =  PLcrossCPLLine3(PL_cross_line,CPL);
    c_p(:,3) = pdist2(c_p,VL_hinge(left,1:2));
    c_p = sortrows(c_p,3);
    c_p = c_p(:,1:2);
    proj_points{end+1} = c_p;
end
%% Getting number of hinge elements below and above axis through [0 0]
[proj_points_size, max_index] = max(cellfun('size', proj_points, 1));
proj_points_max_values = cellfun('size', proj_points, 1) == proj_points_size;

max_axis = proj_points(proj_points_max_values);
plains = {};
plain_offsets = [];

if offset == 0
    pos_plane_2 = PLtrans(pos_plane,TofR(rot(pi/2)));
else
    pos_plane_2 = PLtrans(PLtrans(pos_plane,(e_dir(1,:)/norm(e_dir(1,:)))*(offset-(hinge_width/2))),TofR(rot(pi/2)));
end
origin_axis = PLtrans(middle_axis,e_dir(1,:)*100*max_dim);
for j=1:size(max_axis,2)
    offset_distance = distPointLine(PL_offsetline,max_axis{j}(1,:));
    for left=1:2:proj_points_size-2
        axis_offset = [mean(max_axis{j}(left+1:left+2,1)),mean(max_axis{j}(left+1:left+2,2))];
        plain_temp = [axis_offset;axis_offset+(e_dir*rot(pi/2))];
        inside = insideCPS(pos_plane_2,plain_temp(1,:));
        if inside == 1
            plain_temp = PLtrans(plain_temp,(e_dir(1,:)/norm(e_dir(1,:)))*rot(-pi/2)*offset_distance);
        elseif inside == -1
            plain_temp = PLtrans(plain_temp,(e_dir(1,:)/norm(e_dir(1,:)))*rot(pi/2)*offset_distance);
        end
        if isempty(plain_offsets)
            plains{end+1} = plain_temp;
            plain_offsets = distPointLine(origin_axis,plain_temp(1,:));
        else
            dis_temp =  distPointLine(origin_axis,plain_temp(1,:));
            if min(abs(plain_offsets-dis_temp))>0.1
                 plains{end+1} = plain_temp;
                 plain_offsets = [plain_offsets dis_temp];
            end
        end
    end
end

positive_gl = 1;
negative_gl = 1;
len = pdist2(PL_offsetline(1,:),PL_offsetline(2,:))/2;
for o=1:size(plains,2)
    dist = pdist2(PL_offsetline(2,:),plains{o}(2,:));
    if round(dist,1) < round(len,1)
        positive_gl = positive_gl+1;
    elseif round(dist,1) > round(len,1)
        negative_gl = negative_gl+1;
    end
end
positive_gl = positive_gl*2;
negative_gl = negative_gl*2;
proj_points_size = (size(plains,2)+1)*2;
%% Filling up points list in a way that it can be iterated through easily afterwards to generate hinge elements
for k=1:size(proj_points,2)
    if size(proj_points{k},1) < proj_points_size
        if size(proj_points{k},1) == 2
            proj_points{k}=[proj_points{k};repmat(proj_points{k}(end-1:end,:),(proj_points_size-size(proj_points{k},1))/2,1)];
        else
            num_pos_neg = proj_points{k};
            in_out = insideCPS(pos_plane,num_pos_neg);
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

e_dir = e_dir(1,:)/norm(e_dir(1,:));
for u=1:size(proj_points,2)
    for j=1:size(plains,2)
        or_point = proj_points{u}(j*2-1,:);
        curr_point = proj_points{u}(j*2,:);
        distance_point_plain = distPointLine(plains{j},or_point);
        if pdist2(or_point,curr_point)>distance_point_plain
            new_point = or_point+distance_point_plain*e_dir;
            proj_points{u}(j*2,:) = new_point;
        end
        or_point_2 = proj_points{u}(j*2+2,:);
        curr_point_2 = proj_points{u}(j*2+1,:);
        distance_point_plain_2 = distPointLine(plains{j},or_point_2);
        if pdist2(or_point_2,curr_point_2)>distance_point_plain_2
            new_point = or_point+distance_point_plain*e_dir;
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
else
    torsion = 1;
    if torsion 
    res = 0.05;
   CPL_out = CPLselectinout(CPL,0);
   PL_hinge = PLtransR(PLsquare(min_len,hinge_width),rot(deg2rad(hinge_dir)));
   PL_hinge_l = PLtrans(PL_hinge,e_dir_n*max_dim);
   PL_hinge_r = PLtrans(PL_hinge,e_dir_p*max_dim); 
   PL_comb = CPLbool('+',PL_hinge_l,PL_hinge_r);
   inside = inside2C(CPL_out,PL_comb);
   offsat = 0;
   
   CPLplot(PL_comb);
   while  (inside(1)+inside(3)) ~= size(PL_comb,1)
       offsat= offsat+1;
       PL_hinge_l = PLtrans(PL_hinge_l,e_dir_p*res);
       PL_hinge_r = PLtrans(PL_hinge_l,e_dir_n*res);
       PL_comb = CPLbool('+',PL_hinge_l,PL_hinge_r);       
        CPLplot(PL_comb);
       inside = inside2C(CPL_out,PL_comb);     
   end
   
    SG_hinge_l = SGtrans(SG_hinge_zero,[e_dir_p*(max_dim-((offsat-1)*res)) 0]);    
    SG_hinge_r = SGtrans(SG_hinge_zero,[e_dir_n*(max_dim-((offsat-1)*res)) 0]);
    else
        offsat = 3.4+(min_len/2);
     SG_hinge_l = SGtrans(SG_hinge_zero,[e_dir_p*offsat 0]);    
    SG_hinge_r = SGtrans(SG_hinge_zero,[e_dir_n*offsat 0]);
    end
    
    a = 2;
%     SG = SGcat(SG,SG_hinge_new);
    SG = SGcat(SG_hinge_l,SG_hinge_r);
    
    
end
end