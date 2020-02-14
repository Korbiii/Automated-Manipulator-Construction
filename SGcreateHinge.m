%%  [SG] = SGcreateHinge(CPL,SG_hinge,hinge_dir)
%	=== INPUT PARAMETERS ===
%	CPL:        CPL of element you want to create a hinge for
%	SG_hinge:   SG of your hinge
%	hinge_dir:  Direction of hinge
%   hinge_opti: 0 no opti; 1 opti in pos -1 in negative
%	=== OUTPUT RESULTS ======
%	SG:         SG of connector element
%   offset:     Calculated offset in positive y direction
function [SG,offset] = SGcreateHinge(CPL,SG_hinge,hinge_dir,hinge_opti,hinge_width)
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
hinge_width = hinge_width+1;

%% Calculating best offset
if hinge_opti ~= 0
    PL_offsetline = PLtrans(PL_offsetline,e_dir_p*rot(pi/2)*max_dim);
    size_h = 0; res = 0.3; offset = max_dim;
    while size_h < 2
        size_h = 0;
        PL_offsetline = PLtrans(PL_offsetline,e_dir_n*rot(pi/2)*res);
        offset = offset-res;
        c_p = PLcrossCPLLine2(PL_offsetline,CPL);
        if ~isempty(c_p)
            c_p = sortrows(c_p);
            for c=1:size(c_p,2)   
                for k=c+1:size(c_p,2)
                    dis = pdist2(c_p(c,:),c_p(k,:));
                    if dis > 1
                        PL_hinge_area = [c_p(c,:);PLtrans(c_p(c,:),e_dir_n*rot(pi/2)*hinge_width);PLtrans(c_p(k,:),e_dir_n*rot(pi/2)*hinge_width);c_p(k,:)];
                        inside = inside2C(CPL,PL_hinge_area);
                        if inside(1) ~= 0
                            size_h = size_h+dis;
                        end
                    end
                end
            end
        end
        
    end
    offset = (offset-(hinge_width/2));
SG_hinge = SGtrans(SG_hinge,[e_dir_p*rot(pi/2)*offset 0]);
end
SG_hinge = SGtrans(SG_hinge,[e_dir(2,:)*15 0]); 
VL_hinge = SG_hinge.VL;

SG =[];
proj_points = {};

%%  Generating all cross points between CPL and hingepoint projections
for i=1:size(VL_hinge,1)
    PL_cross_line = [VL_hinge(i,1:2);VL_hinge(i,1:2)+(e_dir*50)];
    c_p =  PLcrossCPLLine2(PL_cross_line,CPL);
    c_p(:,3) = pdist2(c_p,VL_hinge(i,1:2));
    c_p = sortrows(c_p,3);
    c_p = c_p(:,1:2);
    proj_points{end+1} = c_p;   
end
%% Getting number of hinge elements below and above axis through [0 0]
[proj_points_size, max_index] = max(cellfun('size', proj_points, 1));
num_pos_neg = proj_points{max_index};
num_pos_neg = (middle_axis(2,1)-middle_axis(1,1))*(num_pos_neg(:,2)-middle_axis(1,2))-(num_pos_neg(:,1)-middle_axis(1,1))*(middle_axis(2,2)-middle_axis(1,2));
positive_gl = length(find(num_pos_neg<0));
negative_gl = proj_points_size-positive_gl;
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
max_axis = proj_points{max_index};
plains = {};
for i=1:2:size(max_axis,1)-2
    axis_offset = [mean(max_axis(i+1:i+2,1)),mean(max_axis(i+1:i+2,2))];
    plains{end+1} = [axis_offset;axis_offset+(e_dir*rot(pi/2))];
end
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
end