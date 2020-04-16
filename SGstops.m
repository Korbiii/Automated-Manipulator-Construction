%%   [SG] = SGelementstops(CPL)
%	=== INPUT PARAMETERS ===
%   CPL:  CPL of element
%	=== OUTPUT RESULTS ======
%	SG: 	SGstops
function [SG,left_height,right_height] =  SGstops(SGs,CPL_out,h_dir,offset,left_angle,right_angle,length_p)
if size(SGs,2) == 2 
    is_connector = 1; 
    SG = SGs{1};
    SG_2 = SGs{2};
else
    is_connector = 0;
    SG = SGs;
end

if h_dir <= 0 
%     offset = -offset;
end
hinge_width = length_p(:,1);
height = length_p(:,2);
height_SG = abs(max(SG.VL(:,3))-min(SG.VL(:,3)));

if is_connector
    height_SG = [height_SG;abs(max(SG_2.VL(:,3))-min(SG_2.VL(:,3)))];
    e_h_1 =  (height_SG(1) - sum(height(1:2)) -sum(hinge_width(1:2))/2)/2;
    e_h_2 =  (height_SG(2) - sum(height(2:3)) -sum(hinge_width(2:3))/2)/2;
    ele_height = [e_h_1,e_h_2];
    height = height(2);
    hinge_width = hinge_width(2);
else    
    ele_height = (height_SG - (2*height) -(hinge_width))/2;
end


max_dim = max(sizeVL(CPL_out))+1;
middle_axis = PLtransR(PLtrans([-max_dim 0;max_dim 0],[0 -offset]),rot(deg2rad(h_dir)));
e_dir = (middle_axis/norm(middle_axis))*rot(pi/2);
e_dir = (e_dir(1,:)-e_dir(2,:))/norm(e_dir(1,:)-e_dir(2,:));
left_plane = [flip(middle_axis);PLtrans(middle_axis,e_dir*10*max_dim)]; % Plane for finding points in positive area
right_plane = [flip(middle_axis);PLtrans(middle_axis,e_dir*10*-max_dim)];

CPL_out_left = CPLbool('-',CPL_out,left_plane);
CPL_out_right = CPLbool('-',CPL_out,right_plane);

max_distance_left = 0;
for k=1:size(CPL_out_left,1)
    temp_dis = distPointLine(middle_axis,CPL_out_left(k,:));
    if temp_dis>max_distance_left
        max_distance_left = temp_dis;
    end
end
max_distance_right = 0;
for k=1:size(CPL_out_right,1)
     temp_dis = distPointLine(middle_axis,CPL_out_right(k,:));
    if temp_dis>max_distance_right
        max_distance_right = temp_dis;
    end
end


offset_p = floor((max_distance_right/(max_distance_right+max_distance_left))*500);

right_height = height-(tand(left_angle)*max_distance_left);
left_height = height-(tand(right_angle)*max_distance_right);

PLcontour = [(linspace(-max_distance_right,max_distance_left,500))+offset;linspace(left_height,height,offset_p) linspace(height,right_height,500-offset_p)]';

VLcontour = [PLcontour(:,1) zeros(size(PLcontour,1),1) PLcontour(:,2) ]; %create  VLcontour to plot it in vertical plane (x-z-plane)
VLcontour = VLtrans(VLcontour,TofR(rotz(90-h_dir)));
[~,max_row] = max(VLcontour);
dist_hinge = distPointLine(middle_axis,VLcontour(max_row(3),1:2));
if dist_hinge>0.1 
    VLcontour = VLtrans(VLcontour,TofR(rotz(180)));
end

if is_connector
    VLcontour_up = VLtrans(VLcontour,[0 0 ele_height(1)]);
    VLcontour_down = VLtrans(VLswapZ(VLcontour),[0 0 -ele_height(2)]);
    index_up = find(SG.VL(:,3) > ele_height(1)-0.1 & SG.VL(:,3) < ele_height(1)+0.1 );
    index_down = find(SG_2.VL(:,3) < -ele_height(2)+0.1 & SG_2.VL(:,3) > -ele_height(2)-0.1);
else
    VLcontour_up = VLtrans(VLcontour,[0 0 ele_height]);
    VLcontour_down = VLswapZ(VLcontour_up);
    index_down = find(SG.VL(:,3) > -ele_height-0.1 & SG.VL(:,3) < -ele_height+0.1 );
    index_up = find(SG.VL(:,3) < ele_height+0.1 & SG.VL(:,3) > ele_height-0.1);
end



for i=1:size(index_up,1)
    [~,idx] = min(pdist2(SG.VL(index_up(i),1:2),VLcontour_up(:,[1,2])));
    if distPointLine(middle_axis, SG.VL(index_up(i),1:2))>(hinge_width+0.5)
        SG.VL(index_up(i),3) = VLcontour_up(idx,3);
    end
end


if is_connector    
    for i=1:size(index_down,1)
        [~,idx] = min(pdist2(SG_2.VL(index_down(i),1:2),VLcontour_down(:,[1,2])));
        if distPointLine(middle_axis,SG_2.VL(index_down(i),1:2))>(hinge_width+0.5)
            SG_2.VL(index_down(i),3) = VLcontour_down(idx,3);
        end
    end
    SG = {SG;SG_2};
else    
    for i=1:size(index_down,1)
        [~,idx] = min(pdist2(SG.VL(index_down(i),1:2),VLcontour_down(:,[1,2])));
        if distPointLine(middle_axis,SG.VL(index_down(i),1:2))>(hinge_width+0.5)
            SG.VL(index_down(i),3) = VLcontour_down(idx,3);
        end
    end
end


end