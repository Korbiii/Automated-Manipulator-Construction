%%   [SG] = SGelementstops(CPL)
%	=== INPUT PARAMETERS ===
%   CPL:  CPL of element
%	=== OUTPUT RESULTS ======
%	SG: 	SGstops
function [SG,left_height,right_height] =  SGelementstops(CPL,h_dir,left_angle,right_angle,hinge_width,offset,height)
max_dim = max(sizeVL(CPL))+1;
middle_axis = PLtransR(PLtrans([-max_dim 0;max_dim 0],[0 -offset]),rot(deg2rad(h_dir)));
e_dir = (middle_axis/norm(middle_axis))*rot(pi/2);
e_dir = (e_dir(1,:)-e_dir(2,:))/norm(e_dir(1,:)-e_dir(2,:));
left_plane = [flip(middle_axis);PLtrans(middle_axis,e_dir*max_dim)]; % Plane for finding points in positive area
right_plane = [flip(middle_axis);PLtrans(middle_axis,e_dir*-max_dim)];

CPL_out =  CPLselectinout(CPL,0);
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

width = hinge_width+(2*height);
PL_full = PLtransR(PLtrans( PLsquare(width+1,max_dim*2) ,[offset 0]),rot(deg2rad(h_dir)-pi/2));
CPL_cut = CPLbool('-',CPL,PL_full);

offset_p = floor((max_distance_right/(max_distance_right+max_distance_left))*500);

right_height = max(0,height-(tand(left_angle)*max_distance_left));
left_height = max(0,height-(tand(right_angle)*max_distance_right));

PLcontour = [(linspace(-max_distance_right,max_distance_left,500))+offset;linspace(left_height,height,offset_p) linspace(height,right_height,500-offset_p)]';

SG = SGofCPLz(CPL_cut,0.1);
n=size(SG.VL,1);
PLup=[SG.VL(n/2+1:end,1) SG.VL(n/2+1:end,2)];
VLprojection = PLtoVLprojection(PLup, PLcontour,90-h_dir);
SG.VL = [SG.VL(1:n/2,1) SG.VL(1:n/2,2) SG.VL(1:n/2,3);VLprojection];



end