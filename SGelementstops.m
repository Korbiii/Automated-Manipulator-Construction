%%   [SG] = SGelementstops(CPL)
%	=== INPUT PARAMETERS ===
%   CPL:  CPL of element
%	=== OUTPUT RESULTS ======
%	SG: 	SGstops
function [SG] =  SGelementstops(CPL,h_dir,left_angle,right_angle,hinge_width,offset)
max_dim = max(sizeVL(CPL))+1;
middle_axis = PLtransR(PLtrans([-max_dim 0;max_dim 0],[0 offset]),rot(deg2rad(h_dir)));
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

PLsquare_left = PLsquare(max(hinge_width+1,max_distance_left-2),max_dim*2);
PLsquare_right = PLsquare(max(hinge_width+1,max_distance_right-2),max_dim*2);
PLsquare_left = PLtransR(PLtrans(PLsquare_left,[-offset+(max(hinge_width+1,max_distance_left-2)/2) 0]),rot(deg2rad(h_dir)-pi/2));
PLsquare_right= PLtransR(PLtrans(PLsquare_right,[-offset-(max(hinge_width+1,max_distance_right-2)/2) 0]),rot(deg2rad(h_dir)-pi/2));

PL_cut = CPLbool('+',PLsquare_left,PLsquare_right);
CPL_cut = CPLbool('-',CPL,PL_cut);

offset_p = floor((max_distance_right/(max_distance_right+max_distance_left))*500);

right_height = max(0,0.5-(tand(left_angle)*max_distance_left));
left_height = max(0,0.5-(tand(right_angle)*max_distance_right));

PLcontour = [(linspace(-max_distance_right,max_distance_left,500))-offset;linspace(left_height,0.5,offset_p) linspace(0.5,right_height,500-offset_p)]';

SG = SGofCPLz(CPL_cut,0.1);
n=size(SG.VL,1);
PLup=[SG.VL(n/2+1:end,1) SG.VL(n/2+1:end,2)];
VLprojection = PLtoVLprojection(PLup, PLcontour,90-h_dir);
SG.VL = [SG.VL(1:n/2,1) SG.VL(1:n/2,2) SG.VL(1:n/2,3);VLprojection];



end