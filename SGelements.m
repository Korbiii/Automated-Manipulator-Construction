%%  [SG] =  SGelements(CPL,angle_p,length_p,flags)
%	=== INPUT PARAMETERS ===
%	CPL:                CPL of element with tool hole
%   angle_p:            [hinge_dir, hinge_optimization]
%   length_p:           [element_height,hinge_width,min_length,hinge_height]
%   === FLAGS ==============
%   'bottom_element':   Creates element with only top hinge
%   'non_torsion':      
%	=== OUTPUT RESULTS =====
%	SG:             SG of element
function [SG,offset] = SGelements(CPL,angle_p,length_p,varargin)
%% Initializing
bottom_ele = 0; torsion = 1;
for f=1:size(varargin,2)
   switch varargin{f}
       case 'bottom_element'
           bottom_ele = 1;     
           if angle_p(2) == 2, angle_p(2) = 0; end
%            length_p(1) = 5;
       case 'non_torsion'
           torsion = 0;
   end
end

%% Creating base and zeroing it in z
SG = SGofCPLzdelaunayGrid(CPL, length_p(1),0.5,0.5);
hight_SG = abs(max(SG.VL(:,3))-min(SG.VL(:,3)));
SG = SGtrans(SG,[0 0 -hight_SG/2]);

%% Add hinge to base
SG_hinge = SGhingeround(length_p(3),length_p(2), length_p(4));

SG_hinge = SGtransR(SG_hinge,rotz(angle_p(1)));
SG_hinge_2 = SGtransR(SG_hinge,rotz(-90));
[SG_hinge,offset]= SGcreateHinge(CPL,SG_hinge,angle_p(1),angle_p(2),length_p(2),length_p(3), length_p(4));

if length_p(3) ~= 0
    cp = PLcrossCPLLine2(PLtransC([-500 0;500 0],[0 0],deg2rad(angle_p(1))),CPL);
    dis_in_lef = pdist2(cp(3,:),[0 0]);
    dis_in_rig = pdist2(cp(4,:),[0 0]);
    dis_out_lef = pdist2(cp(1,:),[0 0]);
    dis_out_rig = pdist2(cp(2,:),[0 0]);
    SG_hinge =  SGtransR(SG_hinge,rotz(-angle_p(1)));
    if torsion
        distance_whole = dis_out_lef+dis_out_rig;
        SG_hinge_l = SGcutend(SG_hinge,'right',distance_whole-length_p(3));
        SG_hinge_r =  SGcutend(SG_hinge,'left',distance_whole-length_p(3));
        SG_hinge = SGcat(SG_hinge_l,SG_hinge_r);
    else
        left_cut_distance = (dis_out_lef-dis_in_lef);
        right_cut_distance = (dis_out_rig-dis_in_rig);
        SG_hinge = SGcutend(SG_hinge,'left',left_cut_distance-length_p(3));
        SG_hinge = SGcutend(SG_hinge,'right',right_cut_distance-length_p(3));
    end
    SG_hinge =  SGtransR(SG_hinge,rotz(angle_p(1)));
end



SG_hinge_top = SGontop(SG_hinge,SG);
SG_hinge_bottom = SGmirror(SG_hinge_top,'xy');

if angle_p(2) == 2
    [SG_hinge_2,offset_2]= SGcreateHinge(CPL,SG_hinge_2,angle_p(1)-90,angle_p(2),length_p(2),length_p(3), length_p(4));   
    
    if length_p(3) ~= 0
        cp = PLcrossCPLLine2(PLtransC([-500 0;500 0],[0 0],deg2rad(-angle_p(1)+90)),CPL);
        dis_in_lef = pdist2(cp(3,:),[0 0]);
        dis_in_rig = pdist2(cp(4,:),[0 0]);
        dis_out_lef = pdist2(cp(1,:),[0 0]);
        dis_out_rig = pdist2(cp(2,:),[0 0]);
        SG_hinge_2 =  SGtransR(SG_hinge_2,rotz(-angle_p(1)+90));
        if torsion
            distance_whole = dis_out_lef+dis_out_rig;
            SG_hinge_l = SGcutend(SG_hinge_2,'right',distance_whole-length_p(3));
            SG_hinge_r =  SGcutend(SG_hinge_2,'left',distance_whole-length_p(3));
            SG_hinge_2 = SGcat(SG_hinge_l,SG_hinge_r);
        else
            left_cut_distance = (dis_out_lef-dis_in_lef);
            right_cut_distance = (dis_out_rig-dis_in_rig);
            SG_hinge_2 = SGcutend(SG_hinge_2,'left',left_cut_distance-length_p(3));
            SG_hinge_2 = SGcutend(SG_hinge_2,'right',right_cut_distance-length_p(3));
        end
        SG_hinge_2 =  SGtransR(SG_hinge_2,rotz(angle_p(1)-90));        
        
    end
    SG_hinge_2_top = SGontop(SG_hinge_2,SG);
    SG_hinge_2_bottom = SGmirror(SG_hinge_2_top,'xy');
end


if ~bottom_ele    
    if angle_p(2) == 2
        SG_base_ = SG;
        SG = SGcat(SG_base_,SG_hinge_bottom,SG_hinge_2_top);
        SG_2 = SGcat(SG_base_,SG_hinge_2_bottom,SG_hinge_top);
        
    else
        SG = SGcat(SG,SG_hinge_top,SG_hinge_bottom);
    end    
else
    SG = SGcat(SG,SG_hinge_top);
end

%% Add frames to element
e_dir = [sind(angle_p(1)) -cosd(angle_p(1))];
H_f = TofR(rotx(90)*roty(90+angle_p(1)),[offset*-e_dir  length_p(4)+(hight_SG/2)]);
H_f_2 = TofR(rotx(90)*roty(180+angle_p(1)),[offset*-e_dir length_p(4)+(hight_SG/2)]);
if ~bottom_ele
    H_b = TofR(rotx(90)*roty(-90+angle_p(1)),[offset*-e_dir  -(hight_SG/2)-length_p(4)]);
    H_b_2 = TofR(rotx(90)*roty(angle_p(1)),[offset*-e_dir  -(hight_SG/2)-length_p(4)]);
else
    H_b = [rotx(90)*roty(180) [0;0;(-(hight_SG/2))]; 0 0 0 1];
end
if angle_p(2) == 2    
    SG = SGTset(SG,'F',H_f_2);
    SG = SGTset(SG,'B',H_b);
    SG_2 = SGTset(SG_2,'F',H_f);
    SG_2 = SGTset(SG_2,'B',H_b_2);
    SG.offest = offset;
    SG_2.offset = offset_2;
    SG = {SG SG_2};
else
    SG = SGTset(SG,'F',H_f);
    SG = SGTset(SG,'B',H_b);
    SG.offest = offset;
end


end