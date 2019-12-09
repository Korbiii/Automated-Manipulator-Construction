%%   [SG] = SGmanipulatorbase(CPL,middle_tool_r,tool_pos)
%	=== INPUT PARAMETERS ===
%	CPL:            CPL of bottomelements
%   CPL_out:        CPL of outer contour of elements 
%   tool_radius:    Radius for tool
%   tool_pos:       0: middle 1: top
%	=== OUTPUT RESULTS ======
%	SG:             SG of Manipulatorbase
function [SG] = SGmanipulatorbase(CPL,CPL_out,middle_tool_r,base_point,tool_pos)
%%Generate base CPL based on CPL of bottom element
[sizex,sizey,~,~,~,~] = sizeVL(CPL_out);
if tool_pos == 0
    arm_midpoint = (sizex/2)+middle_tool_r;
elseif tool_pos == 1
    arm_midpoint = (sizex/2)+0.5;
end

PL_crimp_hole = PLtrans(PLcircle(1.3),base_point);
PL_crimp_holes = [PL_crimp_hole;NaN NaN;PLtransR(PL_crimp_hole,rot(pi))];
CPL = CPLbool('+',CPL,PL_crimp_holes);

CPL = PLtrans(CPL,[arm_midpoint 0]);
CPL = [CPL;NaN NaN;PLtransR(CPL,rot(pi))];
if tool_pos == 0
CPL = [CPL;NaN NaN;PLcircle(middle_tool_r);NaN NaN;PLtrans(PLcircle(middle_tool_r),[0 2*middle_tool_r+1]);NaN NaN;PLtrans(PLcircle(middle_tool_r),[0 -2*middle_tool_r-1])];
else
    CPL = [CPL;NaN NaN;PLtrans(PLcircle(middle_tool_r),[0 sizey/2+middle_tool_r-2])];
end
size = sizeVL(CPL);
max_value = max(abs(size(1:4)));
CPL = [CPL;NaN NaN;PLcircle(max_value+1)];
sizeCPL = sizeVL(CPL);
if tool_pos == 1
CPL = CPLbool('-',CPL,PLtrans([PLcircle((max_value+sizex/2));NaN NaN;PLcircle(max_value*4)],[0 abs(sizeCPL(3)-size(3))+1]));
CPL = CPLbool('-',CPL,PLtrans([PLcircle((max_value+sizey/2));NaN NaN;PLcircle(max_value*4)],[abs(sizeCPL(1)-size(1))+sizex/2+1 0]));
CPL = CPLbool('-',CPL,PLtrans([PLcircle((max_value+sizey/2));NaN NaN;PLcircle(max_value*4)],[-abs(sizeCPL(1)-size(1))-sizex/2-1 0]));
end


%%
[sizex,sizey,~,~,~,~] = sizeVL(CPL);
sizeCPL = sizeVL(CPL);
PL_cut_x = PLsquare(sizex+1,(sizey-3)/3);
PL_cut_x = PLtrans([PLtrans(PL_cut_x,[0 (sizey-9)/3]);NaN NaN;PLtrans(PL_cut_x,[0 -(sizey-9)/3])],[0 mean(sizeCPL(3:4))]);
CPL_w_cuts_x = CPLbool('-',CPL,PL_cut_x);

PL_cut_y = [sizex/6 sizey;-sizex/6 sizey;-sizex/6 -sizey;sizex/6 -sizey];
PL_cut_y = [PLtrans(PL_cut_y,[sizex/4 0]);NaN NaN;PLtrans(PL_cut_y,[-sizex/4 0])];
CPL_w_cuts_y = CPLbool('-',CPL,PL_cut_y);

SG_cutless = SGofCPLz(CPL,2);
SG_w_cuts_x = SGofCPLz(CPL_w_cuts_x,2);
SG_w_cuts_y = SGofCPLz(CPL_w_cuts_y,2);

SG = SGstack('z',SG_cutless,SG_w_cuts_x,SG_cutless,SG_w_cuts_y,SG_cutless);
SG = SGstackn(SG,5,0);

height_SG = abs(max(SG.VL(:,3))-min(SG.VL(:,3)));

H_b_b = [rotx(0) [0;0;0]; 0 0 0 1];
H_f_b = [rotx(90) [arm_midpoint;0;height_SG]; 0 0 0 1];
H_f1_b = [rotx(90)*roty(180) [-arm_midpoint;0;height_SG]; 0 0 0 1];
 
SG = SGTset(SG,'B',H_b_b);
SG = SGTset(SG,'F',H_f_b);
SG = SGTset(SG,'F1',H_f1_b);
end