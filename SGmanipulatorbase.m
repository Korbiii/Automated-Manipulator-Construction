%%   [SG] = SGmanipulatorbase(CPL,middle_tool_r)
%	=== INPUT PARAMETERS ===
%	CPL:            CPL of bottomelements
%   middle_tool_r:  Radius for tool in the middle 
%	=== OUTPUT RESULTS ======
%	SG:             SG of Manipulatorbase
function [SG] = SGmanipulatorbase(CPL,middle_tool_r,base_point)
%%Generate base CPL based on CPL of bottom element
[sizex,~,~,~,~,~] = sizeVL(CPL);
arm_midpoint = (sizex/2)+middle_tool_r+1;
PL_crimp_hole = PLtrans(PLcircle(1.3),base_point);
PL_crimp_holes = [PL_crimp_hole;NaN NaN;PLtransR(PL_crimp_hole,rot(pi))];
CPL = CPLbool('+',CPL,PL_crimp_holes);

CPL = PLtrans(CPL,[arm_midpoint 0]);
CPL = [CPL;NaN NaN;PLtransR(CPL,rot(pi));NaN NaN;PLcircle(middle_tool_r);NaN NaN;PLtrans(PLcircle(middle_tool_r),[0 2*middle_tool_r+1]);NaN NaN;PLtrans(PLcircle(middle_tool_r),[0 -2*middle_tool_r-1])];
[sizex,sizey,~,~,~,~] = sizeVL(CPL);
CPL = [CPL;NaN NaN;PLcircle(max(([sizex,sizey]/2)+2))];


%%
[sizex,sizey,~,~,~,~] = sizeVL(CPL);
PL_cut_x = [-sizex sizey/6;-sizex -sizey/6;sizex -sizey/6;sizex sizey/6];
PL_cut_x = [PLtrans(PL_cut_x,[0 sizey/4]);NaN NaN;PLtrans(PL_cut_x,[0 -sizey/4])];
CPL_w_cuts_x = CPLbool('-',CPL,PL_cut_x);

PL_cut_y = [sizex/6 sizey;-sizex/6 sizey;-sizex/6 -sizey;sizex/6 -sizey];
PL_cut_y = [PLtrans(PL_cut_y,[sizex/4 0]);NaN NaN;PLtrans(PL_cut_y,[-sizex/4 0])];
CPL_w_cuts_y = CPLbool('-',CPL,PL_cut_y);

SG_cutless = SGofCPLz(CPL,2);
SG_w_cuts_x = SGofCPLz(CPL_w_cuts_x,2);
SG_w_cuts_y = SGofCPLz(CPL_w_cuts_y,2);

SG = SGstack([SG_cutless SG_w_cuts_x SG_cutless SG_w_cuts_y SG_cutless]);
SG = SGstackn(SG,5,0);

height_SG = abs(max(SG.VL(:,3))-min(SG.VL(:,3)));

H_b_b = [rotx(0) [0;0;0]; 0 0 0 1];
H_f_b = [rotx(90) [arm_midpoint;0;height_SG]; 0 0 0 1];
H_f1_b = [rotx(90)*roty(180) [-arm_midpoint;0;height_SG]; 0 0 0 1];
 
SG = SGTset(SG,'B',H_b_b);
SG = SGTset(SG,'F',H_f_b);
SG = SGTset(SG,'F1',H_f1_b);
end