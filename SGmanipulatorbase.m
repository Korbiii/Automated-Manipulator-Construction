%%   [SG] = SGmanipulatorbase(CPL,middle_tool_r,tool_pos)
%	=== INPUT PARAMETERS ===
%	CPL:            CPL of bottomelements
%   CPL_out:        CPL of outer contour of elements 
%   tool_radius:    Radius for tool
%   tool_pos:       0: middle 1: top
%	=== OUTPUT RESULTS ======
%	SG:             SG of Manipulatorbase
function [SG] = SGmanipulatorbase(CPL,CPL_out,varargin)
optic_radius=3.25;      if nargin>=3 && ~isempty(varargin{1});  optic_radius=varargin{1};   end
base_point=3;           if nargin>=4 && ~isempty(varargin{2});  base_point=varargin{2};     end
sensor_channel=0;       if nargin>=5 && ~isempty(varargin{3});  sensor_channel=varargin{3}; end
optic_channel = 0;      if nargin>=6 && ~isempty(varargin{4});  optic_channel=varargin{4};  end
single = 0;             if nargin>=7 && ~isempty(varargin{5});  single=varargin{5};         end
length = 50;             if nargin>=8 && ~isempty(varargin{6});  length=varargin{6};         end
seal = 0;               if nargin>=9 && ~isempty(varargin{7});  seal=varargin{7};           end
%%Generate base CPL based on CPL of bottom element
length = floor(length/10);
[sizex,sizey,~,~,~,~] = sizeVL(CPL_out);
if optic_channel == 1
    arm_midpoint = (sizex/2)+optic_radius;
else
    arm_midpoint = (sizex/2)+0.5;
end

PL_crimp_holes = PLtrans(PLcircle(0.8),base_point);
if ~single
    PL_crimp_holes = [PL_crimp_holes;NaN NaN;PLtransR(PL_crimp_holes,rot(pi))];
end
CPL = CPLbool('+',CPL,PL_crimp_holes);

CPL = PLtrans(CPL,[arm_midpoint 0]);
CPL = [CPL;NaN NaN;PLtransR(CPL,rot(pi))];

if optic_channel == 1
    CPL = [CPL;NaN NaN;PLcircle(optic_radius)];
elseif optic_channel == 2
    CPL = [CPL;NaN NaN;PLtrans(PLcircle(optic_radius),[0 sizey/2+optic_radius-2])];
end

size = sizeVL(CPL);
max_value = max(abs(size(1:4)));
CPL_only_holes = CPL;
CPL = [CPL;NaN NaN;PLcircle(max_value+1)];
sizeCPL = sizeVL(CPL);
if optic_channel == 2  || optic_channel == 0
    CPL = CPLbool('-',CPL,PLtrans([PLcircle((max_value+sizex/2));NaN NaN;PLcircle(max_value*4)],[0 abs(sizeCPL(3)-size(3))+1]));
    CPL = CPLbool('-',CPL,PLtrans([PLcircle((max_value+sizey/2));NaN NaN;PLcircle(max_value*4)],[abs(sizeCPL(1)-size(1))+sizex/2+1 0]));
    CPL = CPLbool('-',CPL,PLtrans([PLcircle((max_value+sizey/2));NaN NaN;PLcircle(max_value*4)],[-abs(sizeCPL(1)-size(1))-sizex/2-1 0]));
else
    CPL = CPLbool('-',CPL,PLtrans([PLcircle((max_value+sizex/2));NaN NaN;PLcircle(max_value*4)],[0 abs(sizeCPL(3)-size(3))-2]));
    CPL = CPLbool('-',CPL,PLtrans([PLcircle((max_value+sizex/2));NaN NaN;PLcircle(max_value*4)],[0 -abs(sizeCPL(3)-size(3))+2]));
end
% CPL = [CPL;NaN NaN;PLtrans(PLcircle(1.5),[0 -7.5])];

if sensor_channel
    CPL_outer_contour = CPLselectinout(CPL,0);
    CPL_calibration = [PLcircle(20);NaN NaN;CPLgrow(CPL_outer_contour,0.2)];
    SG_calibration = SGofCPLz(CPL_calibration,10);
    
    CPL_out_2 = PLtrans(CPL_out,[arm_midpoint 0]);
    CPL_out_2 = [CPL_out_2;NaN NaN;PLtransR(CPL_out_2,rot(pi))];
    CPL_calib_top_2 = [PLcircle(20);NaN NaN;CPLgrow(CPL_out_2,0.2)];
    SG_top_calibration = SGofCPLz(CPL_calib_top_2,10);
    PL_connector = PLcircularpattern(PLcircle(5),17.5,pi/2,4);
    PL_connector = CPLbool('-',PL_connector,[PLcircle(20);NaN NaN;PLcircle(30)]);
    SG_connector = SGofCPLz(PL_connector,135);    
    
    SG_calib_tool = SGstack('z',SG_calibration,SG_connector,SG_top_calibration);
    SGwriteSTL(SG_calib_tool,"SGcalibtool",'','y');
    
    PL_plugs = [1 0;1.4 0;1.4 10;3 10;3 13;1.25 13;1.25 3;1 3];
    SG_plugs = SGofCPLrota(PL_plugs,1.9*pi,false);
    SGwriteSTL(SG_plugs,"SGplugs",'','y'); 
end



%%
[sizex,sizey,~,~,~,~] = sizeVL(CPL);
sizeCPL = sizeVL(CPL);
PL_cut_x = PLsquare(sizex+10,(sizey-3)/3);
PL_cut_x = PLtrans([PLtrans(PL_cut_x,[0 (sizey-9)/3]);NaN NaN;PLtrans(PL_cut_x,[0 -(sizey-9)/3])],[0 mean(sizeCPL(3:4))]);


PL_cut_y = [sizex/6 sizey+10;-sizex/6 sizey;-sizex/6 -sizey;sizex/6 -sizey-10];
PL_cut_y = [PLtrans(PL_cut_y,[sizex/4 0]);NaN NaN;PLtrans(PL_cut_y,[-sizex/4 0])];
if length > 1
    CPL_w_cuts_x = CPLbool('-',CPL,PL_cut_x);
    CPL_w_cuts_y = CPLbool('-',CPL,PL_cut_y);
else
    CPL_w_cuts_y = CPL;
    CPL_w_cuts_x = CPL;
end

SG_cutless = SGofCPLz(CPL,2);
SG_w_cuts_x = SGofCPLz(CPL_w_cuts_x,2);
SG_w_cuts_y = SGofCPLz(CPL_w_cuts_y,2);

SG = SGstack('z',SG_cutless,SG_w_cuts_x,SG_cutless,SG_w_cuts_y,SG_cutless);
SG = SGstackn(SG,length,0);

if seal
    
    CPL_out = CPLselectinout(CPL,0);
    CPL_add_on = [CPL_out;NaN NaN;PLcircle(max_value+2)];
    CPL_bayo_holder = [CPL_out;NaN NaN;PLcircle(max_value+4)];
    CPL_add_on_w_cuts_y = CPLbool('-',CPL_add_on,PL_cut_x);
    CPL_add_on_w_cuts_x = CPLbool('-',CPL_add_on,PL_cut_y);
    SG_add_on = SGofCPLz(CPL_add_on,2);
    SG_add_on_w_cuts_y = SGofCPLz(CPL_add_on_w_cuts_y,2);
    SG_add_on_w_cuts_x = SGofCPLz(CPL_add_on_w_cuts_x,2);
    SG_bayo_holder = SGofCPLz(CPL_bayo_holder,4);
    SG_add_on_stack = SGstack('z',SG_add_on,SG_add_on_w_cuts_y,SG_add_on,SG_add_on_w_cuts_x,SG_add_on,SG_bayo_holder);
    SG_add_on_stack = SGtransrelSG(SG_add_on_stack,SG,'alignbottom');
    
    
    PL_bayo_grip = [PLcircle(max_value+8);NaN NaN;PLcircle(max_value+12)];
    PL_bayo_grip = CPLbool('-',PL_bayo_grip,CPLcopyradial(PLcircle(8),max_value+18,8));
    SG_bayo_grip = SGofCPLz(PL_bayo_grip,12);
    SG_bayo_top_ring = SGofCPLz([PLcircle(max_value+2);NaN NaN;PLcircle(max_value+8)],2);
    SG_bayo_top_ring = SGontop(SG_bayo_top_ring,SG_add_on_stack);
    SG_bayo_grip = SGaligntop(SG_bayo_grip,SG_bayo_top_ring);
    
    PL_square = PLtrans(PLsquare(max_value*10,max_value/2),[max_value*10/2,max_value/4]);
    PL_bayo_middle = PL_square;
    for k = 1:4
        PL_bayo_middle =  CPLbool('+',PL_bayo_middle,PLtransC(PL_square,[0 0],rot((pi/2)*k)));
    end
    
    PL_bayo_middle = CPLbool('-',PL_bayo_middle,PLcircle(max_value+4.15));
    PL_bayo_middle = CPLbool('-',PL_bayo_middle,[PLcircle(max_value*100);NaN NaN;PLcircle(max_value+8)]);
    SG_bayo_middle = SGofCPLz(PL_bayo_middle,6);
    SG_bayo_middle = SGunder(SG_bayo_middle,SG_bayo_top_ring);
    
    
    PL_bayo_bottom = CPLbool('+',PLsquare(max_value*10,max_value),PLsquare(max_value,max_value*10));
    PL_bayo_bottom = CPLbool('-',PL_bayo_bottom,PLcircle(max_value+4.15));
    PL_bayo_bottom = CPLbool('-',PL_bayo_bottom,[PLcircle(max_value*100);NaN NaN;PLcircle(max_value+8)]);
    SG_bayo_bottom = SGofCPLz(PL_bayo_bottom,4);
    SG_bayo_bottom = SGunder(SG_bayo_bottom,SG_bayo_middle);
    
    SG_bayonett = SGcat(SG_bayo_grip,SG_bayo_bottom,SG_bayo_middle,SG_bayo_top_ring);
    
    SG = SGcat(SG,SG_add_on_stack,SG_bayonett);   
    
    
    
    CPL_seal_cup = [PLcircle(max_value+2.15);NaN NaN;PLcircle(max_value+4)];
    CPL_seal_cup_bottom = CPLbool('-',PLcircle(max_value+4),CPL_only_holes);
    SG_seal_cup = SGofCPLz(CPL_seal_cup,8);
    SG_seal_cup_bottom = SGofCPLz(CPL_seal_cup_bottom,3);
    SG_seal_cup = SGstack('z',SG_seal_cup_bottom,SG_seal_cup);
    SG_seal_cup = SGtransrelSG(SG_seal_cup,SG,'alignbottom',2);


end



%% Add Frames
height_SG = abs(max(SG.VL(:,3))-min(SG.VL(:,3)));

H_b_b = [rotx(0) [0;0;0]; 0 0 0 1];
H_f_b = [rotx(90) [arm_midpoint;0;height_SG]; 0 0 0 1];
H_f1_b = [rotx(90)*roty(180) [-arm_midpoint;0;height_SG]; 0 0 0 1];
 
SG = SGTset(SG,'B',H_b_b);
SG = SGTset(SG,'F',H_f_b);
SG = SGTset(SG,'F1',H_f1_b);
if seal
    SG = SGcat(SG,SG_seal_cup);
end
end