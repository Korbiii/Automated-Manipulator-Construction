%%   [SG] = SGconnector(CPL,positions,axis,h_r,push_rod)
%	=== INPUT PARAMETERS ===
%	CPL:                CPL for connector
%   CPL_following:      CPL of following elements
%	positions:          2xn vector of hole positions
%	axis:               2xn vector of axis
%	h_r:                radius of hole for actuator rope
%   end_cap:            Set 1 to generate a end cap
%   hinge_width_b:      hinge width bottom
%   hinge_width_t:      hinge width top
%   single:             single actuator
%   side_stabi:         0 = No stabi 1 = bot; 2 = top
%   cut_orientation:    'x' or 'y' default 'x'
%	=== OUTPUT RESULTS ======
%	SG:         SG of connector element
function [SG] = SGconnector(CPL,CPL_holes,positions,axis_h,h_r,varargin)
%%
hinge_width_b = 1.2;    if nargin>=6 && ~isempty(varargin{1}); hinge_width_b = varargin{1}; end
hinge_width_t = 1.2;    if nargin>=7 && ~isempty(varargin{2}); hinge_width_t = varargin{2}; end
cut_orientation = 'x'; single = 0; end_cap = 0;
for f=3:size(varargin,2)
   switch varargin{f}
       case 'end_cap'
           end_cap = 1;
       case 'single'
           single = 1;
       case 'x'
           cut_orientation = 'x';
       case 'y'
           cut_orientation = 'y';
   end
end




CPL_b = CPL{1};
if size(CPL) == 2
    CPL_f = CPL{2};
else
    CPL_f = CPL{1};
end
CPL_holes_b = CPL_holes{1};
if size(CPL_holes) == 2 
    CPL_holes_f = CPL_holes{2}; 
else
    CPL_holes_f = [];
end

crimp = 2;

%% Shifting all holes to positive x_values
for i=1:size(positions,1)
    if positions(i,1) < 0
        positions(i,:) = -positions(i,:);
    end
end



if crimp == 1
    SG = SGof2CPLsz(CPL_b,CPL_f,12);
    height_SG = max(SG.VL(:,3))-min(SG.VL(:,3));
    SG_holes = SGofCPLz(CPL_holes_b,height_SG-2);
    PL_crimp_hole = CPLconvexhull([PLcircle(h_r*1.5);NaN NaN;PLtrans(PLsquare(h_r*3),[0 -2])]);
    PL_crimp_holes = [];
    for i=1:size(positions,1)
        if single && i == 1
            angle = atan2(positions(i,1),positions(i,2))+pi/2;
            PL_crimp_holes = CPLbool('+',PL_crimp_holes,PLtrans(PLtransR(PL_crimp_hole,rot(angle)),positions(i,:)));
        else
            angle = atan2(positions(i,1),positions(i,2));
            PL_crimp_holes = CPLbool('+',PL_crimp_holes,PLtrans(PLtransR(PL_crimp_hole,rot(pi-angle)),positions(i,:)));
        end
        if i ~=1 || single == 0
            PL_crimp_holes = CPLbool('+',PL_crimp_holes,PLtrans(PLtransR(PL_crimp_hole,rot(-angle)),-positions(i,:)));
        end
    end
    SG_holes_crimp = SGtrans(SGofCPLz(PL_crimp_holes,8),[0 0 2]);
    
    
    SG_holes = SGboolh('+',SG_holes,SG_holes_crimp);
    if ~isempty(CPL_holes_f)
        SG_holes_top = SGofCPLz(CPL_holes_f,3);
        SG_holes = SGboolh('+',SG_holes,SGtrans(SG_holes_top,[0 0 height_SG-2.5]));
    end
    SG = SGbool3('-',SG,SGtrans(SG_holes,[0 0 -0.1]));
    SG = SGtrans(SG,[0 0 (height_SG/2)-max(SG.VL(:,3))]);
else
    CPL_b = CPLbool('-',CPL_b,CPL_holes_b);
    CPL_f_baseholes = CPLbool('-',CPL_f,CPL_holes_b);
    CPL_f = CPLbool('-',CPL_f,CPL_holes_f);
    PL_wireescape = CPLconvexhull([PLcircle(h_r);NaN NaN;PLtrans(PLsquare(h_r*2),[0 -2*h_r])]);
    angle = atan2(positions(1,1),positions(1,2));  %double angle  pi-angle2 -angle2 Angle between centerpoint and holeposition
    [sizex,sizey,~,~,~,~] = sizeVL(CPL_b);
    
    CPL_b_wireescape = CPLbool('-',CPL_b,PLtrans(PLtransR(PL_wireescape,rot(pi-angle)),positions(1,:)));
    CPL_f_wireescape = CPLbool('-',CPL_f_baseholes,PLtrans(PLtransR(PL_wireescape,rot(pi-angle)),positions(1,:))); 
    
        
    if cut_orientation == 'x'
        CPL_b_wirechannels = CPLbool('-',CPL_b_wireescape,PLtrans(PLsquare(3,sizey),[sizex/2 0]));    %% TODO
    else
        width = (sizey/2)-abs(positions(1,2))+(h_r);
        CPL_b_wirechannels = CPLbool('-',CPL_b_wireescape,PLtrans(PLsquare(sizex,width),[0 positions(1,2)-width/2])); %% TODO
    end
    
    if positions(1,1)>0
        CPL_b_wirechannels = CPLbool('-',CPL_b_wirechannels,PLtrans(PLsquare(sizex/2+2,0.8),[-(sizex/2+2)/2 -positions(1,2)])); %% TODO
    else
        CPL_b_wirechannels = CPLbool('-',CPL_b_wirechannels,PLtrans(PLsquare(sizex/2+2,0.8),[(sizex/2+2)/2 positions(1,2)]));  %% TODO
    end
    
    if ~single
        CPL_b_wireescape = CPLbool('-',CPL_b_wireescape,PLtrans(PLtransR(PL_wireescape,rot(-angle)),-positions(1,:)));
        CPL_f_wireescape = CPLbool('-',CPL_f_wireescape,PLtrans(PLtransR(PL_wireescape,rot(-angle)),-positions(1,:)));
        CPL_b_wirechannels = CPLbool('-',CPL_b_wirechannels,PLtrans(PLtransR(PL_wireescape,rot(-angle)),-positions(1,:)));
        if cut_orientation == 'x'
            CPL_b_wirechannels = CPLbool('-',CPL_b_wirechannels,PLtrans(PLsquare(3,sizey),[-sizex/2 0]));    %% TODO
        else
            CPL_b_wirechannels = CPLbool('-',CPL_b_wirechannels,PLtrans(PLsquare(sizex/2,3),[sizex/4 -sizey/2]));    %% TODO
        end
        CPL_b_wirechannels = CPLbool('-',CPL_b_wirechannels,PLtrans(PLsquare(sizex/2+2,0.8),[(sizex/2+2)/2 -positions(1,2)]));
    end    
          
    SG_bottom = SGofCPLz(CPL_b,2);
    SG_wire_layer = SGofCPLz(CPL_b_wirechannels,0.8);
    SG_top_connector = SGof2CPLsz(CPL_b_wireescape,CPL_f_wireescape,2);
    SG_top_layer = SGofCPLz(CPLbool('-',CPL_f,CPL_holes_f),1);
    SG = SGstack('z',SG_bottom,SG_wire_layer,SG_top_connector,SG_top_layer);
    height_SG = max(SG.VL(:,3))-min(SG.VL(:,3));
    SG = SGtrans(SG,[0 0 (height_SG/2)-max(SG.VL(:,3))]);
end

%% add hinge
height = 0.5;

SG_hinge = SGhingeround(0.5,hinge_width_b,height);

SG_hinge_b = SGtrans(SG_hinge,[0 axis_h(1,2) 0]);
SG_hinge_b = SGtransR(SG_hinge_b,rotz(axis_h(1,1)));
SG_hinge_b = SGcreateHinge(CPL_b,SG_hinge_b,axis_h(1,1),axis_h(1,2),hinge_width_b);
SG_hinge_b = SGmirror(SG_hinge_b,'xy');

if ~end_cap
    SG_hinge_t = SGhingeround(0.5,hinge_width_t,height);
    SG_hinge_t = SGtrans(SG_hinge_t,[0 axis_h(2,2) 0]);
    SG_hinge_t = SGtransR(SG_hinge_t,rotz(axis_h(2,1)));
    SG_hinge_t = SGcreateHinge(CPL_f,SG_hinge_t,axis_h(2,1),axis_h(2,2),hinge_width_t);
    SG_hinge_b = SGunder(SG_hinge_b,SG);
    SG_hinge_t = SGontop(SG_hinge_t,SG);
    SG = SGcat(SG_hinge_b,SG_hinge_t,SG);
%     if side_stabi == 2
%         SG = SGcat(SG,SGontop(SG_stabilisator,SG,-1));
%     end
else
    SG = SGcat(SGunder(SG_hinge_b,SG),SG);
end

%% Setting Frames
H_f = [rotx(90)*roty(90+axis_h(2,1)) [-axis_h(2,2);0;((height_SG/2)+height)]; 0 0 0 1];
H_b = [rotx(90)*roty(-90+axis_h(1,1)) [-axis_h(1,2);0;(-(height_SG/2)-height)]; 0 0 0 1];

SG = SGTset(SG,'B',H_b);
SG = SGTset(SG,'F',H_f);
SGwriteSTL(SG,rand+"");
end