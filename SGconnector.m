%%   [SG] = SGconnector(CPL,positions,axis,h_r,push_rod)
%	=== INPUT PARAMETERS ===
%	CPL:        CPL for connector
%   CPL_following: CPL of following elements
%	positions:  2xn vector of hole positions
%	axis:   	2xn vector of axis
%	h_r:        radius of hole for actuator rope
%   end_cap:    Set 1 to generate a end cap
%   hinge_width_b
%   hinge_width_t
%   single
%   side_stabi: 0: No stabi 1: bot; 2: top
%	=== OUTPUT RESULTS ======
%	SG:         SG of connector element
function [SG] = SGconnector(CPL,CPL_following,CPL_holes,CPL_holes_following,positions,axis_h,h_r,varargin)
%%
end_cap = 0;            if nargin>=8 && ~isempty(varargin{1}); end_cap = varargin{1};       end
hinge_width_b = 1.2;    if nargin>=9 && ~isempty(varargin{2}); hinge_width_b = varargin{2}; end
hinge_width_t = 1.2;    if nargin>=10 && ~isempty(varargin{3}); hinge_width_t = varargin{3}; end
single = 0;               if nargin>=11 && ~isempty(varargin{4}); single = varargin{4};          end
side_stabi = 0;         if nargin>=12 && ~isempty(varargin{5}); side_stabi = varargin{5};   end

crimp = 1;
CPL_out = CPLselectinout(CPL,0);

%% Shifting all holes to positive x_values
for i=1:size(positions,1)
    if positions(i,1) < 0
        positions(i,:) = -positions(i,:);
    end
end

SG = SGof2CPLsz(CPL,CPL_following,12);
height_SG = max(SG.VL(:,3))-min(SG.VL(:,3));

SG_holes = SGofCPLz(CPL_holes,height_SG-2);

if crimp == 1
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
else
    PL_crimp_hole = CPLconvexhull([PLcircle(h_r*1.5);NaN NaN;PLtrans(PLsquare(h_r*3),[0 -2])]);
    PL_wire_hole = CPLconvexhull([PLcircle(h_r);NaN NaN;PLtrans(PLsquare(h_r*2),[0 -2])]);
    PL_crimp_holes = [];
    PL_wire_holes = [];
    angle1 = atan2(positions(1,1),positions(1,2))+pi/2;   %single angle
    angle2 = atan2(positions(1,1),positions(1,2));        %double angle
    if single
        PL_wire_holes = CPLbool('+',PL_wire_holes,PLtrans(PLtransR(PL_wire_hole,rot(angle1)),positions(1,:)));
        SG_holes_horizontal = SGtrans(SGofCPLx(PLcircle(h_r),20),[-10 -positions(1,1)-h_r height_SG/2]);
    else
        PL_wire_holes = CPLbool('+',PL_wire_holes,PLtrans(PLtransR(PL_wire_hole,rot(pi-angle2)),positions(1,:)));
        PL_wire_holes = CPLbool('+',PL_wire_holes,PLtrans(PLtransR(PL_wire_hole,rot(-angle2)),-positions(1,:)));
        SG_holes_horizontal = SGboolh('+',SGtrans(SGofCPLx(PLcircle(h_r),20),[-10 -positions(1,1)-h_r height_SG/2]),SGtrans(SGofCPLx(PLcircle(h_r),20),[-10 positions(1,1)+h_r height_SG/2]));
    end
    SG_holes_crimp = SGtrans(SGofCPLz(PL_wire_holes,height_SG/2),[0 0 height_SG/2+0.1]);
    SG_holes_crimp = SGboolh('+',SG_holes_horizontal,SG_holes_crimp);
end

SG_holes = SGboolh('+',SG_holes,SG_holes_crimp);
if ~isempty(CPL_holes_following)
    SG_holes_top = SGofCPLz(CPL_holes_following,3);
    SG_holes = SGboolh('+',SG_holes,SGtrans(SG_holes_top,[0 0 height_SG-2.5]));
end
SG = SGbool3('-',SG,SGtrans(SG_holes,[0 0 -0.1]));
SG = SGtrans(SG,[0 0 (height_SG/2)-max(SG.VL(:,3))]);
%% add hinge
height = 0.5;

SG_hinge = SGhingeround(0.5,hinge_width_b,height);

SG_hinge_b = SGtrans(SG_hinge,[0 axis_h(1,2) 0]);
SG_hinge_b = SGtransR(SG_hinge_b,rotz(axis_h(1,1)));
SG_hinge_b = SGcreateHinge(CPL,SG_hinge_b,axis_h(1,1));
SG_hinge_b = SGmirror(SG_hinge_b,'xy');

if ~end_cap
    SG_hinge_t = SGhingeround(0.5,hinge_width_t,height);
    SG_hinge_t = SGtrans(SG_hinge_t,[0 axis_h(2,2) 0]);
    SG_hinge_t = SGtransR(SG_hinge_t,rotz(axis_h(2,1)));
    SG_hinge_t = SGcreateHinge(CPL_following,SG_hinge_t,axis_h(2,1));
    SG_hinge_b = SGunder(SG_hinge_b,SG);
    SG_hinge_t = SGontop(SG_hinge_t,SG);
    SG = SGcat(SG_hinge_b,SG_hinge_t,SG);
    if side_stabi == 2
        SG = SGcat(SG,SGontop(SG_stabilisator,SG,-1));
    end
else
    SG = SGcat(SGunder(SG_hinge_b,SG),SG);
end

%% Setting Frames
H_f = [rotx(90)*roty(90+axis_h(2,1)) [-axis_h(2,2);0;((height_SG/2)+height)]; 0 0 0 1];
H_b = [rotx(90)*roty(-90+axis_h(1,1)) [-axis_h(1,2);0;(-(height_SG/2)-height)]; 0 0 0 1];

SG = SGTset(SG,'B',H_b);
SG = SGTset(SG,'F',H_f);
end