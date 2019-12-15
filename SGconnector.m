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
%   push
%   side_stabi: 0: No stabi 1: bot; 2: top
%	=== OUTPUT RESULTS ======
%	SG:         SG of connector element
function [SG] = SGconnector(CPL,CPL_following,CPL_out,positions,axis,h_r,varargin)
%%
end_cap = 0;            if nargin>=7 && ~isempty(varargin{1}); end_cap = varargin{1};       end
hinge_width_b = 1.2;    if nargin>=8 && ~isempty(varargin{2}); hinge_width_b = varargin{2}; end
hinge_width_t = 1.2;    if nargin>=9 && ~isempty(varargin{3}); hinge_width_t = varargin{3}; end
push = 0;               if nargin>=10 && ~isempty(varargin{4}); push = varargin{4};          end
side_stabi = 0;         if nargin>=11 && ~isempty(varargin{5}); side_stabi = varargin{5};   end
%% Shifting all holes to positive x_values
for i=1:size(positions,1)
    if positions(i,1) < 0
        positions(i,:) = -positions(i,:);
    end
end
%% Top&Bottom Layer
SG_bt_b = SGofCPLz(CPL,2);
SG_bt_t = SGofCPLz(CPL_following,2);
maxY = max(CPL(:,2));
maxX = max(CPL(:,1));
cut_x = 2.5;
cut_y = 4;
PL_stabi_cut = PLsquare(cut_x,cut_y);
PL_stabi_cut = PLtrans(PL_stabi_cut,[maxX-1.5 maxY-1]);
PL_stabi_cut = CPLbool('+',PL_stabi_cut,VLswapY(VLswapX(PL_stabi_cut)));
switch side_stabi    
    case 1  % Bottom
        CPL_2 = CPLbool('-',CPL,VLswapY(PL_stabi_cut));
        CPL_2 = CPLbool('-',CPL_2,PL_stabi_cut);
        SG_bt_b = SGofCPLz(CPL_2,2);
    case 2      % Top
        PL_stabilisator_or = PLtrans(PLsquare(cut_x-0.4,cut_y-0.4),[0 0.2]);
        PL_stabilisator_or = PLtrans(PL_stabilisator_or,[-maxX+1.5 maxY-1]);
        PL_stabilisator_or =  CPLbool('+',PL_stabilisator_or,VLswapY(VLswapX(PL_stabilisator_or)));
        PL_stabilisator = CPLbool('x',VLswapY(PL_stabilisator_or),CPL);
        PL_stabilisator_2 = CPLbool('x',PL_stabilisator_or,CPL);
        SG_bt_t = SGofCPLz(CPL_following,2);
        SG_stabilisator = SGofCPLz(PL_stabilisator,6);
        SG_stabilisator_2 = SGofCPLz(PL_stabilisator_2,1.5);
        SG_stabilisator = SGcat(SG_stabilisator,SG_stabilisator_2);
end
%% Mid Top&Bottom Layer
e_dir1 = PLshortestDistanceOut(CPL_out,positions(1,:));
angle1 = atan2(abs(diff(e_dir1(:,1))),abs(diff(e_dir1(:,2))));
PL_cut_m_wire = [PLcircseg(h_r,10,0,pi);-h_r*3 -5;h_r*3 -5];
PL_cut_m_wire = PLtransR(PL_cut_m_wire,rot(angle1));
PL_cut_m_wire = PLtrans(PL_cut_m_wire,positions(1,:));
if ~push
    PL_cut_m_wire = [PL_cut_m_wire;NaN NaN;PLtransC(PL_cut_m_wire,[0 0],pi)];
end
if ~end_cap
    e_dir2 = PLshortestDistanceOut(CPL_out,positions(2,:));
    angle2 = atan2(abs(diff(e_dir2(:,1))),abs(diff(e_dir2(:,2))));
    PL_cut_m_bt = [PLcircseg(1.25,10,0,pi);-1.5 -5;1.5 -5];
    PL_cut_m_bt = PLtransR(PL_cut_m_bt,rot(angle2));
    PL_cut_m_bt = PLtrans(PL_cut_m_bt,positions(2,:));
    PL_cut_m_bt = [PL_cut_m_bt;NaN NaN;PLtransC(PL_cut_m_bt,[0 0],pi)];
    CPL_m_bt = CPLbool('-',CPL,PL_cut_m_bt);
    CPL_m_bt = CPLbool('-',CPL_m_bt,PL_cut_m_wire);
    SG_m_bt = SGofCPLz(CPL_m_bt,2.5);
    if side_stabi == 1
        CPL_m_bt =  CPLbool('-',CPL_m_bt,PL_stabi_cut);
        SG_m_bt_b = SGofCPLz(CPL_m_bt,2.5);
    end
    CPL_m = CPLbool('-',CPL,PL_cut_m_wire);
    SG_m = SGofCPLz(CPL_m_bt,5.5);
    if side_stabi == 1
        CPL_m = CPLbool('-',CPL_m_bt,PL_stabi_cut);
        SG_m_b = SGofCPLz(CPL_m,5.5);
    end
else
    %% Middle Endcaplayer
    CPL = CPLbool('-',CPL,PL_cut_m_wire);
    SG_m = SGofCPLz(CPL,10);
end
%%  Combining of Layers
if ~end_cap
    if side_stabi == 1
        SG_m = SGcat(SGunder(SG_m_bt_b,SG_m_b),SG_m_b,SGontop(SG_m_bt,SG_m_b));
    else
        SG_m = SGcat(SGunder(SG_m_bt,SG_m),SG_m,SGontop(SG_m_bt,SG_m));
    end
end
if side_stabi == 1
    SG = SGcat(SGunder(SG_bt_b,SG_m),SG_m,SGontop(SG_bt_t,SG_m));
else
    SG = SGcat(SGunder(SG_bt_b,SG_m),SG_m,SGontop(SG_bt_t,SG_m));
end
height_SG = max(SG.VL(:,3))-min(SG.VL(:,3));
SG = SGtrans(SG,[0 0 (height_SG/2)-max(SG.VL(:,3))]);

%% add hinge
height = 0.5;

SG_hinge = SGhingeround(0.5,hinge_width_b,height);

SG_hinge_b = SGtrans(SG_hinge,[0 axis(1,2) 0]);
SG_hinge_b = SGtransR(SG_hinge_b,rotz(axis(1,1)));
SG_hinge_b = SGcreateHinge(CPL,SG_hinge_b,axis(1,1));
SG_hinge_b = SGmirror(SG_hinge_b,'xy');

if ~end_cap
    SG_hinge_t = SGhingeround(0.5,hinge_width_t,height);
    SG_hinge_t = SGtrans(SG_hinge_t,[0 axis(2,2) 0]);
    SG_hinge_t = SGtransR(SG_hinge_t,rotz(axis(2,1)));
    SG_hinge_t = SGcreateHinge(CPL_following,SG_hinge_t,axis(2,1));
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
H_f = [rotx(90)*roty(90+axis(2,1)) [-axis(2,2);0;((height_SG/2)+height)]; 0 0 0 1];
H_b = [rotx(90)*roty(-90+axis(1,1)) [-axis(1,2);0;(-(height_SG/2)-height)]; 0 0 0 1];

SG = SGTset(SG,'B',H_b);
SG = SGTset(SG,'F',H_f);
end