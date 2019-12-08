%%   [SG] = SGconnector(CPL,positions,axis,h_r,push_rod)
%	=== INPUT PARAMETERS ===
%	CPL:        CPL for connector
%	positions:  2xn vector of hole positions
%	axis:   	2xn vector of axis
%	h_r:        radius of hole for actuator rope
%   end_cap:    Set 1 to generate a end cap
%	=== OUTPUT RESULTS ======
%	SG:         SG of connector element
function [SG] = SGconnector(CPL,CPL_out,positions,axis,h_r,varargin)
%%
end_cap = 0; if nargin>=6 && ~isempty(varargin{1}); end_cap = varargin{1}; end
hinge_width_b = 1.2; if nargin>=7 && ~isempty(varargin{2}); hinge_width_b = varargin{2}; end
hinge_width_t = 1.2; if nargin>=8 && ~isempty(varargin{3}); hinge_width_t = varargin{3}; end
push = 0; if nargin>=9 && ~isempty(varargin{4}); push = varargin{4}; end
%% Shifting all holes to positive x_values
for i=1:size(positions,1)
    if positions(i,1) < 0
        positions(i,:) = -positions(i,:);
    end
end
%% Top&Bottom Layer
e_dir1 = PLshortestDistanceOut(CPL_out,positions(1,:));
angle1 = atan2(abs(diff(e_dir1(:,1))),abs(diff(e_dir1(:,2))));
PL_cut_bt = [PLcircseg(h_r,10,0,pi);-h_r*3 -5;h_r*3 -5];
PL_cut_bt = PLtransR(PL_cut_bt,rot(angle1));
PL_cut_bt = PLtrans(PL_cut_bt,positions(1,:));
if push == 0
    PL_cut_bt = [PL_cut_bt;NaN NaN;PLtransC(PL_cut_bt,[0 0],pi)];
end
CPL_bt = CPLbool('-',CPL,PL_cut_bt);
SG_bt = SGofCPLz(CPL_bt,2);
if ~end_cap
    %% Mid Top&Bottom Layer
    e_dir2 = PLshortestDistanceOut(CPL_out,positions(2,:));
    angle2 = atan2(abs(diff(e_dir2(:,1))),abs(diff(e_dir2(:,2))));
    PL_cut_m_bt = [PLcircseg(1.25,10,0,pi);-1.5 -5;1.5 -5];
    PL_cut_m_bt = PLtransR(PL_cut_m_bt,rot(angle2));
    PL_cut_m_bt = PLtrans(PL_cut_m_bt,positions(2,:));
    PL_cut_m_bt = [PL_cut_m_bt;NaN NaN;PLtransC(PL_cut_m_bt,[0 0],pi)];
    CPL_m_bt = CPLbool('-',CPL,PL_cut_m_bt);
    SG_m_bt = SGofCPLz(CPL_m_bt,2.5);
    
    %% Mid mid Layer
    PL_cut_m = [-1.25 8;1.25 8;1.25 -8;-1.25 -8];
    PL_cut_m = PLtransR(PL_cut_m,rot(angle2));
    PL_cut_m = PLtrans(PL_cut_m,positions(2,:));
    PL_cut_m = [PL_cut_m;NaN NaN;PLtransC(PL_cut_m,[0 0],pi)];
    CPL_m = CPLbool('-',CPL,PL_cut_m);
    SG_m = SGofCPLz(CPL_m,5.5);
else
    %% Middle Endcaplayer
    SG_m = SGofCPLz(CPL,4);
end
%%  Combining of Layers
if ~end_cap
    SG_m = SGcat(SGunder(SG_m_bt,SG_m),SG_m,SGontop(SG_m_bt,SG_m));
end
SG = SGcat(SGunder(SG_bt,SG_m),SG_m,SGontop(SG_bt,SG_m));
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
    SG_hinge_t = SGcreateHinge(CPL,SG_hinge_t,axis(2,1));
    SG = SGcat(SGunder(SG_hinge_b,SG),SG,SGontop(SG_hinge_t,SG));
else
    SG = SGcat(SGunder(SG_hinge_b,SG),SG);
end

%% Setting Frames
H_f = [rotx(90)*roty(90+axis(2,1)) [-axis(2,2);0;((height_SG/2)+height)]; 0 0 0 1];
H_b = [rotx(90)*roty(-90+axis(1,1)) [-axis(1,2);0;(-(height_SG/2)-height)]; 0 0 0 1];

SG = SGTset(SG,'B',H_b);
SG = SGTset(SG,'F',H_f);
end