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
function [SG] = SGconnector(CPL,CPL_holes,positions,section_p,h_r,tool_radius,varargin)
%%
hinge_width_b = 1.2;    if nargin>=7 && ~isempty(varargin{1}); hinge_width_b = varargin{1}; end
hinge_width_t = 1.2;    if nargin>=8 && ~isempty(varargin{2}); hinge_width_t = varargin{2}; end
min_len = [1,1];        if nargin>=9 && ~isempty(varargin{3}); min_len = varargin{3}; end
flags = {};             if nargin>=10 && ~isempty(varargin{4}); flags = varargin{4}; end

cut_orientation = 'y'; single = 0; end_cap = 0; crimp = 1;
% if size(positions,1)>1
%     positions = flip(positions);
% end
for f=1:size(flags,2)
   switch flags{f}
       case 'end_cap'
           end_cap = 1;
       case 'single'
           single = 1;
           crimp = 0;
       case 'x'
           cut_orientation = 'x';
       case 'y'
           cut_orientation = 'y';
       case 'crimp'
           crimp = 1;
   end
end
CPL_b = CPL{1};
if size(CPL,2) == 2
    CPL_f = CPL{2};
else
    CPL_f = CPL{1};
end
CPL_holes_b = CPL_holes{1};
if size(CPL_holes,2) == 2 
    CPL_holes_f = CPL_holes{2}; 
else
    CPL_holes_f = [];
end

%% Shifting all holes to positive x_values
for i=1:size(positions,1)
    if positions(i,1) < 0  && ~single
        positions(i,:) = -positions(i,:);
    end
end

CPL_b = CPLbool('-',CPL_b,CPL_holes_b);
CPL_f_baseholes = CPLbool('-',CPL_f,CPL_holes_b);
CPL_f = CPLbool('-',CPL_f,CPL_holes_f);
PL_wireescape = CPLconvexhull([PLcircle(h_r);NaN NaN;PLtrans(PLsquare(h_r*2),[0 -2*h_r])]);
PL_crimp_hole = CPLconvexhull([PLcircle(h_r*1.5);NaN NaN;PLtrans(PLsquare(h_r*3),[0 -2])]);
angle = atan2(positions(1,1),positions(1,2));  %double angle  pi-angle2 -angle2 Angle between centerpoint and holeposition
[sizey,sizex,~,~,~,~] = sizeVL(CPL_b);
CPL_b_wireescape = CPLbool('-',CPL_b,PLtrans(PLtransR(PL_wireescape,rot(pi-angle)),positions(1,:)));
CPL_f_wireescape = CPLbool('-',CPL_f_baseholes,PLtrans(PLtransR(PL_wireescape,rot(pi-angle)),positions(1,:)));
SG_bottom = SGofCPLz(CPL_b,2);

if crimp
    if size(positions,1) == 2
        angle2 = atan2(positions(2,1),positions(2,2));
        CPL_b_wireescape = CPLbool('-',CPL_b_wireescape,PLtrans(PLtransR(PL_crimp_hole,rot(pi-angle2)),positions(2,:)));
        CPL_b_wireescape = CPLbool('-',CPL_b_wireescape,PLtrans(PLtransR(PL_crimp_hole,rot(-angle2)),-positions(2,:)));
        CPL_f_wireescape = CPLbool('-',CPL_f_wireescape,PLtrans(PLtransR(PL_crimp_hole,rot(pi-angle2)),positions(2,:)));
        CPL_f_wireescape = CPLbool('-',CPL_f_wireescape,PLtrans(PLtransR(PL_crimp_hole,rot(-angle2)),-positions(2,:)));
        
    end
    if ~single
        CPL_b_wireescape = CPLbool('-',CPL_b_wireescape,PLtrans(PLtransR(PL_wireescape,rot(-angle)),-positions(1,:)));
        CPL_f_wireescape = CPLbool('-',CPL_f_wireescape,PLtrans(PLtransR(PL_wireescape,rot(-angle)),-positions(1,:)));
    end
    SG_middle = SGof2CPLsz(CPL_b_wireescape,CPL_f_wireescape,10.5);
    SG_top = SGofCPLz(CPL_f,2);
    SG = SGstack('z',SG_bottom,SG_middle,SG_top);
    
else      
    PL_tool_guard = [PLcircle(tool_radius);NaN NaN;PLcircle(tool_radius+0.5)]; 
    if cut_orientation == 'x'
        width = (sizey/2)-abs(positions(1,1))+(h_r)+5;
        if positions(1,1)>0
            offset = positions(1,1)+width/2-h_r;
        else
            offset = positions(1,1)-width/2+h_r;
        end
        CPL_b_wirechannels = CPLbool('-',CPL_b_wireescape,PLtrans(PLsquare(width,sizex*2),[offset 0]));
    else
        width = (sizex/2)-abs(positions(1,2))+(h_r)+5;
        if positions(1,2)>0
            offset = positions(1,2)+width/2-h_r;
        else
            offset = positions(1,2)-width/2+h_r;
        end
        CPL_b_wirechannels = CPLbool('-',CPL_b_wireescape,PLtrans(PLsquare(sizey*2,width),[0 offset]));
    end
    
    if positions(1,1)>0
        CPL_b_wirechannels = CPLbool('-',CPL_b_wirechannels,PLtrans(PLsquare(sizey*2,0.8),[-sizey -positions(1,2)]));
    else
        CPL_b_wirechannels = CPLbool('-',CPL_b_wirechannels,PLtrans(PLsquare(sizey*2,0.8),[sizey -positions(1,2)]));
    end    
    CPL_b_wirechannels = CPLbool('+',CPL_b_wirechannels,PL_tool_guard);
    
%     if ~single
%         CPL_b_wireescape = CPLbool('-',CPL_b_wireescape,PLtrans(PLtransR(PL_wireescape,rot(-angle)),-positions(1,:)));
%         CPL_f_wireescape = CPLbool('-',CPL_f_wireescape,PLtrans(PLtransR(PL_wireescape,rot(-angle)),-positions(1,:)));
%         CPL_b_wirechannels = CPLbool('-',CPL_b_wirechannels,PLtrans(PLtransR(PL_wireescape,rot(-angle)),-positions(1,:)));
%         if cut_orientation == 'x'
%             CPL_b_wirechannels = CPLbool('-',CPL_b_wirechannels,PLtrans(PLsquare(3,sizey),[-sizex/2 0]));    %% TODO
%         else
%             CPL_b_wirechannels = CPLbool('-',CPL_b_wirechannels,PLtrans(PLsquare(sizex/2,3),[sizex/4 -sizey/2]));    %% TODO
%         end
%         CPL_b_wirechannels = CPLbool('-',CPL_b_wirechannels,PLtrans(PLsquare(sizex/2+2,0.8),[(sizex/2+2)/2 -positions(1,2)]));
%     end    
    
    SG_wire_layer = SGofCPLz(CPL_b_wirechannels,0.6);
    SG_top_connector = SGof2CPLsz(CPL_b_wireescape,CPL_f_wireescape,2);
    SG_top_layer = SGofCPLz(CPLbool('-',CPL_f,CPL_holes_f),1);
    SG = SGstack('z',SG_bottom,SG_wire_layer,SG_top_connector,SG_top_layer);    
end
height_SG = max(SG.VL(:,3))-min(SG.VL(:,3));
SG = SGtrans(SG,[0 0 (height_SG/2)-max(SG.VL(:,3))]);

%% add hinge
height = 0.5;

SG_hinge = SGhingeround(0.5,hinge_width_b,height);
SG_hinge_b = SGtransR(SG_hinge,rotz(section_p(1,1)));
[SG_hinge_b,offset_b] = SGcreateHinge(CPL_b,SG_hinge_b,section_p(1,1),section_p(1,4),hinge_width_b,min_len);
SG_hinge_b = SGmirror(SG_hinge_b,'xy');
offset_t =0;
if ~end_cap
    SG_hinge_t = SGhingeround(0.5,hinge_width_t,height);
    SG_hinge_t = SGtransR(SG_hinge_t,rotz(section_p(2,1)));
    [SG_hinge_t,offset_t] = SGcreateHinge(CPL_f,SG_hinge_t,section_p(2,1),section_p(2,4),hinge_width_t,min_len);
    SG_hinge_b = SGunder(SG_hinge_b,SG);
    SG_hinge_t = SGontop(SG_hinge_t,SG);
    SG = SGcat(SG_hinge_b,SG_hinge_t,SG);
else
    SG = SGcat(SGunder(SG_hinge_b,SG),SG);
end

%% add stops
% 
% SG_stop_b = SGelementstops(CPL_b,section_p(1,1),angles(1,1),angles(2,1),hinge_width_b,offset_b);
% SG_stop_b = SGmirror(SG_stop_b,'xy');
% SG_stop_b = SGtrans(SG_stop_b,[0 0 -height_SG/2]);
% if ~end_cap
%     SG_stop_t = SGelementstops(CPL_f,section_p(2,1),angles(2,1),angles(2,2),hinge_width_t,offset_t);
%     SG_stop_t = SGtrans(SG_stop_t,[0 0 height_SG/2]);    
%     SG_stop_b = SGcat(SG_stop_b,SG_stop_t);
% end
% SG = SGcat(SG,SG_stop_b);
%% Setting Frames
H_f = [rotx(90)*roty(90+section_p(2,1)) [-section_p(2,4);0;((height_SG/2)+height)]; 0 0 0 1];
H_b = [rotx(90)*roty(-90+section_p(1,1)) [-section_p(1,4);0;(-(height_SG/2)-height)]; 0 0 0 1];

SG = SGTset(SG,'B',H_b);
SG = SGTset(SG,'F',H_f);
SG.offset = [offset_b,offset_t];
% SGwriteSTL(SG,rand+"");
end