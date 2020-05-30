%%   [SG] = SGconnector(CPL,CPL_holes,positions,angle_p,length_p,h_r,tool_radius,flags)
%	=== INPUT PARAMETERS ===
%	CPL:                CPL of sections to connect
%   CPL_holes:          CPL of holes of sections to connect
%	positions:          2xn vector of hole positions
%	angle_p:            2x2 Vector cotaining orientation of sections to connect [xy-Angle, optimization]
%	length_p:           3x2 Vector containing hinge parameters of sections to connect [Hinge_width, Mininum length of hinge, hinge_height]
%   h_r:                Radius of Bowdencable
%   tool_radius:        Radius of tool
%   === FLAGS ==============              
%   'end_cap':          Creates Connector with only bottom hinge;
%   'single':           Create Connector with only one Bowdencable per section; 
%   'crimp':            Creates slots for crimp connections; 
%   'x'/'y':            Orientation of slots for non crimp connection;   
%	=== OUTPUT RESULTS =====
%	SG:         SG of connector element
function [SG] = SGconnector(CPL,CPL_holes,positions,angle_p,length_p,h_r,tool_radius,varargin)
flags = {}; if nargin>=8 && ~isempty(varargin{1}); flags = varargin{1}; end
cut_orientation = 'y'; single = 0; end_cap = 0; crimp = 1;

for f=1:size(flags,2)
   switch flags{f}
       case 'end_cap'
           end_cap = 1;
           length_p(2,3) = 0;
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
angles = {};
CPL_b = CPL{1};
if size(CPL,1) == 2
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
if ~single
    for k = 1:size(positions,2)
        positions{k}(positions{k}(:,1)<0,:) = -positions{k}(positions{k}(:,1)<0,:);
    end
end

CPL_b = CPLbool('-',CPL_b,CPL_holes_b);
CPL_f_baseholes = CPLbool('-',CPL_f,CPL_holes_b);
CPL_f = CPLbool('-',CPL_f,CPL_holes_f);
PL_wireescape = CPLconvexhull([PLcircle(h_r*1.25);NaN NaN;PLtrans(PLsquare(h_r*2),[0 -10*h_r])]);
PL_crimp_hole = CPLconvexhull([PLcircle(h_r*1.5);NaN NaN;PLtrans(PLsquare(h_r*3),[0 -10*h_r])]);
[sizey,sizex,~,~,~,~] = sizeVL(CPL_b);
SG_bottom = SGofCPLz(CPL_b,2);
CPL_b_wireescape = CPL_b;
CPL_f_wireescape = CPL_f;
for k=1:size(positions{1},1)
    angles{end+1} = atan2(positions{1}(k,1),positions{1}(k,2));  %double angle  pi-angle2 -angle2 Angle between centerpoint and holeposition
    CPL_b_wireescape = CPLbool('-',CPL_b_wireescape,PLtrans(PLtransR(PL_wireescape,rot(pi-angles{k})),positions{1}(k,:)));
    CPL_f_wireescape = CPLbool('-',CPL_f_wireescape,PLtrans(PLtransR(PL_wireescape,rot(pi-angles{k})),positions{1}(k,:)));
end


if crimp   
    if size(positions,2) == 2
        for k=1:size(positions{2},1)
            angle2 = atan2(positions{2}(k,1),positions{2}(k,2));
            CPL_b_wireescape = CPLbool('-',CPL_b_wireescape,PLtrans(PLtransR(PL_crimp_hole,rot(pi-angle2)),positions{2}(k,:)));
            CPL_b_wireescape = CPLbool('-',CPL_b_wireescape,PLtrans(PLtransR(PL_crimp_hole,rot(-angle2)),-positions{2}(k,:)));
            CPL_f_wireescape = CPLbool('-',CPL_f_wireescape,PLtrans(PLtransR(PL_crimp_hole,rot(pi-angle2)),positions{2}(k,:)));
            CPL_f_wireescape = CPLbool('-',CPL_f_wireescape,PLtrans(PLtransR(PL_crimp_hole,rot(-angle2)),-positions{2}(k,:)));
        end
    end
    
    if ~single
        for k=1:size(positions{1},1)
            CPL_b_wireescape = CPLbool('-',CPL_b_wireescape,PLtrans(PLtransR(PL_wireescape,rot(-angles{k})),-positions{1}(k,:)));
            CPL_f_wireescape = CPLbool('-',CPL_f_wireescape,PLtrans(PLtransR(PL_wireescape,rot(-angles{k})),-positions{1}(k,:)));
        end
    end
  
    SG_middle = SGof2CPLsz(CPL_b_wireescape,CPL_f_wireescape,10.5,'','miny');
    SG_top = SGofCPLz(CPL_f,2);
    SG = SGstack('z',SG_bottom,SG_middle,SG_top);
    
else      
    PL_tool_guard = [PLcircle(tool_radius);NaN NaN;PLcircle(tool_radius+0.5)]; 
    if cut_orientation == 'x'
        width = (sizey/2)-abs(positions(1,1))+(h_r)+5;
        if positions{1}(1)>0
            offset = positions{1}(1)+width/2-h_r;
        else
            offset = positions{1}(1)-width/2+h_r;
        end
        CPL_b_wirechannels = CPLbool('-',CPL_b_wireescape,PLtrans(PLsquare(width,sizex*2),[offset 0]));
        if angle_p(1,2) == 2
            CPL_b_wirechannels = CPLbool('-',CPL_b_wirechannels,PLtrans(PLsquare(width,sizex*2),[-offset 0]));
        end
    else
        width = (sizex/2)-abs(positions{1}(2))+(h_r)+5;
        if positions{1}(2)>0
            offset = positions{1}(2)+width/2-h_r;
        else
            offset = positions{1}(2)-width/2+h_r;
        end
        CPL_b_wirechannels = CPLbool('-',CPL_b_wireescape,PLtrans(PLsquare(sizey*2,width),[0 offset]));
         if angle_p(1,2) == 2
            CPL_b_wirechannels = CPLbool('-',CPL_b_wirechannels,PLtrans(PLsquare(sizey*2,width),[0 -offset]));
        end
    end
    
    if positions{1}(1)>0
        CPL_b_wirechannels = CPLbool('-',CPL_b_wirechannels,PLtrans(PLsquare(sizey*2,0.8),[-sizey -positions{1}(2)]));
    else
        CPL_b_wirechannels = CPLbool('-',CPL_b_wirechannels,PLtrans(PLsquare(sizey*2,0.8),[sizey -positions{1}(2)]));
    end    
    CPL_b_wirechannels = CPLbool('+',CPL_b_wirechannels,PL_tool_guard);
    SG_wire_layer = SGofCPLz(CPL_b_wirechannels,0.6);
    SG_top_connector = SGof2CPLsz(CPL_b_wireescape,CPL_f_wireescape,2,'','miny');
    SG_top_layer = SGofCPLz(CPLbool('-',CPL_f,CPL_holes_f),1);
    SG = SGstack('z',SG_bottom,SG_wire_layer,SG_top_connector,SG_top_layer);    
end
height_SG = max(SG.VL(:,3))-min(SG.VL(:,3));
SG = SGtrans(SG,[0 0 (height_SG/2)-max(SG.VL(:,3))]);

%% add hinge

SG_hinge = SGhingeround(length_p(1,2),length_p(1,1),length_p(1,3));
SG_hinge_b = SGtransR(SG_hinge,rotz(angle_p(1,1)));
[SG_hinge_b,offset_b] = SGcreateHinge(CPL_b,SG_hinge_b,angle_p(1,1),angle_p(1,2),length_p(1,1),length_p(1,2),length_p(1,3));
SG_hinge_b = SGmirror(SG_hinge_b,'xy');
offset_t =0;
if ~end_cap
    SG_hinge_t = SGhingeround(length_p(1,2),length_p(2,1),length_p(2,3));
    SG_hinge_t = SGtransR(SG_hinge_t,rotz(angle_p(2,1)));
    [SG_hinge_t,offset_t] = SGcreateHinge(CPL_f,SG_hinge_t,angle_p(2,1),angle_p(2,2),length_p(2,1),length_p(1,2),length_p(2,3));
    SG_hinge_b = SGunder(SG_hinge_b,SG);
    SG_hinge_t = SGontop(SG_hinge_t,SG);
    SG = SGcat(SG_hinge_b,SG_hinge_t,SG);
else
    SG = SGcat(SGunder(SG_hinge_b,SG),SG);
end

%% Setting Frames
e_dir_f = [sind(angle_p(2,1)) -cosd(angle_p(2,1))];
e_dir_b = [sind(angle_p(1,1)) -cosd(angle_p(1,1))];

H_f = [rotx(90)*roty(90+angle_p(2,1)) [offset_t*-e_dir_f';((height_SG/2)+length_p(2,3))]; 0 0 0 1];
H_b = [rotx(90)*roty(-90+angle_p(1,1)) [offset_b*-e_dir_b';(-(height_SG/2)-length_p(1,3))]; 0 0 0 1];

SG = SGTset(SG,'B',H_b);
SG = SGTset(SG,'F',H_f);
SG.offset = [offset_b,offset_t];
end