%%  [SG]=SGelements(CPL,h_dir,h_offset,[bottom_ele,side_stabi,hinge_width,ele_height])
%	=== INPUT PARAMETERS ===
%	CPL:            CPL of element with tool hole
%	h_dir:          Angle of hinge
%	h_opti:       Hinge offset from middleplane
%	bottom_ele:     1: Is starting Element default: isnt starting Element
%   side_stabi:     1: Create Side Stabilisation. default: No stabi
%   hinge_width:    Override default hinge width of 1.2mm
%   ele_height:     Override default element height of 2mm
%	=== OUTPUT RESULTS ======
%	SG:             SG of element
function [SG,CPL] = SGelements(CPL,h_dir,varargin)
%% Initializing
height = 0.5; bottom_ele = 0;side_stabi = 0; 
h_opti = 0;             if nargin>=3 && ~isempty(varargin{1});  h_opti = varargin{1};  end
hinge_width = 1.2;  if nargin>=4 && ~isempty(varargin{2});  hinge_width = varargin{2};  end
ele_height = 2;     if nargin>=5 && ~isempty(varargin{3});  ele_height = varargin{3};   end

for f=3:size(varargin,2)
   switch varargin{f}
       case 'bottom_element'
           bottom_ele = 1;
       case 'side_stabi'
           side_stabi = 1;           
   end
end



%% Creating base and zeroing it in z
if side_stabi == 1
    maxY = max(CPL(:,2));
    maxX = max(CPL(:,1));  
    cut_x = 2.5;
    cut_y = 4;
    
    PL_stabi_cut = PLsquare(cut_x,cut_y);
    PL_stabilisator = PLtrans(PLsquare(cut_x-0.4,cut_y-0.4),[0 0.2]);
        
    PL_stabi_cut = PLtrans(PL_stabi_cut,[maxX-1.5 maxY-1]);  
    PL_stabilisator = PLtrans(PL_stabilisator,[-maxX+1.5 maxY-1]);
    
    PL_stabi_cut = CPLbool('+',PL_stabi_cut,VLswapY(VLswapX(PL_stabi_cut)));    
    PL_stabilisator =  CPLbool('+',PL_stabilisator,VLswapY(VLswapX(PL_stabilisator)));
    
    % Need two elements for this setup
    PL_stabi_stabi_1 = CPLbool('x',PL_stabilisator,CPL);
    PL_stabi_stabi_2 = CPLbool('x',VLswapY(PL_stabilisator),CPL);      
    
    CPL_1 = CPLbool('-',CPL,PL_stabi_cut);
    CPL_2 = CPLbool('-',CPL,VLswapY(PL_stabi_cut));    
    
    SG_stabi_1 = SGtrans(SGofCPLz(PL_stabi_stabi_1,6),[0 0 1]);
    SG_stabi_2 = SGtrans(SGofCPLz(PL_stabi_stabi_2,6),[0 0 1]); 
    
    SG = SGofCPLz(CPL_1,ele_height/2);
    CPL_1 = CPLbool('-',CPL_1,VLswapX(PL_stabi_cut)); 
    SG = SGcat(SG,SGunder(SGofCPLz(CPL_1,ele_height/2),SG));  
        
    SG_2 = SGofCPLz(CPL_2,ele_height/2);    
    CPL_2 = CPLbool('-',CPL_2,PL_stabi_cut); 
    SG_2 = SGcat(SG_2,SGunder(SGofCPLz(CPL_2,ele_height/2),SG_2));  
    
    height_SG = abs(max(SG.VL(:,3))-min(SG.VL(:,3)));
else    
    SG = SGofCPLz(CPL,ele_height);
    height_SG = abs(max(SG.VL(:,3))-min(SG.VL(:,3)));
    SG = SGtrans(SG,[0 0 -height_SG/2]);
end



%% Add hinge to base
SG_hinge = SGhingeround(0.5,hinge_width,height);
SG_hinge = SGtransR(SG_hinge,rotz(h_dir));
[SG_hinge,offset]= SGcreateHinge(CPL,SG_hinge,h_dir,h_opti,hinge_width);

SG_hinge_top = SGontop(SG_hinge,SG);
SG_hinge_bottom = SGmirror(SG_hinge_top,'xy');

if ~bottom_ele
    if side_stabi == 1        
        SG = SGcat(SG,SG_hinge_top,SG_hinge_bottom,SG_stabi_1);
        SG_2 = SGcat(SG_2,SG_hinge_top,SG_hinge_bottom,SG_stabi_2);
        
        SG_stabi_g_1 = SGtrans(SGgrow(SG_stabi_1,0.2),TofR(rotz(2),[0 0 -4]));
        SG_stabi_g_2 = SGtrans(SGgrow(SG_stabi_2,0.2),[0 0 -4]);        
    else
        SG = SGcat(SG,SG_hinge_top,SG_hinge_bottom);
    end
else
    SG = SGcat(SG,SG_hinge_top);
end

%% Add stops
max_dim = max(sizeVL(CPL))+1;
middle_axis = PLtransR(PLtrans([-max_dim 0;max_dim 0],[0 offset]),rot(deg2rad(h_dir)));
e_dir = (middle_axis/norm(middle_axis))*rot(pi/2);
e_dir = (e_dir(1,:)-e_dir(2,:))/norm(e_dir(1,:)-e_dir(2,:));
left_plane = [flip(middle_axis);PLtrans(middle_axis,e_dir*max_dim)]; % Plane for finding points in positive area
right_plane = [flip(middle_axis);PLtrans(middle_axis,e_dir*-max_dim)];

CPL_out =  CPLselectinout(CPL,0);
CPL_out_left = CPLbool('-',CPL_out,left_plane);
CPL_out_right = CPLbool('-',CPL_out,right_plane);

max_distance_left = 0;
for k=1:size(CPL_out_left,1)
    temp_dis = distPointLine(middle_axis,CPL_out_left(k,:));
    if temp_dis>max_distance_left
        max_distance_left = temp_dis;
    end
end
max_distance_right = 0;
for k=1:size(CPL_out_right,1)
     temp_dis = distPointLine(middle_axis,CPL_out_right(k,:));
    if temp_dis>max_distance_right
        max_distance_right = temp_dis;
    end
end

PLsquare_left = PLsquare(max(hinge_width+1,max_distance_left-2),max_dim*2);
PLsquare_right = PLsquare(max(hinge_width+1,max_distance_right-2),max_dim*2);
PLsquare_left = PLtrans(PLsquare_left,[-offset+(max(hinge_width+1,max_distance_left-2)/2) 0]);
PLsquare_right= PLtrans(PLsquare_right,[-offset-(max(hinge_width+1,max_distance_right-2)/2) 0]);

PL_cut = CPLbool('+',PLsquare_left,PLsquare_right);
CPL_cut = CPLbool('-',CPL,PL_cut);

offset_p = floor((max_distance_right/(max_distance_right+max_distance_left))*500);

PLcontour = [(linspace(-max_distance_right,max_distance_left,500))-offset;linspace(0.5,1,offset_p) linspace(1,0,500-offset_p)]';

SG_stop = SGofCPLz(CPL_cut,0.1);
n=size(SG_stop.VL,1);
PLup=[SG_stop.VL(n/2+1:end,1) SG_stop.VL(n/2+1:end,2)];
VLprojection = PLtoVLprojection(PLup, PLcontour);
SG_stop.VL = [SG_stop.VL(1:n/2,1) SG_stop.VL(1:n/2,2) SG_stop.VL(1:n/2,3);VLprojection];
SG_stop = SGtrans(SG_stop,[0 0 height_SG/2]);

SG = SGcat(SG,SG_stop);
SG = SGcat(SG,SGmirror(SG_stop,'xy'));

%% Add frames to element
H_f = TofR(rotx(90)*roty(90+h_dir),[-h_opti 0 height+(height_SG/2)]);

if ~bottom_ele
    H_b = TofR(rotx(90)*roty(-90+h_dir),[-h_opti 0 -(height_SG/2)-height]);
else
    H_b = [rotx(90)*roty(180) [0;0;(-(height_SG/2))]; 0 0 0 1];
end

SG = SGTset(SG,'F',H_f);
SG = SGTset(SG,'B',H_b);
if side_stabi ==1
    SG_2 = SGcolor(SG_2);
    SG_2 = SGTset(SG_2,'F',H_f);
    SG_2 = SGTset(SG_2,'B',H_b);
    SG = [SG SG_2];
end

end