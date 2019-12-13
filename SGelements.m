%%   [SG]=SGelements(CPL,h_dir,h_offset,h_r)
%	=== INPUT PARAMETERS ===
%	CPL:            CPL of element with tool hole
%	h_dir:          Angle of hinge
%	h_offset:       Hinge offset from middleplane
%	bottom_ele:     1: Is starting Element default: isnt starting Element
%   side_stabi:     1: Create Side Stabilisation. default: No stabi
%   hinge_width:    Override default hinge width of 1.2
%   ele_height:     Override default element height of 2
%	=== OUTPUT RESULTS ======
%	SG:         SG of element
function [SG,CPL] = SGelements(CPL,h_dir,h_offset,varargin)
%% Initializing
height = 0.5;
bottom_ele = 0;     if nargin>=4 && ~isempty(varargin{1});  bottom_ele = varargin{1};   end
side_stabi = 0;     if nargin>=5 && ~isempty(varargin{2});  side_stabi = varargin{2};   end
hinge_width = 1.2;  if nargin>=6 && ~isempty(varargin{3});  hinge_width = varargin{3};  end
ele_height = 2;     if nargin>=7 && ~isempty(varargin{3});  ele_height = varargin{4};   end
%% Creating base and zeroing it in z
if side_stabi == 1
    maxY = max(CPL(:,2));
    maxX = max(CPL(:,1));  
    cut_x = 2.5;
    cut_y = 4;
    
    PL_stabi_cut = PLsquare(cut_x,cut_y);
    PL_stabilisator = PLtrans(PLsquare(cut_x-0.4,cut_y-0.4),[0 0.2]);
        
    PL_stabi_cut = PLtrans(PL_stabi_cut,[maxX-2 maxY-1]);  
    PL_stabilisator = PLtrans(PL_stabilisator,[-maxX+2 maxY-1]);
    
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
SG_hinge = SGtrans(SG_hinge,[0 h_offset 0]);
SG_hinge = SGtransR(SG_hinge,rotz(h_dir));
SG_hinge= SGcreateHinge(CPL,SG_hinge,h_dir);

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

%% Add frames to element
H_f = TofR(rotx(90)*roty(90+h_dir),[-h_offset 0 height+(height_SG/2)]);

% H_f = [rotx(90)*roty(90+h_dir) [0;0;height+(height_SG/2)]; 0 0 0 1];
if ~bottom_ele
    H_b = TofR(rotx(90)*roty(-90+h_dir),[-h_offset 0 -(height_SG/2)-height]);
%     H_b = [rotx(90)*roty(-90+h_dir) [0;0;(-(height_SG/2)-height)]; 0 0 0 1];
else
    H_b = [rotx(90)*roty(180) [0;0;(-(height_SG/2))]; 0 0 0 1];
end

SG = SGTset(SG,'F',H_f);
SG = SGTset(SG,'B',H_b);
SG = SGcolor(SG);
if side_stabi ==1
    SG_2 = SGcolor(SG_2);
    SG_2 = SGTset(SG_2,'F',H_f);
    SG_2 = SGTset(SG_2,'B',H_b);
    SG_2 = SGcolor(SG_2);
    SG = [SG SG_2];
end

end