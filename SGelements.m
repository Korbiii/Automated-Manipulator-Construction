%%  [SG] = SGelements(CPL,parameters,[hinge_width,ele_height])
%	=== INPUT PARAMETERS ===
%	CPL:            CPL of element with tool hole
%   parameters:     nx4
%   hinge_width:    Override default hinge width of 1.2mm
%   ele_height:     Override default element height of 2mm
%   flags:          'bottom_element';'side_stabi'
%	=== OUTPUT RESULTS ======
%	SG:             SG of element
function [SG,offset] = SGelements(CPL,angle_p,varargin)
%% Initializing
 bottom_ele = 0;side_stabi = 0; 
hinge_width = 1.2;  if nargin>=3 && ~isempty(varargin{1});  hinge_width = varargin{1};  end
ele_height = 2;     if nargin>=4 && ~isempty(varargin{2});  ele_height = varargin{2};   end
min_len = 1;        if nargin>=5 && ~isempty(varargin{3});  min_len = varargin{3}; end
height = 0.5;       if nargin>=6 && ~isempty(varargin{4});  height = varargin{4}; end

h_dir = angle_p(1);
h_opti = angle_p(2);


for f=5:size(varargin,2)
   switch varargin{f}
       case 'bottom_element'
           bottom_ele = 1;        
   end
end



%% Creating base and zeroing it in z

%     SG = SGofCPLz(CPL,ele_height);
SG = SGofCPLzdelaunayGrid(CPL,ele_height,0.5,0.5);
height_SG = abs(max(SG.VL(:,3))-min(SG.VL(:,3)));
SG = SGtrans(SG,[0 0 -height_SG/2]);


%% Add hinge to base
SG_hinge = SGhingeround(min_len,hinge_width,height);
SG_hinge = SGtransR(SG_hinge,rotz(h_dir));
SG_hinge_2 = SGtransR(SG_hinge,rotz(-90));
[SG_hinge,offset]= SGcreateHinge(CPL,SG_hinge,h_dir,h_opti,hinge_width,min_len,height);
SG_hinge_top = SGontop(SG_hinge,SG);
SG_hinge_bottom = SGmirror(SG_hinge_top,'xy');

if h_opti == 2
    [SG_hinge_2,offset_2]= SGcreateHinge(CPL,SG_hinge_2,h_dir-90,h_opti,hinge_width,min_len,height);
    SG_hinge_2_top = SGontop(SG_hinge_2,SG);
    SG_hinge_2_bottom = SGmirror(SG_hinge_2_top,'xy');
end


if ~bottom_ele    
    if h_opti == 2
        SG_base_ = SG;
        SG = SGcat(SG_base_,SG_hinge_bottom,SG_hinge_2_top);
        SG_2 = SGcat(SG_base_,SG_hinge_2_bottom,SG_hinge_top);
        
    else
        SG = SGcat(SG,SG_hinge_top,SG_hinge_bottom);
    end    
else
    SG = SGcat(SG,SG_hinge_top);
end

%% Add frames to element
H_f = TofR(rotx(90)*roty(90+h_dir),[offset 0 height+(height_SG/2)]);
H_f_2 = TofR(rotx(90)*roty(180+h_dir),[offset 0 height+(height_SG/2)]);
if ~bottom_ele
    H_b = TofR(rotx(90)*roty(-90+h_dir),[offset 0 -(height_SG/2)-height]);
    H_b_2 = TofR(rotx(90)*roty(h_dir),[offset 0 -(height_SG/2)-height]);
else
    H_b = [rotx(90)*roty(180) [0;0;(-(height_SG/2))]; 0 0 0 1];
end
if h_opti == 2    
    SG = SGTset(SG,'F',H_f_2);
    SG = SGTset(SG,'B',H_b);
    SG_2 = SGTset(SG_2,'F',H_f);
    SG_2 = SGTset(SG_2,'B',H_b_2);
    SG.offest = offset;
    SG_2.offset = offset_2;
    SG = {SG SG_2};
else
    SG = SGTset(SG,'F',H_f);
    SG = SGTset(SG,'B',H_b);
    SG.offest = offset;
    SG = SG;
end


end