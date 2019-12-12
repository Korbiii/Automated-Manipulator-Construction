%%   [CPL] = PLpatternXY(PL,x_num,y_num,x_dis,y_dis)
%	=== INPUT PARAMETERS ===
%	PL:     Contour of PL you want to pattern
%	x_num:	Number of copies in x-direction
%	y_num:	Number of copies in y-direction
%   x_dis:  Distance between copies in x-direction
%   y_dis:  Distance between copies in y-direction
%	=== OUTPUT RESULTS ======
%   CPL: 	CPL of copied elements
function [SG] = SGpatternXY(SG_element,x_num,y_num,x_dis,y_dis,varargin)
x_z_value=0;       if nargin>=6 && ~isempty(varargin{1}); x_z_value=varargin{1}; end
y_z_value=0;       if nargin>=7 && ~isempty(varargin{2}); y_z_value=varargin{2}; end

SG = SG_element;
for i=1:x_num-1
    SG = SGcat(SG,SGtrans(SG_element,[x_dis*i 0 x_z_value*i]));
end
SG_x = SG;
for i=1:y_num-1    
    SG = SGcat(SG,SGtrans(SG_x,[0 y_dis*i y_z_value*i]));
end
SG = SGtrans0(SG);
end
