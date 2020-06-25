%%   [SG] = SGflatbrace(x,y,[z])
%	=== INPUT PARAMETERS ===
%   x:  dimension in x 
%   y:  dimension in y
%   z:  optional z dimension: default = 10;
%	=== OUTPUT RESULTS ======
%	SG: Flat brace with x shape
function [SG] =  SGflatbrace(x,y,varargin)
z=5;       if nargin>=3 && ~isempty(varargin{1}); z=varargin{1}; end
PL_frame = [PLsquare(x,y);NaN NaN;PLsquare(x-2*z,y-2*z)];
x_d = sqrt(x^2+y^2);
phi = acos(x/x_d);
PL_diagonal = PLtransR(PLsquare(x_d-(2*z),z),rot(phi));
PL_diagonal = CPLbool('+',PL_diagonal,PLmirror0(PL_diagonal,'x',0));
PL = CPLbool('+',PL_frame,PL_diagonal);
SG = SGofCPLz(PL,z);
end