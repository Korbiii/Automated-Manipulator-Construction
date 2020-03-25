%%   [SG]=SGhingeround(length, width, h_height)
%	=== INPUT PARAMETERS ===
%	length:		Hinge length
%	width:		Middle width of hinge
%	h_height:	Height of Hinge
%	=== OUTPUT RESULTS ======
%	SG:         SG of hinge with round contour
function [SG] = SGhingeround(length, width, h_height)
PL_hinge_top = PLcircseg(width/2,floor(width*10),pi,0);
PL_hinge_base = PLtrans(PLcircseg(h_height,floor(h_height*20),0,-pi/2),[-h_height-width/2 0]);
CPL = [PL_hinge_top;VLswapX(PL_hinge_base);flip(PL_hinge_base)];
CPL = CPLaddauxpoints(CPL,0.125);

SG = SGofCPLzdelaunayGrid(CPL,length,'',0.125);
SG = SGtrans(SG,[0;h_height;-length/2]);
SG = SGtransR(SG,rot(pi/2,pi/2,0));
end