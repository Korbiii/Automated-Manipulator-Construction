%%  [SG] = SGcrimptensioner(length,width,tension_length)
%	=== INPUT PARAMETERS ===
%	length:         crimp length
%	width:          crimp max width
%	tension length:	dx of tensioning
%   which:          1 = screw 2= Nut 0 = both 3 = nut nut
%	=== OUTPUT RESULTS ======
%	SG:         SG of crimp tensioner
function [SG] = SGcrimptensioner(length,width,tension_length,which)

SG_stationary = SGscrewDIN(10,10+tension_length,'','',0.2);
SG_hole = SGofCPLz(PLcircle(1),10+tension_length);
SG_stationary = SGboolh('-',SG_stationary,SG_hole);

SG_screw = SGscrewDIN(-10,10+tension_length,'',PLcircle(8,6));

SG_crimp = [PLcircle(8,6);NaN NaN;PLcircle(0.7)];
SG_crimp = SGofCPLz(SG_crimp,3);
SG_crimp_2 = SGof2CPLsz([PLcircle(8,6);NaN NaN;PLcircle(width/2)],[PLcircle(width+2);NaN NaN;PLcircle(width/2)],length);
SG_crimp = SGcat(SGontop(SG_crimp_2,SG_crimp),SG_crimp);
SG_screw = SGcat(SGontop(SG_crimp,SG_screw),SG_screw);

if which == 0
    SG_nut = SGscrewDIN(-10,5,'',PLcircle(8,6));
    SG_screw = SGcat(SGunder(SG_nut,SG_screw,2),SG_screw);
SG = SGcat(SGunder(SG_stationary,SG_screw, -20),SG_screw);
elseif which == 1
    SG = SG_screw;
elseif which == 2
    SG = SG_stationary;
elseif which == 3
    SG = SGscrewDIN(-10,5,'',PLcircle(8,6));
end
    SG = SGtrans(SG,TofR(roty(90)*rotx(-90)));

end