%%  [SG] = SGrealhinge(length, diameter)
%	=== INPUT PARAMETERS ===

%	=== OUTPUT RESULTS ======
function [SG] = SGrealhinge(length, diameter,distance)

SG_mid = SGofCPLx(PLcircle(diameter/2),(length/2)-3);
SG_mid_end = SGofCPLx(PLcircle((diameter/2)+3),3);
PL_rotator = [PLcircle((diameter/2)+3);NaN NaN;PLcircle((diameter/2)+0.3)];
PL_rotator_2 = CPLbool('-',PL_rotator,PLsquare(diameter/2,50));
PL_rotator_2 = CPLbool('-',PL_rotator_2,PLsquare(50,diameter/2));
SG_rotator_mid = SGofCPLx(PL_rotator_2,(length/2)-8.5);
SG_rotator_ends = SGofCPLx(PL_rotator,3);
SG_rotator_end_mid = SGofCPLx(PL_rotator,1.5);
SG_rotator = SGstack('x',SG_rotator_end_mid,SG_rotator_mid,SG_rotator_ends);

[sizex_1,~,~,~,~,~] = sizeVL(SG_rotator.VL);
SG_rotator = SGtrans(SGtrans0(SG_rotator),[-sizex_1/2 0 0]);

SG = SGstack('x',SG_mid,SG_mid_end);
[sizex,~,~,~,~,~] = sizeVL(SG.VL);

SG = SGtrans(SGtrans0(SG),[-sizex/2 0 0]);
SG = SGcat(SG_rotator,SG);
PL_brace_stationary = [-(diameter/2)-3 0;distance 0;distance -diameter*2];
SG_brace_stationary = SGofCPLx(PL_brace_stationary,3);

SG_brace_stationary = SGtransrelSG(SG_brace_stationary,SG,'alignleft');
SG = SGcat(SG,SG_brace_stationary);

SG = SGcat(SG,SGmirror(SG,'yz'));

end