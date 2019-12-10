%%  [SG] = SGmotormountSM40CL(radius)
%	=== INPUT PARAMETERS ===
%	radius:        radius of rotordisk
%	=== OUTPUT RESULTS ======
%	SG:         SG of Servomount
function [SG] = SGmotormountSM40CL(radius)

servo_d = [46.5 28.5 34];
radius = radius+2;
PL_base_cutout = PLtrans(PLcircle(6),[0 -35.25]); %unterer Nippel
PL_screw_pattern = PLtrans(PLpatternXY(PLcircle(2),2,2,16,12),[0 21]);

CPL_base = [-radius 4;radius 4;radius -servo_d(1);-radius -servo_d(1);NaN NaN;PL_base_cutout];
SG_base = SGofCPLz(CPL_base,4);
CPL_screw_base = [-radius 0;radius 0;radius servo_d(3)+5;-radius servo_d(3)+5;NaN NaN;PL_screw_pattern];
SG_screw_base = SGofCPLz(CPL_screw_base,8);
SG_screw_base = SGtransrelSG(SG_screw_base,SG_base,'rotx',pi/2,'behind',-4);

SG_tensioning = SGcrimptensioner(10,3,20);
[sx,~,~,~,~,~] = sizeVL(SG_tensioning.VL);
SG_tensioning = SGtransrelSG(SG_tensioning,SG_screw_base,'ontop','alignright',sx/2-1,'alignback',24);

SG_tension_holder = SGofCPLz([0 0;radius+0.4*sx 0;radius+0.4*sx 13;sx 13;NaN NaN;PLtrans(PLcircle(0.5),[radius-1 7])],8);
SG_tension_holder = SGtrans(SG_tension_holder,rotx(90));
SG_tension_holder = SGtransrelSG(SG_tension_holder,SG_screw_base,'ontop','alignfront');
SG_tensioning = SGcat(SG_tensioning,SG_tension_holder);
SG_tensioning = SGcat(SG_tensioning,SGmirror(SG_tensioning,'yz'));

SG_brace = SGtrans(SGofCPLz([0 0;10 0;25 15;25 25],6),TofR(rotx(90)*roty(90)));
SG_brace = SGtransrelSG(SG_brace,SG_base,'ontop','alignback','alignleft');
SG_brace = SGcat(SG_brace,SGmirror(SG_brace,'yz'));


% SGplot(SGontop(SGofCPLcommand('c 60,h 3,c 19,h 5,move 0 -11,b 28.5 46.5,h 34,move 0 -23.25'),SG_base));

SG = SGcat(SG_base,SG_screw_base,SG_tension_holder,SG_tensioning,SG_brace);
SGs = SGanalyzeGroupParts(SG);
SGs.SG(2) = SGtransrelSG(SGs.SG(2),SGs.SG(1),'alignbottom',-6,'alignfront','centerx',10);
SGs.SG(3) = SGtransrelSG(SGs.SG(3),SGs.SG(1),'alignbottom',-6,'alignfront','centerx',-10);
SG = SGcat(SGs);
SGwriteSTL(SG,'','y');
end