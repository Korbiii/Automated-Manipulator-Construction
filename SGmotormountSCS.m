%%  [SG] = SGmotormountSCS(radius,CPL_servo)
%	=== INPUT PARAMETERS ===
%	radius:        radius of rotordisk
%   CPL_servo:     CPL of Servomount
%	=== OUTPUT RESULTS ======
%	SG:         SG of Servomount
function [SG] = SGmotormountSCS(radius,CPL_servo)

maxY = max(CPL_servo(:,2));
CPL_base = [-radius-2 -maxY-5;-radius-2 maxY+30;radius+2 maxY+30;radius+2 -maxY-5;NaN NaN; CPL_servo];
SG_base = SGofCPLz(CPL_base,5);

PL_front = [-2 2;4 2;4 -2;-2 -2;NaN NaN;PLcircle(0.4)];
SG_front = SGofCPLz(PL_front,2);
PL_middle = [-2 2;4 2;4 -2;-2 -2;-2 -1;1 -1;1 1;-2 1];
SG_middle  = SGofCPLz(PL_middle,12);
SG_crimp_holder = SGcat(SG_front,(SGunder(SG_middle,SG_front)));
PL_back = [-2 2;4 2;4 -2;-2 -2;-2 -0.3;1 -0.5;1 0.5;-2 0.3];
SG_back  = SGofCPLz(PL_back,5);
SG_crimp_holder = SGtrans(SGcat(SG_crimp_holder,(SGunder(SG_back,SG_crimp_holder))),TofR(rotx(90)));
SG_crimp_holder = SGtransrelSG(SG_crimp_holder,SG_base,'alignleft','alignbehind','ontop');
SG_crimp_holder = SGstackn(SG_crimp_holder,3,0);
[~, y, z] = sizeVL(SG_crimp_holder.VL);
PL_brace = [0 z;0 0;10 0];
SG_brace = SGtrans(SGofCPLz(PL_brace,y),TofR(rotx(90)));
SG_brace = SGtransrelSG(SG_brace,SG_crimp_holder,'alignbehind','right','alignbottom');
SG_crimp_holder = SGcat(SG_crimp_holder,SG_brace);
SG_crimp_holder = SGcat(SGmirror(SG_crimp_holder,'yz'),SG_crimp_holder);

[~,y,~] = sizeVL(SG_base);
SG_guide = SGtrans(SGofCPLz(PL_back,y),TofR(rotx(90)));
SG_guide = SGstackn(SG_guide,2,0);
SG_guide = SGtransrelSG(SG_guide,SG_base,'alignleft','under','alignbehind');

[~,y,z] = sizeVL(SG_guide);
PL_guide_brace = [0 -z;0 0;10 0];
SG_guide_brace =  SGtrans(SGofCPLz(PL_guide_brace,y),TofR(rotx(90)));
SG_guide_brace = SGtransrelSG(SG_guide_brace,SG_guide,'alignbehind','right','aligntop');
SG_guide = SGcat(SG_guide_brace,SG_guide);
SG_guide = SGcat(SG_guide,SGmirror(SG_guide,'yz'));

SG = SGcat(SG_base,SG_crimp_holder,SG_guide);
end