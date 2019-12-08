%%  [SG] = SGcrimpholder(crimp_length)
%	=== INPUT PARAMETERS ===
%	crimp_length:   length of crimp
%	=== OUTPUT RESULTS ======
%	SG:         SG crimp_holder
function [SG] = SGcrimpholder(crimp_length)

PL_front = [-2 2;4 2;4 -2;-2 -2;NaN NaN;PLcircle(0.4)];
SG_front = SGofCPLz(PL_front,2);
PL_middle = [-2 2;4 2;4 -2;-2 -2;-2 -1;1 -1;1 1;-2 1];
SG_middle  = SGofCPLz(PL_middle,crimp_length);
SG_crimp_holder = SGcat(SG_front,(SGunder(SG_middle,SG_front)));
PL_back = [-2 2;4 2;4 -2;-2 -2;-2 -0.3;1 -0.5;1 0.5;-2 0.3];
SG_back  = SGofCPLz(PL_back,5);
SG = SGtrans(SGcat(SG_crimp_holder,(SGunder(SG_back,SG_crimp_holder))),TofR(rotx(90)));

end
