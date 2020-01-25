%%  [SG] = SGmotormountSM85BL(radius)
%	=== INPUT PARAMETERS ===
%	radius:        radius of rotordisk
%	=== OUTPUT RESULTS ======
%	SG:         SG of Servomount
function [SG] = SGmotormountSM85BL(radius)
%% Dimensions of Servo
servo_d = [62 34 47];

PL_screw_holes = PLpatternXY(PLcircle(1.5),1,2,28,28);
CPL_top =  [PLsquare(12,34);NaN NaN;PLtrans(PL_screw_holes,[3 0])];
SG_screw_plate = SGofCPLz(CPL_top,4);

PL_brace_top = PLsquare(10,34);
PL_brace_bottom = PLtrans(PLsquare(5,34),[-2.5 0]);
SG_brace = SGof2CPLsz(PL_brace_bottom,PL_brace_top,20);
SG_brace = SGtransrelSG(SG_brace,SG_screw_plate,'aligntop','right','centery');

SG_tensioning = SGtrans(SGcrimptensioner(10,3,20,2),TofR(rotz(-90),[0 radius 9]));
SG_tensioning = SGcat(SG_tensioning,SGtrans(SG_tensioning,[0 -2*radius 0]));
SG_tensioning = SGtransrelSG(SG_tensioning,SG_brace,'alignright',20,'right');

PL_thread_brace = PLroundcorners2([0 0;10 10;radius+5 10;radius+5 0],[2,3,4],4.9);
PL_thread_brace = [PL_thread_brace;NaN NaN;PLtrans(PLcircle(1),[radius 5])];
SG_thread_brace = SGofCPLz(PL_thread_brace,10);
SG_thread_brace = SGtransrelSG(SG_thread_brace,SG_brace,'rotx',pi/2,'rotz',pi/2,'ontop','alignleft');
SG_thread_brace = SGcat(SG_thread_brace,SGmirror(SG_thread_brace,'xz'));


SG = SGcat(SG_screw_plate,SG_brace,SG_tensioning,SG_thread_brace);
% SGs = SGanalyzeGroupParts(SG);
% SGs.SG(2) = SGtransrelSG(SGs.SG(2),SGs.SG(1),'aligntop','right');
% SG = SGcat(SGs);
clf;
% SGplot(SG);

end