%%  [SG] = SGpushpullmountSM85BL()
%	=== INPUT PARAMETERS ===
%	=== OUTPUT RESULTS ======
%	SG:         SG of Servomount
function [SG] = SGpushpullmountSM85BL()
%% Dimensions of Servo
servo_d = [62 34 47];
sledge_x = 20;

%% Sledge
PL_sledge = [-sledge_x/2 0;-sledge_x/2-5 -5;sledge_x/2+5 -5;sledge_x/2 0];
SG_sledge = SGofCPLy(PL_sledge,45);

SG_sledge_gear_rack = SGofCPLz(PLgearrackDIN(1,14,8),10);
SG_sledge_gear_rack = SGtransrelSG(SG_sledge_gear_rack,SG_sledge,'rotz',pi/2,'centery','under');

PL_fork = CPLbool('-',PLsquare(8,20),PLsquare(8,10));
SG_fork = SGofCPLz(PL_fork,18);
SG_fork = SGtransrelSG(SG_fork,SG_sledge,'centery','ontop');

%% Base
PL_base = PLtrans(PLsquare(sledge_x+20,10),[0 -5]);
PL_base = CPLbool('-',PL_base,PLsquare(sledge_x,50));
PL_base = CPLbool('-',PL_base,PLgrow(PL_sledge,0.3,true));
SG_base = SGofCPLy(PL_base,100);
SG_base = SGtransrelSG(SG_base,SG_sledge,'transy',10);

%% Gear
SG_gear =  SGofCPLz([PLgearDIN(1,20,1);NaN NaN;PLcircle(1)],5);
SG_gear = SGtransrelSG(SG_gear,SG_sledge_gear_rack,'alignbottom','left',-2,'centery');

%% Rest
SG_act = SGcat(SGbox([25,60,18]),SGtrans(SGbox([17,24,12]),[0 42 0]));
SG_act = SGtransrelSG(SG_act,SG_fork,'alignbottom','alignback',-5);
SG_servo = SGbox(servo_d);
SG_servo = SGtransrelSG(SG_servo,SG_gear,'centerx',16,'centery','under',2);
SG = SGcat(SG_sledge,SG_base,SG_fork,SG_act,SG_sledge_gear_rack,SG_gear,SG_servo);




end