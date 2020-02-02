%%  [SG] = SGpushpullmountSM85BL()
%	=== INPUT PARAMETERS ===
%	=== OUTPUT RESULTS ======
%	SG:         SG of Servomount
function [SG] = SGpushpullmountSM40CL(SG_connector)
%% Dimensions of Servo
servo_d = [28.5,46.5,34.5];
sledge_x = 20;

%% Sledge
PL_sledge = [-sledge_x/2 0;-sledge_x/2-5 -5;sledge_x/2+5 -5;sledge_x/2 0];
SG_sledge = SGofCPLy(PL_sledge,45);

SG_sledge_gear_rack = SGofCPLz(PLgearrackDIN(1,14,8),12);
SG_sledge_gear_rack = SGtransrelSG(SG_sledge_gear_rack,SG_sledge,'rotz',pi/2,'centery','under');

PL_fork = CPLbool('-',PLsquare(8,20),PLsquare(8,5));
SG_fork = SGofCPLz(PL_fork,18);
SG_fork = SGtransrelSG(SG_fork,SG_sledge,'centery','ontop');


%% Gear
SG_gear =  SGofCPLz([PLgearDIN(1,30,1);NaN NaN;PLcircle(12.5)],6);
SG_gear = SGtransrelSG(SG_gear,SG_sledge_gear_rack,'alignbottom','left',-2,'centery');
PL_connector_middle = [PLcircle(12.5);NaN NaN;PLcircle(1.5);NaN NaN;CPLcopyradial(PLcircle(1.5),7,4)];
SG_connector_middle = SGofCPLz(PL_connector_middle,1.5);
SG_connector = SGstack('z',SG_connector,SG_connector_middle);
SG_connector = SGtransrelSG(SG_connector,SG_gear,'center','alignbottom');

SG_gear = SGcat(SG_gear,SG_connector);

%% Base
PL_base = PLtrans(PLsquare(sledge_x+20,10),[0 -4.7]);
PL_base = CPLbool('-',PL_base,PLsquare(sledge_x,50));
PL_base = CPLbool('-',PL_base,PLgrow(PL_sledge,0.3,true));
SG_base = SGofCPLy(PL_base,100);
SG_base = SGtransrelSG(SG_base,SG_sledge,'transy',10);
size_base = SGsize(SG_base);
PL_stops = PLsquare(abs(size_base(1))+abs(size_base(2)),abs(size_base(5))+abs(size_base(6)));
SG_stops_f =SGtransrelSG( SGofCPLy(PL_stops,5),SG_base,'aligntop','behind','alignleft');
SG_stops_b = SGtransrelSG( SGofCPLy(PL_stops,20),SG_base,'aligntop','infront','alignleft');
SG_stops = SGcat(SG_stops_f,SG_stops_b);
PL_servo_mount = [PLsquare(46,26.5);NaN NaN;PLtrans(PLpatternXY(PLcircle(1.5),2,2,24,12),[4 0])];
PL_servo_mount = CPLbool('-',PL_servo_mount,PLtrans(PLsquare(23.25,17),[-11.625,-8.5]));
PL_servo_mount = PLroundcorners(PL_servo_mount,[1,2,3,4],[5,10,5,5]);
SG_servo_mount = SGofCPLx(PL_servo_mount,5);
SG_servo_mount_l = SGtransrelSG(SG_servo_mount,SG_gear,'center','centery',-12,'under',2,'transx',(-servo_d(1)/2)-2.5);
SG_servo_mount_r = SGtransrelSG(SG_servo_mount,SG_gear,'center','centery',-12,'under',2,'transx',(servo_d(1)/2)+2.5);
SG_servo_mount = SGcat(SG_servo_mount_l,SG_servo_mount_r);

PL_bracket_left = [0 0;-5 0;-5 19;5 19;5 9;0 9];
SG_bracket_left = SGofCPLy(PL_bracket_left,servo_d(2));
SG_bracket_left = SGtransrelSG(SG_bracket_left,SG_servo_mount_l,'ontop','alignleft','alignfront');

PL_bracket_right = [5 -5;15.7 -5;15.7 9.3;10.7 9.3;10.7 0;5 0];
SG_bracket_right = SGofCPLy(PL_bracket_right,servo_d(2));
SG_bracket_right = SGtransrelSG(SG_bracket_right,SG_servo_mount_r,'ontop',-5,'alignfront','right');

PL_bracket = CPLbool('-',PLsquare(70,35),CPLbool('-',PLsquare(50,40),PLsquare(25,40)));
PL_bracket_top = CPLbool('-',PL_bracket,PLsquare(60,20));
PL_bracket_top = CPLbool('-',PL_bracket_top,PLsquare(80,16));
PL_bracket_bottom = CPLbool('-',PL_bracket,PLsquare(60,25));
PL_bracket_bottom = CPLbool('-',PL_bracket_bottom,PLsquare(80,16));
SG_bracket_top = SGofCPLz(PL_bracket_top,3);
SG_bracket_bottom = SGofCPLz(PL_bracket_bottom,17.6);
SG_bracket = SGstack('z',SG_bracket_bottom,SG_bracket_top);
SG_bracket = SGtransrelSG(SG_bracket,SG_base,'rotz',pi/2,'ontop');
SG_bracket = SGtransrelSG(SG_bracket,SG_fork,'infront',5);


%% Rest
SG_act = SGcat(SGbox([25,60,18]),SGtrans(SGbox([17,24,12]),[0 42 0]));
SG_act = SGtransrelSG(SG_act,SG_fork,'alignbottom','alignback',-5);
SG_servo = SGbox(servo_d);
SG_servo = SGtransrelSG(SG_servo,SG_gear,'centerx','centery',-12,'under',2);
SG = SGcat(SG_sledge,SG_stops,SG_base,SG_fork,SG_act,SG_sledge_gear_rack,SG_gear,SG_servo,SG_servo_mount,SG_bracket_left,SG_bracket_right,SG_bracket);




end