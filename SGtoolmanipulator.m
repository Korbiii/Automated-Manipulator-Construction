%%   [SG] = SGtoolmanipulator()
%	=== INPUT PARAMETERS ===
%	=== OUTPUT RESULTS ======
function [SG] = SGtoolmanipulator(CPL_servo,SG_servo_connector)
%% Initializing of values


distance = 7.5;   %% tool opening distance im mm
maxY = max(CPL_servo(:,2));
maxX = max(CPL_servo(:,1));
SG_servo_mount = SGofCPLz([PLsquare(maxX*2.5,maxY*2.1);NaN NaN;CPL_servo],5);
[~,servo_sy,~,~,~,~] = sizeVL(SG_servo_mount);


%% Sledge
SG_toolholder_base = SGof2CPLsz([PLsquare(50,115);NaN NaN;PLsquare(20,40)],[PLsquare(40,115);NaN NaN;PLsquare(20,40)],10);

PL_front_sledge_plate = PLtrans(PLsquare(25,50),[7.5 0]);
PL_front_sledge_plate = CPLbool('-',PL_front_sledge_plate,PLtrans(PLcircle(6.3),[10 25]));
PL_front_sledge_plate = CPLbool('-',PL_front_sledge_plate,PLtrans(PLsquare(25,9),[-10,-21.5]));
SG_front_sledge_plate = SGofCPLy(PL_front_sledge_plate,10);
SG_front_sledge_plate = SGtransrelSG(SG_front_sledge_plate,SG_toolholder_base,'ontop','alignback',-20);

SG_hinge_fixed = SGofCPLy(PLcircle(3),6);
SG_hinge_moving = SGofCPLy([PLcircle(3.2);NaN NaN;PLcircle(6)],5.5);
SG_hinge_moving = SGtransrelSG(SG_hinge_moving,SG_hinge_fixed,'center');

PL_hinge_fixed_connection = [-3 0;7 0;7 -10];
PL_hinge_fixed_connection = CPLbool('+',PL_hinge_fixed_connection,PLcircle(3));
SG_hinge_fixed_connetion = SGofCPLy(PL_hinge_fixed_connection,2);
SG_hinge_fixed = SGstack('y',SG_hinge_fixed_connetion,SG_hinge_fixed,SG_hinge_fixed_connetion);

SG_hinge_moving = SGtransrelSG(SG_hinge_moving,SG_hinge_fixed,'centery');
PL_hinge_moving_connection = [-6 0;7 10;7 0];
PL_hinge_moving_connection = CPLbool('-',PL_hinge_moving_connection,PLcircle(3.2));
SG_hinge_moving_connection = SGofCPLy(PL_hinge_moving_connection,5.5);
SG_hinge_moving_connection = SGtransrelSG(SG_hinge_moving_connection,SG_hinge_moving,'centery');
SG_hinge_moving = SGcat(SG_hinge_moving,SG_hinge_moving_connection);


PL_front_top = PLtrans(PLsquare(25,10),[7.5 0]);
PL_front_top = CPLbool('-',PL_front_top,PLtrans(PLcircle(6.3),[10 -5]));
SG_front_top = SGofCPLy(PL_front_top,10);
SG_front_top = SGtransrelSG(SG_front_top,SG_hinge_moving,'centery','right','aligntop');

PL_closer_guide = [CPLbool('+',PLtrans(PLsquare(5,10),[-2.5 0]),PLcircle(5));NaN NaN;PLcircle(3)];
SG_closer_guide = SGofCPLz(PL_closer_guide,10);
SG_closer_guide = SGtransrelSG(SG_closer_guide,SG_front_top,'right','aligntop','alignfront');

SG_closer_screw = SGofCPLz(PL_closer_guide,10);
SG_closer_screw = SGcat(SG_closer_screw,SGscrewDIN(-5,10,'',PLcircle(5)));
SG_closer_screw = SGtransrelSG(SG_closer_screw,SG_front_sledge_plate,'right','aligntop','alignfront');

SG_front_top = SGcat(SG_front_top,SG_closer_guide);
SG_hinge_moving = SGcat(SG_hinge_moving,SG_front_top);
SG_hinge_moving = SGtrans(SG_hinge_moving,TofR(roty(-10)));
SG_hinge = SGcat(SG_hinge_moving,SG_hinge_fixed);
SG_hinge = SGtransrelSG(SG_hinge,SG_front_sledge_plate,'centery','ontop',-10,'alignleft',13);

PL_back_sledge_plate = [PLsquare(40,60);NaN NaN;PLtrans(PLsquare(10.2,10.2),[10 21])];
PL_back_sledge_plate = CPLbool('-',PL_back_sledge_plate,PLtrans(PLsquare(20,8),[-10 -26]));
SG_back_sledge_plate = SGofCPLy(PL_back_sledge_plate,10);
SG_back_sledge_plate = SGtransrelSG(SG_back_sledge_plate,SG_toolholder_base,'alignfront','ontop');

SG_cam_servo_bracket = SGof2CPLz(PLsquare(servo_sy,5),PLtrans(PLsquare(40,12.5),[0 3.75]),15);
SG_cam_servo_bracket = SGtransrelSG(SG_cam_servo_bracket,SG_back_sledge_plate,'rotx',-pi/2,'infront','centerx','aligntop',-29.5);

SG_cam_servo_mount =  SGtransrelSG(SG_servo_mount,SG_cam_servo_bracket,'rotz',pi/2,'aligntop','alignleft','infront');

SG_sledge_gear_rack = SGofCPLz(PLgearrackDIN(2,18,8),8);
SG_sledge_gear_rack = SGtransrelSG(SG_sledge_gear_rack,SG_toolholder_base,'rotz',pi/2,'alignback','center','ontop');

SG_rotator_servo_mount = SGtrans(SG_servo_mount,TofR(roty(90)*rotx(90)));
SG_rotator_servo_mount = SGtransrelSG(SG_rotator_servo_mount,SG_back_sledge_plate,'alignleft',5,'behind',38,'alignbottom',-12);
SG_rotator_servo_mount_bracket = SGofCPLy(PLsquare(servo_sy,2.5),5);
SG_rotator_servo_mount_bracket = SGtransrelSG(SG_rotator_servo_mount_bracket,SG_rotator_servo_mount,'under','alignleft','alignback');
SG_rotator_servo_mount_bracket_2 = SGof2CPLsz(PLsquare(18,5),PLtrans(PLsquare(30,5),[5 0]),9);
SG_rotator_servo_mount_bracket_2 = SGtransrelSG(SG_rotator_servo_mount_bracket_2,SG_rotator_servo_mount_bracket,'alignright','under','alignback');
SG_rotator_servo_mount_bracket_3 = SGof2CPLsz(PLsquare(servo_sy,5),PLtrans(PLsquare(40,5),[-3 0]),43);
SG_rotator_servo_mount_bracket_3 = SGtransrelSG(SG_rotator_servo_mount_bracket_3,SG_rotator_servo_mount,'rotx',pi/2,'ontop','alignback','centerx');
height_guide = 8;
PL_rotator_servo_mount_top_guide = [0 0;10 0;10 -5;5 -5;5 -height_guide;0 -height_guide];
PL_rotator_servo_mount_top_guide = [flip(PL_rotator_servo_mount_top_guide);VLswapX(PL_rotator_servo_mount_top_guide)];

PL_rotator_servo_mount_top_guide = CPLbool('-',PL_rotator_servo_mount_top_guide,PLcircle(3.1));
SG_rotator_servo_mount_top_guide = SGofCPLy(PL_rotator_servo_mount_top_guide,10);
SG_rotator_servo_mount_top_guide = SGtransrelSG(SG_rotator_servo_mount_top_guide,SG_rotator_servo_mount_bracket_3,'ontop','alignback','transx',10);

PL_rotator_servo_mount_top_guide_fixer = [0 0;10.2 0;10.2 -5.2;5 -5.2;5 -height_guide;0 -height_guide];
PL_rotator_servo_mount_top_guide_fixer = [flip(PL_rotator_servo_mount_top_guide_fixer);VLswapX(PL_rotator_servo_mount_top_guide_fixer)];
PL_rotator_servo_mount_top_guide_fixer = CPLbool('-',PLsquare(30,15),PL_rotator_servo_mount_top_guide_fixer);
PL_rotator_servo_mount_top_guide_fixer = CPLbool('-',PL_rotator_servo_mount_top_guide_fixer,PLcircle(3.1));
SG_rotator_servo_mount_top_guide_fixer = SGofCPLy(PL_rotator_servo_mount_top_guide_fixer,10);
SG_rotator_servo_mount_top_guide_fixer = SGtransrelSG(SG_rotator_servo_mount_top_guide_fixer,SG_rotator_servo_mount_top_guide,'infront',2,'ontop',-5,'centerx'); 
%% Cam Rotor
SG_cam_rotor = SGofCPLz([PLcircle(15);NaN NaN;PLcircle(1.5)],5);
SG_cam_rotor_stud = SGofCPLz([PLtrans(PLcircle(9),[0 distance]);NaN NaN;PLcircle(1.5)],7.5);
SG_cam_rotor = SGstack('z',SG_servo_connector,SG_cam_rotor,SG_cam_rotor_stud);
SG_cam_rotor = SGtransrelSG(SG_cam_rotor,SG_cam_servo_mount,'centery','centerx',10,'ontop',8);

%% Cam Guide    
SG_cam_guide_front = SGofCPLy(PLsquare(10),30);
PL_cam_guide_tip = [PLsquare(10);NaN NaN;PLcircle(2)];
PL_cam_guide_tip = CPLbool('-',PL_cam_guide_tip,PLtrans(PLsquare(2.2,10),[0 5]));
SG_cam_guide_tip = SGofCPLz(PL_cam_guide_tip,5);
SG_cam_guide_tip = SGtrans0(SGstack('z',SGofCPLz(PLsquare(10),5),SG_cam_guide_tip));

SG_cam_guide_front = SGstack('y',SG_cam_guide_tip,SG_cam_guide_front);

SG_cam_guide_back = SGofCPLz([PLsquare(23.2+distance,18.2);NaN NaN;PLsquare(23.2+distance+10,18.2+10)],10);
SG_cam_guide_back = SGtransrelSG(SG_cam_guide_back,SG_cam_guide_front,'aligntop','centerx');
SG_guide = SGstack('y',SG_cam_guide_front,SG_cam_guide_back);
SG_guide = SGtransrelSG(SG_guide,SG_back_sledge_plate,'alignfront',36,'aligntop',-4,'centerx',10);

%% Base
SG_main_guide = SGofCPLy([5 0;0 10;15 10;15 0],120.06);
SG_main_guide = SGtransrelSG(SG_main_guide,SG_toolholder_base,'alignbottom',0.3,'alignfront',15,'right',-4.7);
SG_main_guide = SGcat(SGmirror(SG_main_guide,'yz'),SG_main_guide);

[sx,sy,~,~,~,~] = sizeVL(SG_main_guide);

SG_main_guide_base = SGofCPLz([PLsquare(sx,sy);NaN NaN;PLsquare(sx-30,sy-20)],10);
SG_main_guide_base = SGtransrelSG(SG_main_guide_base,SG_main_guide,'under','alignback');

SG_main_servo_mount = SGtransrelSG(SG_servo_mount,SG_main_guide,'centery','left',-8);
SG_main_servo_mount = SGtransrelSG(SG_main_servo_mount,SG_sledge_gear_rack,'ontop',6);

PL_main_servo_bracket = CPLbool('-',PLsquare(100,39),[PLtrans(PLsquare(86,34),[0 -2.5])]);
PL_main_servo_bracket = CPLbool('-',PL_main_servo_bracket,[PLtrans(PLsquare(servo_sy,5),[-10 17])]);
SG_main_servo_bracket = SGofCPLx(PL_main_servo_bracket,maxX*2);
SG_main_servo_bracket = SGtransrelSG(SG_main_servo_bracket,SG_main_servo_mount,'aligntop','alignright');
SG_main_servo_bracket = SGtransrelSG(SG_main_servo_bracket,SG_main_guide,'alignback');

%% Base Gear
SG_base_gear = SGofCPLz([PLgearDIN(2,36);NaN NaN;PLcircle(15)],6);
SG_servo_connector_base = SGofCPLcommand('c 5.6 8 3,move 13 0,enter,c 30,c 35,-,enter,c 16.5,-,h 3,dupr 6,col y');
SG_servo_connector_base_gear = SGstack('z',SGofCPLz([PLcircle(15);NaN NaN;PLcircle(1.5)],3),SG_servo_connector_base);
SG_base_gear = SGcat(SG_base_gear,SGtransrelSG(SG_servo_connector_base_gear,SG_base_gear,'centery','alignbottom'));
SG_base_gear = SGtransrelSG(SG_base_gear,SG_main_servo_mount,'centerx','centery',10,'under',7);

%% Tool Rotating Socket 
SG_clamp = SGofCPLz([PLcircle(3.1);NaN NaN;PLcircle(8)],10);
SG_screw_hole = SGofCPLz(PLcircle(1.6),10);
SG_clamp = SGtrans(SG_clamp,TofR(rotx(90),[0 5 0]));
SG_clamp = SGtransrelSG(SGbool3('-',SG_clamp,SG_screw_hole),SG_front_sledge_plate,'infront',0.3,'transx',10,'ontop',-8);

SG_gear_rotating_socket = SGofCPLz([PLgearDIN(2,10);NaN NaN;PLcircle(3.15)],10);
SG_gear_rotating_socket = SGtransrelSG(SG_gear_rotating_socket,SG_clamp,'rotx',pi/2,'center','infront');

SG_spacer = SGofCPLz([PLcircle(3.1);NaN NaN;PLcircle(6)],10.6);
SG_spacer = SGtransrelSG(SG_spacer,SG_clamp,'rotx',pi/2,'center','behind');
SG_clamp_2 = SGtransrelSG(SG_clamp,SG_spacer,'behind');
SG_gear_rotating_socket = SGcolor(SGcat(SG_gear_rotating_socket,SG_clamp,SG_spacer,SG_clamp_2));
%% Gear for Servo that rotates Tool
SG_gear_rotator = SGofCPLz(PLtransR([PLgearDIN(2,16);NaN NaN;PLcircle(1.5)],rot(0.15)),10);
SG_gear_rotator = SGcat(SG_gear_rotator,SGontop(SG_servo_connector,SG_gear_rotator));
SG_gear_rotator = SGtransrelSG(SG_gear_rotator,SG_rotator_servo_mount,'rotx',pi/2,'centerx',10,'centerz','behind',10);

%% Combining Elements Sledge
SG_sledge = SGcat(SG_cam_servo_mount,SG_cam_servo_bracket,SG_gear_rotating_socket,SG_front_sledge_plate,SG_toolholder_base,SG_back_sledge_plate,SG_sledge_gear_rack,SG_rotator_servo_mount,SG_rotator_servo_mount_bracket,SG_rotator_servo_mount_bracket_2,SG_hinge,SG_closer_screw,SG_rotator_servo_mount_bracket_3,SG_rotator_servo_mount_top_guide,SG_rotator_servo_mount_top_guide_fixer);

%% Combining Elements of Sledge and adding space for screw
SG_base = SGcat(SG_main_guide_base,SG_main_guide,SG_main_servo_mount,SG_main_servo_bracket);


%% Combining all Elements as Assembly
SG = SGcat(SG_sledge,SG_cam_rotor,SG_guide,SG_base,SG_base_gear,SG_gear_rotator);

%% Writing STL Files
SGwriteSTL(SG_sledge,"Sledge",'','y');
SGwriteSTL(SG_base,"Base",'','y');
SGwriteSTL(SG_cam_rotor,"ExzenterRotor",'','y');
SGwriteSTL(SG_guide,"ExzenterLaeufer",'','y');
SGwriteSTL(SG_base_gear,"ZahnradHauptbewegung",'','y');
% SGwriteSTL(SG_gear_rotator,"Tool Rotating Gear",'','y');
SGwriteSTL(SG,"Assembly",'','y');

end