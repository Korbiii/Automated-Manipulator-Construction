%%   [SG] = SGtoolmanipulator()
%	=== INPUT PARAMETERS ===
%	=== OUTPUT RESULTS ======
function [SG] = SGtoolmanipulator(CPL_servo,SG_servo_connector)
%% Initializing of values
distance = 5;   %% tool opening distance im mm
maxY = max(CPL_servo(:,2));
maxX = max(CPL_servo(:,1));
SG_servo_mount = SGofCPLz([PLsquare(maxX*2.5,maxY*2.5);NaN NaN;CPL_servo],5);

%% Sledge
SG_toolholder_base = SGof2CPLsz([PLsquare(50,115);NaN NaN;PLsquare(20,40)],[PLsquare(40,115);NaN NaN;PLsquare(20,40)],10);

PL_front_sledge_plate = [PLtrans(PLsquare(25,54),[7.5 0]);NaN NaN;PLtrans(PLcircle(6.3),[10 15])];
PL_front_sledge_plate = CPLbool('-',PL_front_sledge_plate,PLtrans(PLsquare(25,9),[-10,-22.5]));
SG_front_sledge_plate = SGofCPLy(PL_front_sledge_plate,10);
SG_front_sledge_plate = SGtransrelSG(SG_front_sledge_plate,SG_toolholder_base,'ontop','alignback',-20);

PL_back_sledge_plate = [PLsquare(40,54);NaN NaN;PLtrans(PLsquare(10.2,10.2),[10 15])];
PL_back_sledge_plate = CPLbool('-',PL_back_sledge_plate,[PLtrans(PLsquare(25,9),[-10,-22.5]);NaN NaN;PLtrans(PLsquare(15,17.125),[-12.5,18.44])]);
SG_back_sledge_plate = SGofCPLy(PL_back_sledge_plate,10);
SG_back_sledge_plate = SGtransrelSG(SG_back_sledge_plate,SG_toolholder_base,'alignfront','ontop');

SG_cam_servo_bracket = SGof2CPLz(PLsquare(maxY*2.5,5),PLtrans(PLsquare(40,12.5),[0 3.75]),15);
SG_cam_servo_bracket = SGtransrelSG(SG_cam_servo_bracket,SG_back_sledge_plate,'rotx',-pi/2,'infront','centerx','aligntop',-32.5);

SG_cam_servo_mount =  SGtransrelSG(SG_servo_mount,SG_cam_servo_bracket,'rotz',pi/2,'aligntop','alignleft','infront');

SG_sledge_gear_rack = SGofCPLz(PLgearrackDIN(2,18,8),9);
SG_sledge_gear_rack = SGtransrelSG(SG_sledge_gear_rack,SG_toolholder_base,'rotz',pi/2,'alignback','center','ontop');

SG_rotator_servo_mount = SGtrans(SG_servo_mount,TofR(roty(90)*rotx(90)));
SG_rotator_servo_mount = SGtransrelSG(SG_rotator_servo_mount,SG_back_sledge_plate,'alignleft',-5,'behind',38,'alignbottom',-11.5);
SG_rotator_servo_mount_bracket = SGofCPLy(PLsquare(maxY*2.5,2.5),5);
SG_rotator_servo_mount_bracket = SGtransrelSG(SG_rotator_servo_mount_bracket,SG_rotator_servo_mount,'under','alignleft','alignback');
SG_rotator_servo_mount_bracket_2 = SGof2CPLsz(PLsquare(18,5),PLtrans(PLsquare(50,5),[16 0]),9);
SG_rotator_servo_mount_bracket_2 = SGtransrelSG(SG_rotator_servo_mount_bracket_2,SG_rotator_servo_mount_bracket,'alignright','under','alignback');
SG_rotator_servo_mount_bracket_3 = SGofCPLy(PLsquare(5,maxX*2.5+2.5),44);
SG_rotator_servo_mount_bracket_3 = SGtransrelSG(SG_rotator_servo_mount_bracket_3,SG_rotator_servo_mount,'left','alignback','aligntop');
SG_rotator_servo_mount_top_guide = SGtrans(SGofCPLz([PLsquare(15,10);NaN NaN;PLtrans(PLcircle(3.15),[0 0])],5),TofR(rotx(90)));
SG_rotator_servo_mount_top_guide = SGtransrelSG(SG_rotator_servo_mount_top_guide,SG_rotator_servo_mount,'ontop','alignback','transx',10);

%% Cam Rotor
SG_cam_rotor = SGofCPLz([PLcircle(15);NaN NaN;PLcircle(1.5)],5);
SG_cam_rotor_stud = SGofCPLz([PLtrans(PLcircle(8),[0 distance]);NaN NaN;PLcircle(1.5)],5);
SG_cam_rotor = SGstack('z',SG_servo_connector,SG_cam_rotor,SG_cam_rotor_stud);
SG_cam_rotor = SGtransrelSG(SG_cam_rotor,SG_cam_servo_mount,'centery','centerx',10,'ontop',8);

%% Cam Guide
SG_cam_guide_front = SGofCPLy(PLsquare(10),25);
SG_cam_guide_front_hole = SGofCPLy([PLsquare(10);NaN NaN;PLcircle(2)],4);
SG_cam_guide_front_gap = SGofCPLy(CPLbool('-',PLsquare(10),PLsquare(6,12)),4);
SG_cam_guide_front = SGstack('y',SG_cam_guide_front_hole,SG_cam_guide_front_gap,SG_cam_guide_front_hole,SG_cam_guide_front);

SG_cam_guide_front_insert = SGofCPLy(CPLbool('-',PLsquare(5.5,9.5),PLtrans(PLsquare(2,10),[0 -3.5])),3.5);
SG_temp_connection = SGtransrelSG(SGofCPLy(PLcircle(0.5),3),SG_cam_guide_front,'aligntop');
SG_cam_guide_front = SGstack('y',SG_cam_guide_front_insert,SG_temp_connection,SG_cam_guide_front);

SG_cam_guide_back = SGofCPLz([PLsquare(23.2+distance,16.2);NaN NaN;PLsquare(23.2+distance+10,16.2+10)],10);
SG_cam_guide_back = SGtransrelSG(SG_cam_guide_back,SG_cam_guide_front,'aligntop','centerx');
SG_guide = SGstack('y',SG_cam_guide_front,SG_cam_guide_back);
SG_guide = SGtransrelSG(SG_guide,SG_back_sledge_plate,'infront',-34,'aligntop',-7,'centerx',10);

%% Base
SG_main_guide = SGofCPLy([5 0;0 10;15 10;15 0],120.06);
SG_main_guide = SGtransrelSG(SG_main_guide,SG_toolholder_base,'alignbottom',0.3,'alignfront',15,'right',-4.7);
SG_main_guide = SGcat(SGmirror(SG_main_guide,'yz'),SG_main_guide);

[sx,sy,~,~,~,~] = sizeVL(SG_main_guide);

SG_main_guide_base = SGofCPLz([PLsquare(sx,sy);NaN NaN;PLsquare(sx-30,sy-20)],10);
SG_main_guide_base = SGtransrelSG(SG_main_guide_base,SG_main_guide,'under','alignback');

SG_main_servo_mount = SGtransrelSG(SG_servo_mount,SG_main_guide,'centery','left',-13);
SG_main_servo_mount = SGtransrelSG(SG_main_servo_mount,SG_sledge_gear_rack,'ontop',9);

PL_main_servo_bracket = CPLbool('-',PLsquare(43,100),[PLtrans(PLsquare(38,86),[2.5 0])]);
PL_main_servo_bracket = CPLbool('-',PL_main_servo_bracket,[PLtrans(PLsquare(5,maxY*2.4),[-19 -10])]);
SG_main_servo_bracket = SGofCPLx(PL_main_servo_bracket,maxX*2);
SG_main_servo_bracket = SGtransrelSG(SG_main_servo_bracket,SG_main_servo_mount,'aligntop','alignleft');
SG_main_servo_bracket = SGtransrelSG(SG_main_servo_bracket,SG_main_guide,'alignback');

%% Base Gear
SG_base_gear = SGofCPLz([PLgearDIN(2,32,1);NaN NaN;PLcircle(1.5)],7);
SG_base_gear = SGcat(SG_base_gear,SGtransrelSG(SG_servo_connector,SG_base_gear,'centery','ontop'));
SG_base_gear = SGtransrelSG(SG_base_gear,SG_main_servo_mount,'centerx','centery',10,'under',8);

%% Tool Rotating Socket 
SG_clamp = SGofCPLz([PLcircle(3.1);NaN NaN;PLcircle(8)],10);
SG_screw_hole = SGofCPLz(PLcircle(1.6),10);
SG_clamp = SGtrans(SG_clamp,TofR(rotx(90),[0 5 0]));
SG_clamp = SGtransrelSG(SGbool3('-',SG_clamp,SG_screw_hole),SG_front_sledge_plate,'infront',0.3,'transx',10,'centerz',15);

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
SG_sledge = SGcat(SG_cam_servo_mount,SG_cam_servo_bracket,SG_gear_rotating_socket,SG_front_sledge_plate,SG_toolholder_base,SG_back_sledge_plate,SG_sledge_gear_rack,SG_rotator_servo_mount,SG_rotator_servo_mount_bracket,SG_rotator_servo_mount_top_guide,SG_rotator_servo_mount_bracket_3,SG_rotator_servo_mount_bracket_2);

%% Combining Elements of Sledge and adding space for screw
SG_base = SGcat(SG_main_guide_base,SG_main_guide,SG_main_servo_mount,SG_main_servo_bracket);

SG_screw_space = SGtransrelSG(SGofCPLz(PLcircle(10),25),SG_base,'alignbottom',0.3,'left',-20,'centery',10);
SG_base = SGbool3('-',SG_base,SG_screw_space);

%% Combining all Elements as Assembly
SG = SGcat(SG_sledge,SG_cam_rotor,SG_guide,SG_base,SG_base_gear,SG_gear_rotator);

%% Writing STL Files
SGwriteSTL(SG_sledge,"Sledge",'','y');
SGwriteSTL(SG_base,"Base",'','y');
SGwriteSTL(SG_cam_rotor,"ExzenterRotor",'','y');
SGwriteSTL(SG_guide,"ExzenterLaeufer",'','y');
SGwriteSTL(SG_base_gear,"ZahnradHauptbewegung",'','y');
SGwriteSTL(SG_gear_rotator,"Tool Rotating Gear",'','y');
SGwriteSTL(SG,"Assembly",'','y');

end