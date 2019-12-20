function [SG] = SGboxinlay()


SG_base_mid = SGbox([40 54.5 5]);

PL_base_front = [PLsquare(140,20);NaN NaN;PLpatternXY(PLcircle(2),2,1,120,0)];
SG_base_front = SGofCPLz(PL_base_front,10);

SG_base_front = SGtransrelSG(SG_base_front,SG_base_mid,'behind','alignbottom');

PL_base_back = [PLsquare(140,40);NaN NaN;PLpatternXY(PLcircle(2),2,1,120,0)];
SG_base_back = SGofCPLz(PL_base_back,10);
SG_base_back = SGtransrelSG(SG_base_back,SG_base_mid,'infront','alignbottom');

SG_base_mid_holder = SGbox([56 20 5]);
SG_base_mid_holder = SGtransrelSG(SG_base_mid_holder,SG_base_back,'behind','aligntop');

SG_tool_mover_holder_base = SGbox([110,20,10]);
PL_tool_mover_holder = CPLbool('-',PLsquare(20),PLtrans(PLsquare(10),[5 5]));
SG_tool_mover_holder = SGofCPLz(PL_tool_mover_holder,9);
SG_tool_mover_holder = SGtransrelSG(SG_tool_mover_holder,SG_tool_mover_holder_base,'ontop','alignleft');
SG_tool_mover_holder = SGcat(SG_tool_mover_holder_base,SG_tool_mover_holder,SGmirror(SG_tool_mover_holder,'yz'));
SG_tool_mover_holder = SGtrans(SG_tool_mover_holder,[0 -60 0]);
SG_tool_mover_holder = SGcat(SG_tool_mover_holder,SGmirror(SG_tool_mover_holder,'xz'));
SG_tool_mover_holder = SGtransrelSG(SG_tool_mover_holder,SG_base_mid,'transz',75);

PL_frame = [PLsquare(110,120);NaN NaN;PLsquare(90,100)];
PL_diagonal = PLtransC(PLsquare(140,10),[0 0],pi/3.75);
PL_diagonal = CPLbool('+',PL_diagonal,PLmirror0(PL_diagonal,'x',0));
PL_frame = CPLbool('+',PL_frame,PL_diagonal);
SG_frame = SGofCPLz(PL_frame,10);
SG_frame = SGtransrelSG(SG_frame,SG_tool_mover_holder,'alignbottom');

SG_brace_front = SGof2CPLsz(PLsquare(24,20),PLtrans(PLsquare(24,20),[0 23]),63);
SG_brace_front = SGtransrelSG(SG_brace_front,SG_tool_mover_holder,'alignback','under');
SG_hole = SGtrans(SGofCPLy(PLcircle(4),60),[0 70 55]);
SG_brace_front = SGbool3('-',SG_brace_front,SG_hole);

SG_brace_back_right = SGofCPLz(PLsquare(20,10),63);
SG_brace_back_right = SGtransrelSG(SG_brace_back_right,SG_base_back,'ontop','centery',-2.5,'alignright',-25);
SG_brace_back_right = SGcat(SG_brace_back_right,SGmirror(SG_brace_back_right,'yz'));

PL_base_back_platform = PLsquare(140,62);
PL_sm85_holes = PLtrans(PLpatternXY(PLcircle(1.6),2,3,28,28),[22.25 0]);
PL_board_holes = PLtrans(PLpatternXY(PLcircle(1.6),2,2,49,29),[-35 0]);
PL_base_back_platform = [PL_base_back_platform;NaN NaN;PL_sm85_holes;NaN NaN;PLtrans(PLcircle(4),[22.25 -16]);NaN NaN;PL_board_holes];
SG_base_back_platform =  SGofCPLz(PL_base_back_platform,5);
SG_base_back_platform = SGtransrelSG(SG_base_back_platform,SG_base_back,'alignbottom','infront');

SG_back_servo_guide = SGofCPLz(CPLbool('-',PLsquare(50,62),PLsquare(34.5,62)),5);
SG_back_servo_guide = SGtransrelSG(SG_back_servo_guide,SG_base_back_platform,'ontop','transx',22.25,'alignfront');

SG = SGcat(SG_base_mid,SG_base_front,SG_base_back,SG_base_mid_holder,SG_tool_mover_holder,SG_frame,SG_brace_front,SG_brace_back_right,SG_base_back_platform,SG_back_servo_guide);
SGwriteSTL(SG,"SGinlay");


end