clf;clear;
addpath('Essentials');

SG_tool_mover = SGreadSTL("STLs\Assembly.STL");
SG_motor_mount = SGreadSTL("STLs\MotormountSM40.STL");
SG_motormount_85BL = SGreadSTL("STLs\MotormountSM85BL.STL");
SG = SGpatternXY(SG_motor_mount,2,1,90,0,0,-15);
SG_tool_mover = SGtransrelSG(SG_tool_mover,SG,'alignback','ontop',20);

servo_d = [34 62 47];
SG_SM85BL = SGtrans(SGbox(servo_d),[0 -125 -4]);
SG_motormount_85BL = SGtransrelSG(SG_motormount_85BL,SG_SM85BL,'rotz',pi/2,'ontop',-15,'centerx',-7.5,'alignfront',-15);
SG_SM85BL = SGcat(SG_SM85BL,SG_motormount_85BL);
SG = SGcat(SG,SG_tool_mover,SG_SM85BL);

SG = SGcolor(SG);
SGplot(SG);
SGwriteSTL(SG,"Boxlayout");
clear;