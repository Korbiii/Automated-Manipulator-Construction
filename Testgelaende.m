clf;clear;
addpath('Essentials');

% SG_tool_mover = SGreadSTL("STLs\Assembly.STL");
% SG_motor_mount = SGreadSTL("STLs\MotormountSM40.STL");
SG_motor_mount = SGreadSTL("STLs\MotormountSM85BL.STL");
SG_rotor = SGreadSTL("STLs\Servorotor.STL");
% SG_servo = SGbox([28.5,46.5,34]);
SG_servo = SGbox([62 34 47]);
% 
SG_motor_mount = SGtransrelSG(SG_motor_mount,SG_servo,'right',-12,'ontop',-16);
SG_rotor = SGtransrelSG(SG_rotor,SG_servo,'center','ontop',2,'transx',-11);
% 
SG = SGcat(SG_motor_mount,SG_servo,SG_rotor);
SG = SGcolor(SG);
SGwriteSTL(SG,"SM85");
SGplot(SG);