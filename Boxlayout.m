clf;clear;
addpath('Essentials');

SG_tool_mover = SGreadSTL("Z:\mem\HIWI\Krieger\##Studenten\###HIWI\Korbinian Rzepka\SA\SG-Scripts\STLs\Assembly.STL");
SG_motor_mount = SGreadSTL("Z:\mem\HIWI\Krieger\##Studenten\###HIWI\Korbinian Rzepka\SA\SG-Scripts\STLs\MotormountSM40.STL");

SG = SGpatternXY(SG_motor_mount,2,3,80,75,0,-15);
SG_tool_mover = SGtransrelSG(SG_tool_mover,SG,'right',10,'alignbottom');
SG_tool_mover = SGcat(SGmirror(SG_tool_mover,'yz'),SG_tool_mover);
SG = SGcat(SG,SG_tool_mover);

SG = SGcolor(SG);
SGplot(SG);
SGwriteSTL(SG,"Boxlayout");
clear;