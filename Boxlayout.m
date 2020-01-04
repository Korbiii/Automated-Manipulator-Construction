clf;
addpath('Essentials');

% SG_tool_mover_o = SGreadSTL("STLs\Assembly.STL");
% SM40_o = SGreadSTL("STLs\SM40.STL");
% SM85_o = SGreadSTL("STLs\SM85.STL");
SG_tool_mover = SG_tool_mover_o;
SM40 = SM40_o;
SM85 = SM85_o;

SG_inlay = SGboxinlay();
SM40 = SGtrans0(SM40);
SM40 = SGtransrelSG(SM40,SG_inlay,'transx',47,'alignbottom','transy',26);
SM40 = SGcat(SM40,SGmirror(SM40,'yz'));
SM85 = SGtrans0(SM85);
SM85 = SGtransrelSG(SM85,SG_inlay,'alignbottom',-5,'rotz',pi/2,'transx',20,'transy',-65);
SG_tool_mover = SGtransrelSG(SG_tool_mover,SG_inlay,'center','ontop','transx',-50,'transy',25,'transz',-10);




SG = SGcat(SG_inlay,SM40,SM85,SG_tool_mover);
SG = SGcolor(SG);
SGplot(SG);
SGwriteSTL(SG,"Boxlayout");