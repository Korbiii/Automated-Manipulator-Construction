%%  [SG] = SGmotormountSM85BL(radius)
%	=== INPUT PARAMETERS ===
%	radius:        radius of rotordisk
%	=== OUTPUT RESULTS ======
%	SG:         SG of Servomount
function [SG] = SGmotormountSM85BL(radius)
%% Dimensions of Servo
servo_d = [62 34 47];

PL_screw_holes = [PLcircle(1.5);NaN NaN;PLtrans(PLcircle(1.5),[0 28]);NaN NaN;PLtrans(PLcircle(1.5),[0 56])];
PL_screw_holes = PLtrans0([PL_screw_holes;NaN NaN;PLtrans(PL_screw_holes,[28 0]);NaN NaN;PLtrans(PLcircle(6),[14 15])]);
maxY = max(PL_screw_holes(:,2));

CPL_base_outl = [-radius-2 -maxY-5;-radius-2 maxY+30;radius+2 maxY+30;radius+2 -maxY-5];
CPL_base = [CPL_base_outl;NaN NaN; PL_screw_holes];
CPL_base_screw_head = [CPL_base_outl;NaN NaN; PLgrow(PL_screw_holes,2)];
SG_base = SGofCPLz(CPL_base,2.5);
SG_base_screw_head = SGofCPLz(CPL_base_screw_head,2.5);
SG_base = SGcat(SGunder(SG_base_screw_head,SG_base),SG_base);

PL_front = [-2 2;4 2;4 -2;-2 -2;NaN NaN;PLcircle(0.4)];
SG_front = SGofCPLz(PL_front,2);
PL_middle = [-2 2;4 2;4 -2;-2 -2;-2 -1;1 -1;1 1;-2 1];
SG_middle  = SGofCPLz(PL_middle,12.5);
SG_crimp_holder = SGcat(SG_front,(SGunder(SG_middle,SG_front)));
PL_back = [-2 2;4 2;4 -2;-2 -2;-2 -0.3;1 -0.5;1 0.5;-2 0.3];
SG_back  = SGofCPLz(PL_back,5);
SG_crimp_holder = SGtrans(SGcat(SG_crimp_holder,(SGunder(SG_back,SG_crimp_holder))),TofR(rotx(90)));
SG_crimp_holder = SGtransrelSG(SG_crimp_holder,SG_base,'alignleft','alignbehind','ontop',servo_d(3));
[crimp_width,~,crimp_height] =sizeVL(SG_crimp_holder.VL);
SG_crimp_holder = SGstackn(SG_crimp_holder,3,0);
[~, y, z] = sizeVL(SG_crimp_holder.VL);
[base_width,~,~] =sizeVL(SG_base.VL);
% PL_brace = [0 z;0 0;base_width-crimp_width-10 -servo_d(3);base_width-crimp_width -servo_d(3)];
PL_brace = [0 z;0 0;base_width-crimp_width-10 -servo_d(3);base_width-(2*crimp_width) -servo_d(3);base_width-(2*crimp_width) -servo_d(3)+(2*crimp_height);base_width-crimp_width -servo_d(3)+(2*crimp_height);base_width-crimp_width 0;base_width-2*crimp_width 0;base_width-crimp_width-10 -35];
SG_brace = SGtrans(SGofCPLz(PL_brace,y),TofR(rotx(90)));
SG_brace = SGtransrelSG(SG_brace,SG_crimp_holder,'alignbehind','right','aligntop');
SG_crimp_holder = SGcat(SG_crimp_holder,SG_brace);
SG_crimp_holder = SGcat(SGmirror(SG_crimp_holder,'yz'),SG_crimp_holder);

% SGplot(rotor_servo(25,SG_connector_SM85,[10.5 4 1.5]));

[~,y,~] = sizeVL(SG_base);
SG_guide = SGtrans(SGofCPLz(PL_back,y),TofR(rotx(90)));
SG_guide = SGstackn(SG_guide,2,0);
SG_guide = SGtransrelSG(SG_guide,SG_base,'alignleft','ontop','alignbehind');

[~,y,z] = sizeVL(SG_guide);
PL_guide_brace = [servo_d(2)/2-1.5 0;radius-crimp_width 0;radius-crimp_width z;servo_d(2)/2-1.5 z];
SG_guide_brace =  SGtrans(SGofCPLz(PL_guide_brace,y),TofR(rotx(90)));
SG_guide_brace = SGtransrelSG(SG_guide_brace,SG_guide,'alignbehind','right','aligntop');
SG_guide = SGcat(SG_guide_brace,SG_guide);
SG_guide = SGcat(SG_guide,SGmirror(SG_guide,'yz'));

SGplot(SGontop(SGofCPLcommand('c 60,h 3,c 26,h 5,move 0 -17,b 34 62,h 47'),SG_base));
SG = SGcat(SGcat(SG_base,SG_crimp_holder),SG_guide);
end