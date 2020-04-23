clear all; clf;
addpath('Essentials');
path = prefdir; path = strsplit(path, '\');path{7} = 'MATLAB Add-Ons\Toolboxes\SolidGeometry 4.7'; path = strjoin(path(1:7),'\'); addpath(path);
axis equal; hold on;


%% Kidneys
CPL = PLtrans(PLkidney(7,17,pi/6.5),[-12 0]);
CPL2 = PLtrans(PLkidney(4,14,pi/6.5),[-9 0]);
CPL3 = PLtrans(PLkidney(2,12,pi/12),[-7 0]);

%% Kidneys small
% CPL = PLtrans(PLkidney(9.5,14.5,pi/5.5),[-12.5 0]);
% CPL2 = PLtrans(PLkidney(7,11,pi/4.5),[-9 0]);
% CPL3 = PLtrans(PLkidney(7,11,pi/7),[-9 0]);


%% Ovals
% CPL = PLtrans0(PLtransR(PLcircleoval(5.2,'',5),rot(pi/2)));
% CPL2 = PLtrans0(PLtransR(PLcircleoval(5.2,'',4),rot(pi/2)));
% CPL3 = PLtrans0(PLtransR(PLcircleoval(5.2,'',2),rot(pi/2)));

% %% Ovals small
CPL_o = PLtrans0(PLtransR(PLcircleoval(4,'',5),rot(pi/2)));
% CPL2 = PLtrans0(PLtransR(PLcircleoval(3,'',4),rot(pi/2)));
% CPL3 = PLtrans0(PLtransR(PLcircleoval(3,'',2),rot(pi/2)));

% [SGm,SGc,ranges] = SGmanipulator({CPL},6.8,[90 75 75 1;0 90 90 0;90 10 270 0],[27 4 1.2 1 0.5;30 4 1.0 1 0.5;55 2 0.8 2 0.5],'first_single','optic_radius',3,'optic_top','seal','length',70); %% Aktuellen Manipulator ausgeben
% [SGm,SGc,ranges] = SGmanipulator({CPL},6.8,[90 75 75 1;0 90 90 0;90 10 270 0],[27 4 1.2 0.5 0.5;30 4 1.0 1 0.5;55 2 0.8 2 0.5],'single','optic_radius',3,'optic_top','seal','length',20); %% Aktuellen Manipulator ausgeben
% [SGm,SGc,ranges] = SGmanipulator({CPL},6.8,[90 60 0 1;0 90 90 0;90 10 270 0],[27 4 1.2 2 0.5;30 4 1.0 1 0.5;55 2 0.8 2 0.5],'single','optic_radius',3,'optic_top','length',60,'bottom_up'); %% Aktuellen Manipulator ausgeben
% [SGm,SGc,ranges] = SGmanipulator({CPL},6.8,[90 60 0 1;0 90 90 0;90 10 270 0],[27 4 1.2 2 0.5;30 4 1.0 1 0.5;55 2 0.8 2 0.5],'first_single','optic_radius',3,'optic_top','length',100,'bottom_up','seal'); %% Aktuellen Manipulator ausgeben
% [SGm,SGc,ranges] = SGmanipulator({CPL},6.8,[90 50 0 1;0 90 90 0;90 100 180 0],[27 4 1.2 2 0.5;30 4 1.0 1 0.5;55 2 0.8 2 0.5],'symmetric','first_single','optic_radius',3,'length',140,'optic_top','bottom_up','angles',[-0.6,0,0;-0.6,0,0]); %% Aktuellen Manipulator ausgeben
% [SGm,SGc,ranges] = SGmanipulator({CPL;CPL2;CPL3},3,[90 75 75 -1;0 90 90 0;90 90 180 -1],[10 1 0.8 0.25 1;14 1 0.8 0.25 0.5;20 1 0.8 0.25 0.5],'single','tip','optic_mid','optic_radius',5,'bottom_up','torsion','hole_r',0.4); %% MiniPushpull
% [SGm,SGc,ranges] = SGmanipulator({CPL},6.8,[90;0;90],[27;30;55],'symmetric');

% [SGm,SGc,ranges] = SGmanipulator({{CPL;CPL2},{CPL2;CPL3}},[6;4],{[90 75 75 -1;0 90 90 0;90 90 180 0],[0 20 20 1;90 90 90 0;0 50 50 0]},{[27;30;55],[10;10;10]},'num_arms',2,'bottom_up','first_single','angles',[-0.5,-0.5,0.5;-0.9,-0.5,0.5]);
% [SGm,SGc,ranges] = SGmanipulator({{CPL;CPL2},{CPL2;CPL3}},[6;4],{[90 75 75 1;0 90 90 0;90 120 120 2],[0 20 20 0;90 90 90 0;0 50 50 0]},{[27;30;55],[10;10;10]},'num_arms',2,'bottom_up','first_single','angles',[0 0 1;0 0 0 ],'optic_top');
% [SGm,SGc,ranges] = SGmanipulator({{CPL;CPL2},{CPL2;CPL3},{PLcircle(5)}},[6;4;3],{[90 75 75 1;0 90 90 0;90 120 120 0],[0 20 20 0;90 90 90 0;0 50 50 0],[90 20 20 0;0 90 90 0;0 50 50 2]},{[27;30;55],[10;10;10],[10;10;20]},'angles',[-0.5 0 1;0.5 0 0;-0.5 0 0.5 ],'num_arms',3,'bottom_up','first_single');
% [SGm,SGc,ranges] = SGmanipulator({{CPL;CPL2},{CPL2;CPL3}},[6;4],{[90 75 75 1;0 90 90 0;90 120 120 2],[0 20 20 0;90 90 90 0;0 50 50 0]},{[27;30;55],[10;10;10]},'num_arms',2,'bottom_up','first_single');


% [SGm,SGc,ranges] = SGmanipulator({{CPL;CPL2},{CPL2;CPL3},{PLcircle(5)},{CPL_o}},[6;4;4.5;3],{[90 75 75 1;0 90 90 0;90 80 80 0],[90 20 20 0;0 60 60 0;90 50 50 0],[0 20 20 -1;0 50 50 2],[90 20 20 0;0 90 90 0;90 50 50 0]},{[27;30;55],[10;10;10],[10;20],[10;20;25]},'angles',[-0.5 0 1;-0.5 0 0;0.5 0.8 0;-0.5 0 0.5],'num_arms',4,'bottom_up','optic_mid','single');
% [SGm,SGc,ranges] = SGmanipulator({CPL;CPL2},[6],{[45 75 75 1;-50 90 90 0;33 80 80 0]},{[27;30;55]},'angles',[-0.5 0 1],'bottom_up','single','optic_mid');

[SGm,SGc,ranges] = SGmanipulator({CPL3},[1],{[90 75 75 0;90 75 75 0]},{[27;27]},'angles',[-0.5 0],'bottom_up','crimp','optic_mid');
% [SGm,SGc,ranges] = SGmanipulator({{CPL;CPL2},{CPL;CPL2},{PLcircle(4.2)}},[6;6;3.5],{[90 60 60 1;0 70 70 0;90 120 120 0],[90 60 60 1;0 70 70 0;90 120 120 0],[0 120 45 -1;0 150 150 2]},{[27 4 1.2 1 0.5;30 4 1.0 1 0.5;55 2 0.8 2 0.5],[27 4 1.2 1 0.5;30 4 1.0 1 0.5;55 2 0.8 2 0.5],[20 2 1 1 0.5;40 2 1 0.25 0.5]},'optic_mid','angles',[-0.5 0 0;-0.5 0 0;0 0 0],'num_arms',4,'bottom_up','first_single','hole_radius',0.7,'optic_radius',2.2);
% [SGm,SGc,ranges] = SGmanipulator({{CPL;CPL2},{CPL;CPL2},{PLcircle(3.75)}},[6;6;1.5],{[90 60 60 1;0 70 70 0;90 120 120 0],[90 60 60 1;0 70 70 0;90 120 120 0],[0 120 45 -1;0 150 150 2]},{[27 4 1.2 1 0.5;30 4 1.0 1 0.5;55 2 0.8 2 0.5],[27 4 1.2 1 0.5;30 4 1.0 1 0.5;55 2 0.8 2 0.5],[20 2 1 1 0.5;40 2 1 0.25 0.5]},'optic_mid','angles',[-0.3 0 0;-0.3 0 0;0 0 0],'num_arms',4,'bottom_up','first_single','hole_radius',0.7,'optic_radius',2.2);
% 
% CPL_endoskop = [PLcircle(6);NaN NaN;PLtrans(PLcircle(1.4),[-4 0]);NaN NaN;PLtrans(PLcircle(1.4),[4 0])];
% [SGm,SGc,ranges] = SGmanipulator({CPL_endoskop},[7.5/2],{[45 180 180 2]},{[60]},'angles',[0],'bottom_up','single','tip');

%  CPL_endoskop = [PLcircle(3.5);NaN NaN;PLtrans(PLcircle(0.75),[-2 0])];
% [SGm,SGc,ranges] = SGmanipulator({CPL_endoskop},[1.5],{[45 180 180 2]},{[40 2 0.8 0.25 0.5]},'angles',[-0],'bottom_up','single','tip','hole_radius',0.4);
% 

% [SGm,SGc,ranges] = SGmanipulator({PLcircle(3.3)},[3],{[45 180 180 2]},{[30 2 0.8 0.25 0.5]},'angles',[-0],'bottom_up','single','tip','hole_radius',0.4);

% [SGm,SGc,ranges] = SGmanipulator({PLcircle(35)},[6],{[45 180 180 0]},{[250 2 0.8 0.25 0.5]},'angles',[-0],'bottom_up','single','tip','hole_radius',0.4);



% n=104;[SGm,SGc,ranges] = SGmanipulator(repmat({{PLcircle(5)},{PLcircle(5)}},1,n/2),repelem(4.5,n,1),repmat({[90 20 20 1;0 50 50 2]},1,n),repmat({[10;20],[12;20],[15;20],[12;20]},1,n/4),'angles',repmat([-0.9 0],n,1),'radial','num_arms',4,'bottom_up','single');
SGwriteSTL(SGm,"SGmanipulator",'','y');
% 
% CPL = PLtrans(PLkidney(7,16,pi/5.5),[-11.5 0]);

% CPL_servo = PLtrans0([PLsquare(20.3,40.3);NaN NaN;PLtrans(PLcircle(2.25),[5.1 24.5]);NaN NaN;PLtrans(PLcircle(2.25),[-5.1 24.5]);NaN NaN;PLtrans(PLcircle(2.25),[5.1 -24.5]);NaN NaN;PLtrans(PLcircle(2.25),[-5.1 -24.5])]);
% %CPL_SM40 = []

%StarHornConnector
SG_starhorn_conn = SGofCPLcommand('c 5.6 8 3,move 13 0,enter,c 30,c 35,-,enter,c 16.5,-,h 2,dupr 6,col y');
SG_starhorn_conn = SGofCPLcommand('c 5.6 8 3,move 13 0,enter,c 27,c 35,-,enter,c 16.5,-,h 2,dupr 6,col y');

%SM40BLHorn
SG_connector_SM40 = SGofCPLcommand('c 25,d 3 7 0,d 3 -7 0,d 3 0 7,d 3 0 -7,c 8,h 2,enter,c 19,c 25,h 2.5,rel under 0,cat,col y');

%SM85CLHorn
SG_connector_SM85 = SGofCPLcommand('c 30,d 3 10.5 0,d 3 -10.5 0,d 3 0 10.5,d 3 0 -10.5,c 13.5,h 1,enter,c 26,c 30,h 2.5,rel under 0,cat,col y');

%SM120BLHorn
SG_connector_SM120 =SGofCPLcommand('c 38,d 3 12.5 0,d 3 -12.5 0,d 3 0 12.5,d 3 0 -12.5,c 16,h 3,enter,c 34.5,c 38,h 3,rel under 0,cat,col y');
% SGplot(SG_connector_SM120);

% SGplot(SGservorotor(25,SG_starhorn_conn,[10.5 6 1.5]));
% SGwriteSTL(SGservorotor(25,SG_starhorn_conn,''),"Rotor_SCS40",'','y');
% SGwriteSTL(SGservorotor(25,SG_connector_SM40,[7 4 1.5]),"Rotor_SM40BL",'','y');
% SGwriteSTL(SGservorotor(25,SG_connector_SM40,[7 4 1.5]),"Servorotor",'','y');
% SGwriteSTL(SGservorotor(25,SG_connector_SM85,[10.5 4 1.5]),"Rotor_SM85CL",'','y');
% SGwriteSTL(SGservorotor(25,SG_connector_SM120,[12.5 4 1.5],1),"Rotor_SM120BCL",'','y');
% SGplot(SGmotormountSCS(25,CPL_servo));
% SGwriteSTL(SGmotormountSM40CL(25),"MotormountSM40",'','y'); 
% SGwriteSTL(SGmotormountSM85BL(25),"MotormountSM85BL",'','y');
% SGwriteSTL(SGmotormountSM120BL(25),"MotormountSM120BL",'','y');
% Motormount_SM40CL(25); 
% SGplot(SGtoolmanipulator(CPL_servo,SG_starhorn_conn));
% SGplot(SGtoolmanipulatorSM40());
% SGplot(SGboxinlay);
% SGwriteSTL(SGboxinlay);
% SGplot(SGmanipulatorBox(3,[1,2],2));
% SGplot(SGmanipulatorBoxSimple([60]));
% SGplot(SGpushpullmountSM85BL);
% SGwriteSTL(SGpushpullmountSM40CL(SG_connector_SM40),"SG-pushpullmount");

