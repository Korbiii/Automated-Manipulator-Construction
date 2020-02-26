clear; clf;
addpath('Essentials');
path = prefdir; path = strsplit(path, '\');path{7} = 'MATLAB Add-Ons\Toolboxes\SolidGeometry 4.7'; path = strjoin(path(1:7),'\'); addpath(path);
axis equal; hold on;


% %% Kidneys
% CPL = PLtrans(PLkidney(7,17,pi/6.5),[-12 0]);
% CPL2 = PLtrans(PLkidney(4,14,pi/6.5),[-9 0]);
% CPL3 = PLtrans(PLkidney(2,12,pi/12),[-7 0]);

%% Ovals
% CPL = PLtrans0(PLtransR(PLcircleoval(5.2,'',5),rot(pi/2)));
% CPL2 = PLtrans0(PLtransR(PLcircleoval(5.2,'',4),rot(pi/2)));
% CPL3 = PLtrans0(PLtransR(PLcircleoval(5.2,'',2),rot(pi/2)));

% %% Ovals small
% CPL = PLtrans0(PLtransR(PLcircleoval(3.5,'',5),rot(pi/2)));
% CPL2 = PLtrans0(PLtransR(PLcircleoval(3,'',4),rot(pi/2)));
% CPL3 = PLtrans0(PLtransR(PLcircleoval(3,'',2),rot(pi/2)));

% SGwriteSTL(SGmanipulator({CPL CPL2},3.4,[90 75 4.2;0 90 0;90 270 0]),"SGmanipulator",'','y');

% SGm = SGmanipulator({CPL},3.4,[90 75 75 1;0 90 90 0;90 10 270 0],[27 4 1.2;30 4 1.0;55 2 0.8],'first_single'); %% Aktuellen Manipulator ausgeben
% [SGm,SGc] = SGmanipulator({CPL CPL2},3.4,[90 75 75 1;0 90 90 0;90 10 270 0],[27 4 1.2;30 4 1.0;55 2 0.8],'single'); %% Push Pull
% SGm = SGmanipulator({CPL CPL2},3.4,[90 75 75 1;0 90 90 0;90 10 270 0;0 90 90 1],[27 4 1.2;30 4 1.0;55 2 0.8;20 2 0.6],'single'); %% Push Pull Extra Long
% SGm = SGmanipulator({CPL CPL2 CPL3},1.5,[90 75 75 1;0 90 90 0;90 90 180 -1],[12 1 0.8;12 1 0.8;24 1 0.8],'single'); %% MiniPushpull


% SGwriteSTL(SGm,"SGmanipulator",'','y');
% 
% CPL = PLtrans(PLkidney(7,16,pi/5.5),[-11.5 0]);

CPL_servo = PLtrans0([PLsquare(20.3,40.3);NaN NaN;PLtrans(PLcircle(2.25),[5.1 24.5]);NaN NaN;PLtrans(PLcircle(2.25),[-5.1 24.5]);NaN NaN;PLtrans(PLcircle(2.25),[5.1 -24.5]);NaN NaN;PLtrans(PLcircle(2.25),[-5.1 -24.5])]);
%CPL_SM40 = []

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

% % SGplot(SGservorotor(25,SG_starhorn_conn,[10.5 6 1.5]));
% SGwriteSTL(SGservorotor(25,SG_starhorn_conn,''),"Rotor_SCS40",'','y');
% SGwriteSTL(SGservorotor(25,SG_connector_SM40,[7 4 1.5]),"Rotor_SM40BL",'','y');
% SGwriteSTL(SGservorotor(25,SG_connector_SM40,[7 4 1.5]),"Servorotor",'','y');
% SGwriteSTL(SGservorotor(25,SG_connector_SM85,[10.5 4 1.5]),"Rotor_SM85CL",'','y');
% SGwriteSTL(SGservorotor(25,SG_connector_SM120,[12.5 4 1.5]),"Rotor_SM120BCL",'','y');
% SGplot(SGmotormountSCS(25,CPL_servo));
% % SGwriteSTL(SGmotormountSM40CL(25),"MotormountSM40",'','y'); 
% SGwriteSTL(SGmotormountSM85BL(25),"MotormountSM85BL",'','y');
% SGwriteSTL(SGmotormountSM120BL(25),"MotormountSM120BL",'','y');
% Motormount_SM40CL(25); 
% SGplot(SGtoolmanipulator(CPL_servo,SG_starhorn_conn));
SGplot(SGtoolmanipulatorSM40(CPL_servo,SG_connector_SM40));
% SGplot(SGboxinlay);
% SGwriteSTL(SGboxinlay);
% SGplot(SGmanipulatorBox(3,[1,2],2));
% SGplot(SGmanipulatorBox(3,[1 1 1]));
% SGplot(SGpushpullmountSM85BL);
% SGwriteSTL(SGpushpullmountSM40CL(SG_connector_SM40),"SG-pushpullmount");

