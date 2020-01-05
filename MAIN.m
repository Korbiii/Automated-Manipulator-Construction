clear; clf;
addpath('Essentials');
path = prefdir; path = strsplit(path, '\');path{7} = 'MATLAB Add-Ons\Toolboxes\SolidGeometry 4.7'; path = strjoin(path(1:7),'\'); addpath(path);
% CPL = PLtrans(PLkidney(7,16,pi/5.5),[-11.5 0]);
CPL = PLtrans(PLkidney(7,17,pi/6.5),[-12 0]);
% SGwriteSTL(SGmanipulator(CPL,3.4,[90 75 4.2;0 90 0;90 270 0],0,1),"SGmanipulator",'','y');

CPL_servo = PLtrans0([PLsquare(20.3,40.3);NaN NaN;PLtrans(PLcircle(2.25),[5.1 24.5]);NaN NaN;PLtrans(PLcircle(2.25),[-5.1 24.5]);NaN NaN;PLtrans(PLcircle(2.25),[5.1 -24.5]);NaN NaN;PLtrans(PLcircle(2.25),[-5.1 -24.5])]);
%CPL_SM40 = []

%StarHornConnector
SG_starhorn_conn = SGofCPLcommand('c 5.6 8 3,move 13 0,enter,c 30,c 35,-,enter,c 16.5,-,h 2,dupr 6,col y');
SG_starhorn_conn = SGofCPLcommand('c 5.6 8 3,move 13 0,enter,c 27,c 35,-,enter,c 16.5,-,h 2,dupr 6,col y');

%SM40BLHorn
SG_connector_SM40 = SGofCPLcommand('c 25,d 3 7 0,d 3 -7 0,d 3 0 7,d 3 0 -7,c 8,h 2,enter,c 19,c 25,h 2.5,rel under 0,cat,col y');

%SM85CLHorn
SG_connector_SM85 = SGofCPLcommand('c 30,d 3 10.5 0,d 3 -10.5 0,d 3 0 10.5,d 3 0 -10.5,c 13.5,h 1,enter,c 26,c 30,h 2.5,rel under 0,cat,col y');

% % SGplot(SGservorotor(25,SG_starhorn_conn,[10.5 6 1.5]));
% SGwriteSTL(SGservorotor(25,SG_starhorn_conn,''),"Rotor_SCS40",'','y');
% SGwriteSTL(SGservorotor(25,SG_connector_SM40,[7 4 1.5]),"Rotor_SM40BL",'','y');
% SGwriteSTL(SGservorotor(25,SG_connector_SM40,[7 4 1.5]),"Servorotor",'','y');
% % SGwriteSTL(SGservorotor(25,SG_connector_SM85,[10.5 4 1.5]),"Rotor_SM85CL",'','y');
% SGplot(SGmotormountSCS(25,CPL_servo));
% % SGwriteSTL(SGmotormountSM40CL(25),"MotormountSM40",'','y'); 
% SGwriteSTL(SGmotormountSM85BL(25),"MotormountSM85BL",'','y');
% Motormount_SM40CL(25); 
% SGplot(SGtoolmanipulator(CPL_servo,SG_starhorn_conn));
% SGplot(SGtoolmanipulatorSM40(CPL_servo,SG_connector_SM40));
% SGplot(SGboxinlay);
% SGwriteSTL(SGboxinlay);
SGplot(SGmanipulatorBox(2,[2]));
% SGwriteSTL(SGmanipulatorBox(3,3));

