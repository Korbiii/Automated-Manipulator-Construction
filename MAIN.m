clear all; clf;
addpath('Essentials');
path = prefdir; path = strsplit(path, '\');path{7} = 'MATLAB Add-Ons\Toolboxes\SolidGeometry 4.7'; path = strjoin(path(1:7),'\'); addpath(path);
axis equal; hold on;


%% Kidneys
% CPL = PLtrans(PLkidney(7,17,pi/6.5),[-12 0]);
% CPL2 = PLtrans(PLkidney(4,14,pi/6.5),[-9 0]);
% CPL3 = PLtrans(PLkidney(2,12,pi/12),[-7 0]);

% %% Kidneys small
% CPL = PLtrans(PLkidney(9.5,14.5,pi/5.5),[-12.5 0]);
% CPL2 = PLtrans(PLkidney(7,11,pi/4.5),[-9 0]);
% CPL3 = PLtrans(PLkidney(7,11,pi/7),[-9 0]);

%% TEST FOR SYMMETRIC ARMS
% CPL = PLtrans(PLkidney(7,17,pi/6.5),[-12 0]);
% CPL2 = PLtrans(PLkidney(4,14,pi/6.5),[-9 0]);
% [SGm,SGc,ranges] = SGmanipulator({CPL,CPL2},6.8,[90;0;90],[27;30;55],'symmetric');
%%ONE ARM TEST
% CPL = PLtrans(PLkidney(7,17,pi/6.5),[-12 0]);
% CPL2 = PLtrans(PLkidney(4,14,pi/6.5),[-9 0]);
% [SGm,SGc,ranges] = SGmanipulator({CPL;CPL2},[6],{[45 75 75 1;-50 90 90 0;33 80 80 0]},{[27;30;55]},'angles',[-0.5 0 1],'single','optic_mid');
%%TWO ARMS TEST
% CPL = PLtrans(PLkidney(7,17,pi/6.5),[-12 0]);
% CPL2 = PLtrans(PLkidney(4,14,pi/6.5),[-9 0]);
% CPL3 = PLtrans(PLkidney(2,12,pi/12),[-7 0]);
% [SGm,SGc,ranges] = SGmanipulator({{CPL;CPL2;CPL3},{CPL2;CPL3}},[6;4],{[90 75 75 -1;0 90 90 0;90 90 180 0],[0 20 20 -1;90 90 90 0;0 50 50 0]},{[27;30;55],[10;10;10]},'first_single','angles',[-0.5,-0.5,0.5;-0.9,-0.5,0.5]);
%%THREE ARMS TEST
% CPL = PLtrans(PLkidney(7,17,pi/6.5),[-12 0]);
% CPL2 = PLtrans(PLkidney(4,14,pi/6.5),[-9 0]);
% CPL3 = PLtrans(PLkidney(2,12,pi/12),[-7 0]);
% [SGm,SGc,ranges] = SGmanipulator({{CPL;CPL2},{CPL2;CPL3},{PLcircle(5)}},[6;4;3],{[90 75 75 1;0 90 90 0;90 120 120 0],[0 20 20 0;90 90 90 0;0 50 50 0],[90 20 20 0;0 90 90 0;0 50 50 2]},{[27;30;55],[10;10;10],[10;10;20]},'angles',[-0.9 0 1;0.5 0 0;-0.9 0 0.5 ],'num_arms',3,'first_single');
%% FOUR ARM TEST
% CPL = PLtrans(PLkidney(7,17,pi/6.5),[-12 0]);
% CPL2 = PLtrans(PLkidney(4,14,pi/6.5),[-9 0]);
% CPL3 = PLtrans(PLkidney(2,12,pi/12),[-7 0]);
% [SGm,SGc,ranges] = SGmanipulator({{CPL;CPL2},{CPL2;CPL3},{PLcircle(5)},{CPL_o}},[6;4;4.5;3],{[90 75 75 1;0 90 90 0;90 80 80 0],[90 20 20 0;0 60 60 0;90 50 50 0],[0 20 20 -1;0 50 50 2],[90 20 20 0;0 90 90 0;90 50 50 0]},{[27;30;55],[10;10;10],[10;20],[10;20;25]},'angles',[-0.5 0 1;-0.5 0 0;0.5 0.8 0;-0.5 0 0.5],'num_arms',4,'optic_mid','single');

% %% TWO ARMS + CAMERA ARM
% CPL = PLtrans(PLkidney(7,17,pi/6.5),[-12 0]);
% CPL2 = PLtrans(PLkidney(4,14,pi/6.5),[-9 0]);
% [SGm,SGc,ranges] = SGmanipulator({{CPL;CPL2},{CPL;CPL2},{PLcircle(4.5)}},[3;3;4],{[90 60 60 1;0 70 70 0;90 120 120 0],[90 60 60 1;0 70 70 0;90 120 120 0],[0 90 45 -1;0 250 250 2]},{[27 4 1.2 1 0.5;30 4 1.0 0 0.5;55 2 0.8 0 0.5],[27 4 1.2 1 0.5;30 4 1.0 0 0.5;55 2 0.6 0 0.5],[30 2 1 1 0.5;40 2 0.6 0 0.5]},'optic_mid','angles',[-0.5 0 0;-0.5 0 0;0.9 -0.5 0],'first_single','hole_radius',0.75,'optic_radius',2.2);
% [SGm,SGc,ranges] = SGmanipulator({{CPL;CPL2},{CPL;CPL2},{PLcircle(4)}},[6;6;1.5],{[90 60 60 1;0 70 70 0;90 120 120 0],[90 60 60 1;0 70 70 0;90 120 120 0],[0 120 45 -1;0 150 150 2]},{[27 4 1.2 1 0.5;30 4 1.0 1 0.5;55 2 0.8 2 0.5],[27 4 1.2 1 0.5;30 4 1.0 1 0.5;55 2 0.8 2 0.5],[25 2 1 1 0.5;40 2 1 0.25 0.5]},'optic_mid','angles',[-0.3 0 0;-0.3 0 0;0 0 0],'first_single','hole_radius',0.75,'optic_radius',2.2);

% %% TWO ARMS + CAMERA ARM
% CPL = CPLconvexhull(PLtrans(PLkidney(7,17,pi/6.5),[-12 0]));
% CPL2 = PLtrans(PLkidney(4,14,pi/6.5),[-9 0]);
% PL_led = [PLtrans(PLcircle(0.6),[0 4]);NaN NaN;PLtrans(PLcircle(0.6),[0 -4])];
% 
% 
% CPL = PLroundcorners(PLsquare(10,16),[1,2,3,4],[2,8,8,2]);
% CPL2 =  PLroundcorners(PLsquare(10,14),[1,2,3,4],[2,7,7,2]);
% 
% 
% CPL = PLroundcorners(PLsquare(9.5,15),[1,2,3,4],[2,7,7,2]);
% CPL2 =  PLroundcorners(PLsquare(9.5,15),[1,2,3,4],[2,7,7,2]);
% % CPL = [CPL;NaN NaN;PL_led];
% % CPL2 = [CPL2;NaN NaN;PL_led];
% 
% PL_led = [PLtrans(PLcircle(0.6),[-4 -3]);NaN NaN;PLtrans(PLcircle(0.6),[4 -3])];
% CPL_camera =  PLroundcorners(PLsquare(11,9),[1,2,3,4],[2,2,5,5]);
% % CPL_camera = [CPL_camera;NaN NaN;PL_led];
% 
% [SGm,SGc,ranges,fc,phi] = SGmanipulator({{CPL;CPL2},{CPL;CPL2},{CPL_camera}},[6;6;4],{[90 60 60 1;0 80 70 0;90 160 160 0],[90 60 60 1;0 70 70 0;90 160 160 0],[0 90 45 -1;0 250 250 2]},{[27 4 1.2 1 0.5;30 4 1.0 0 0.5;55 2 0.8 2 0.5],[27 4 1.2 1 0.5;30 4 1.0 0 0.5;55 2 0.6 2 0.5],[30 2 1 2 0.5;40 2 0.6 1 0.5]},'angles',[-0.2 0 0;-0.2 0 0;0 -0 0],'first_single','hole_radius',0.75,'length',15);
% [SGm,SGc,ranges] = SGmanipulator({{CPL;CPL2},{CPL;CPL2},{PLcircle(4)}},[6;6;1.5],{[90 60 60 1;0 70 70 0;90 120 120 0],[90 60 60 1;0 70 70 0;90 120 120 0],[0 120 45 -1;0 150 150 2]},{[27 4 1.2 1 0.5;30 4 1.0 1 0.5;55 2 0.8 2 0.5],[27 4 1.2 1 0.5;30 4 1.0 1 0.5;55 2 0.8 2 0.5],[25 2 1 1 0.5;40 2 1 0.25 0.5]},'optic_mid','angles',[-0.3 0 0;-0.3 0 0;0 0 0],'first_single','hole_radius',0.75,'optic_radius',2.2);
% 
% SGwriteSTL(SGc{1},"150mm",'','y')

%% ENDOSKOPTESTS
% CPL_endoskop = [PLcircle(8.5);NaN NaN;PLtrans(PLcircle(1.4),[-6 0]);NaN NaN;PLtrans(PLcircle(1.4),[6 0])];
% [SGm,SGc,ranges] = SGmanipulator({CPL_endoskop},[7.5],{[45 500 500 2]},{[120 2 0.8 2 0.5]},'angles',[0],'single','tip','torsion');

%  CPL_endoskop = [PLcircle(3.5);NaN NaN;PLtrans(PLcircle(0.75),[-2 0])];
% [SGm,SGc,ranges] = SGmanipulator({CPL_endoskop},[1.5],{[45 360 360 2]},{[80 2 0.5 0.25 0.5]},'angles',[-0],'single','tip','hole_radius',0.4);

% [SGm,SGc,ranges] = SGmanipulator({PLcircle(3.5)},[3],{[45 360 360 2]},{[60 2 0.5 0.25 0.5]},'angles',[-0],'single','tip','hole_radius',0.4);

% 
% CPL_G = load('C:\Users\Korbi\Desktop\CPL_G_7mm.mat');
% CPL_G = CPL_G.CPL_G;
% CPL_G = CPLbool('+',CPL_G,PLcircle(3.8));
% CPL_G_in = CPLselectinout(CPL_G,1);
% CPL_G_out = CPLselectinout(CPL_G,0);
% [SGm,SGc,ranges,fc,phi] = SGmanipulator({flip(CPL_G)},[7],{[45 270 270 2]},{[40 2 0.6 0 0.5]},'angles',[1],'tip');




% [SGm,SGc,ranges] = SGmanipulator({flip(CPL_G)},[7],{[-33 0 0 0;45 270 270 2]},{[4 2 0.6 0 0.5;40 2 0.6 1.5 0.5]},'angles',[0 0],'tip');
% 
PLtm = load('C:\Users\Korbi\Desktop\PLtm.mat');
PLtm = PLtrans0(PLtm.PLtm);
PLtm = PLtrans(PLtm,[-0.3 0]);
PLtm = CPLbool('+',PLtm,PLcircle(1.5));
[SGm,SGc,ranges] = SGmanipulator({flip(PLtm)},[3],{[45 90 90 2]},{[20 2 0.6 0.5 0.5]},'angles',[0],'single','tip');

% CPL_G = load('C:\Users\Korbi\Desktop\CPL_G6.mat');
% CPL_G = CPL_G.CPL_G;
% CPL_G = CPLbool('+',CPL_G,PLcircle(3.8));
% CPL_G_in = CPLselectinout(CPL_G,1);
% CPL_G_out = CPLselectinout(CPL_G,0);
% [SGm,SGc,ranges] = SGmanipulator({flip(CPL_G)},[7],{[45 270 270 2]},{[40 2 0.8 1 0.5]},'angles',[0]);



%% THE KRAKEN
% n=104;[SGm,SGc,ranges] = SGmanipulator(repmat({{PLcircle(5)},{PLcircle(5)}},1,n/2),repelem(4.5,n,1),repmat({[90 20 20 1;0 50 50 2]},1,n),repmat({[10;20],[12;20],[15;20],[12;20]},1,n/4),'angles',repmat([-0.9 0],n,1),'radial','num_arms',4,'bottom_up','single');

% 
SGwriteSTL(SGm,"SGmanipulator",'','y');




%StarHornConnector
SG_starhorn_conn = SGofCPLcommand('c 5.6 8 3,move 13 0,enter,c 27,c 35,-,enter,c 16.5,-,h 2,dupr 6,col y');

%SM40BLHorn
SG_connector_SM40 = SGofCPLcommand('c 25,d 3 7 0,d 3 -7 0,d 3 0 7,d 3 0 -7,c 8,h 2,enter,c 19,c 25,h 2.5,rel under 0,cat,col y');

%SM85CLHorn
SG_connector_SM85 = SGofCPLcommand('c 30,d 3 10.5 0,d 3 -10.5 0,d 3 0 10.5,d 3 0 -10.5,c 13.5,h 1,enter,c 26,c 30,h 2.5,rel under 0,cat,col y');

%SM120BLHorn
SG_connector_SM120 =SGofCPLcommand('c 38,d 3 12.5 0,d 3 -12.5 0,d 3 0 12.5,d 3 0 -12.5,c 16,h 3,enter,c 34.5,c 38,h 3,rel under 0,cat,col y');
% SGplot(SG_connector_SM120);

% SGplot(SGmanipulatorBox([132 80 80]));



% SGwriteSTL(SGmotormountSM85BL(25),"MotormountSM85BL",'','y');
% SGwriteSTL(SGmotormountSM120BL(25),"MotormountSM120BL",'','y');
% SGplot(SGtoolmanipulatorSM40());
% SGplot(SGpushpullmountSM85BL);
% SGplot(SGpushpullmountSM40CL(SG_connector_SM40));

% SGwriteSTL(SGpushpullmountSM40CL(SG_connector_SM40),"SG-pushpullmount");

