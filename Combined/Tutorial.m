%% SGmanipulator
% Function that can be used to generate a range of different singleport
% manipulators

%% Section 1 
% Simplest Manipulatorstructure/ Endoscope manipulator
CPL = PLcircle(5);
SGmanipulator({CPL},[2],[90 60 60 2],[20 2 0.8 2 0.5],'tip');

%% Section 2 Generating Basic Manipulator
% Generating simple Manipulator structure
CPL = PLtrans(PLkidney(7,17,pi/6.5),[-12 0]);
[SGm] = SGmanipulator({PLtrans(PLkidney(7,17,pi/6.5),[-12 0])},6.8,[90;0;90],[27;30;55],'symmetric');
SGwriteSTL(SGm);

