%% SGmanipulator
% Function that can be used to generate a range of different singleport
% manipulators

%% Overview over input arguments
% Simplest Manipulatorstructure
SGmanipulator({PLcircle(5)},...     CPL to define contour of arm
                          3,...     Diameter for tool channel at [0 0]
               [90 60 60 2],...     [xy-Plane angle, Deflection angle 1, Deflection angle 2, Section flag (2 = alternating section)]
           [20 2 0.8 2 0.5],...     [Section length, Element height, Hinge width, Hinge length, hinge radius] 
                     'tip');        ...Flag to not generate shaft

%% Generating Basic Manipulator
% Generating simple Manipulator structure
CPL = PLtrans(PLkidney(7,17,pi/6.5),[-12 0]);
SGmanipulator({CPL},...             CPL to define contour of arm
                6.8,...             Diameter for tool channel at [0 0]
          [90;0;90],...             xy-plane angles.
         [27;30;55],...             Section lengths.
       'symmetric');                ...Flag to generate two identical arms.
   
%% Three arm Manipulator
% 
CPL = PLtrans(PLkidney(7,17,pi/6.5),[-12 0]);
CPL2 = PLtrans(PLkidney(4,14,pi/6.5),[-9 0]);
CPL3 = PLtrans(PLkidney(2,12,pi/12),[-7 0]);

                SGmanipulator({{CPL;CPL2},...       Cell list for each arm.
                              {CPL2;CPL3},...       Number of sections is based on angle inputs.
                           {PLcircle(5)}},...       Putting less CPLs than number of sections results in the last CPL being used for remaining sections.
     ...
                                  [6;4;3],...       Array of inner diameters for each arm
     ...
     {[90 75 75 1;0 90 90 0;90 120 120 0],...       Setting the angle flag to -1 or 1 optimizes the hinge in one direction 
         [0 20 20 0;90 90 90 0;0 50 50 0],...       
        [90 20 20 0;0 90 90 0;0 50 50 2]},...
     ...
                        {[27 2;30 4;55 2],...       Length and angle parameters can be specified to different degrees.
                         [10 3;10 2;10 3],...       
                        [10 4;10 2;20 2]},...
     ...
                           'first_single',...       Flag results in only one bowden cable channel for the first sections.  
                                'torsion',...       Flag specifies if hinges are optimized for torsion or for leverage.
                                 'radial',...       Flag arranges arms in a circle around [0 0]. This has to be used if nuber of arms is greater than 4.
 'angles',[-0.9 0 1;0.5 0 0;-0.9 0 0.5 ]);          ...Flag is used to deflect the sections in SGTchain in percent of maximum angle.

