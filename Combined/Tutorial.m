%% Tutorial XX: Generate Single Port Manipulators

%% Motivation for this tutorial: 

%% Introduced Function:
% * SGmanipulator - Generates a single port manipulator.

%% Generating simplest Manipulator
% Simplest Manipulator structure
SGmanipulator({PLcircle(5)},...     CPL to define contour of arm
                          3,...     Diameter for tool channel at [0 0]
                          0,...     Orientation of hinge
                         15,...     Section length 
                     'tip');        ...Flag to not generate shaft
view(90,30);

%% Orientation angle
% Orientation angle changes the orientation of the rotation axis of the hinge
SGmanipulator({PLcircle(5)},...     
                          3,...     
                         30,...     Previous Example with 30 degree orientation angle. Value range: -180 to 180.
                         15,...     
                     'tip');        
view(90,30);

%% Deflection angles
% Besides orientation angle the deflection angles of a section can be specified.
SGmanipulator({PLcircle(5)},...     
                          3,...     
                  [0 45 90],...     [Deflection angle 1, Deflection angle 2]    
                         15,...     
                     'tip');        
view(90,30);


%% Optimizing for one deflection direction
% The hinge can be optimized for one rotation direction.
SGmanipulator({PLcircle(5)},...     
                          3,...     
                [0 45 90 1],...     Angle flag set to 1 optimizes hinge position for second deflection angle. First angle gets ignored. -1 for the first angle.
                         15,...     
                     'tip');        
view(90,0);

%% Alternating Sections.
% The fourth angle parameter set to 2 generates alternating sections.
SGmanipulator({PLcircle(5)},...    
                          3,...     
                [0 90 90 2],...     Angle flag set to 2 generates an alternating section. Second angle is offset 90 degrees to the original angle. 
                         15,...     
                     'tip');        
view(90,30);

%% Element Height
% With the argument after section length the element height can be altered.
SGmanipulator({PLcircle(5)},...    
                          3,...     
                          0,...      
                     [15 4],...     Element height set to 4. Default is 2. 
                     'tip');        
view(90,0); delete(findobj('type', 'patch','FaceAlpha',0.3));

%% Hinge width
% The second argument behind section lengths specifies the hinge width.
SGmanipulator({PLcircle(5)},...    
                          3,...     
                          0,...      
                   [15 4 2],...     Hinge width set to 2. Default is 0.6
                     'tip');        
view(90,0); delete(findobj('type', 'patch','FaceAlpha',0.3));

%% Hinge length
% The third argument behind section lengths specifies the hinge length.
SGmanipulator({PLcircle(5)},...    
                          3,...     
                          0,...      
                 [15 4 2 1],...     Hinge length set to 2. Default is 2. Setting this value to 0 projects the hinge over the whole element.
                     'tip');        
view(0,0); delete(findobj('type', 'patch','FaceAlpha',0.3));

%% Hinge Diameter
% The fourth argument behind section lengths specifies the hinge diameter and therefor its height.
SGmanipulator({PLcircle(5)},...    
                          3,...     
                          0,...      
               [15 4 2 1 2],...     Hinge Diamater set to 2. Default is 0.5.
                     'tip');        
view(90,0); delete(findobj('type', 'patch','FaceAlpha',0.3));

%% Shaft length
% Specify the length of the shaft after the element for the crimps.
SGmanipulator({PLcircle(5)},...     
                          3,...     
                          0,...     
                         15,...     
                   'length',5);        ...Parameter to set shaft length
view(90,0);

%% Flexible Shaft
% Specify the length of the shaft after the element for the crimps.
SGmanipulator({PLcircle(5)},...     
                          3,...     
                          0,...     
                         15,...
                   'flex',0,...        Flex flag generates a flexible shaft. An angle value has to be given for pre deflecting the flexible shaft.
                   'length',5);        ...Parameter to set shaft length
view(90,0);



