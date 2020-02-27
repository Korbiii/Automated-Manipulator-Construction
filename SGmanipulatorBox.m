%%  [SG] = SGmanipulatorBox(dofs,varargin)
%	=== INPUT PARAMETERS ===
%	dofs:       number of degress of freedom
%   power:      1xn Vector, 1 = 4Nm 2 = 8Nm 3 = 12Nm
%	=== OUTPUT RESULTS ======
%
function [SG] = SGmanipulatorBox(dofs,power)
%% Variables defining

rotor_radius = 20;

SM40BL_dimensions = [46.5,28.5,34.5]+0.5;   %depth,widht,height
SM85CL_dimensions = [62,34,47]+0.5;         %depth,widht,height
SM120BL_dimensions = [78 43 65.5]+0.5;      %depth,widht,height
SM40BL_axle_distance = 35.25;
SM85CL_axle_distance = 47;
SM120BL_axle_distance = 56.5;
SM40BL_holes = [2,2,16,12];                 %x_num,y_num,x_dis,y_dis
SM85CL_holes = [2,3,28,28];                 %x_num,y_num,x_dis,y_dis
SM120BL_holes = [2,3,35,35];                %x_num,y_num,x_dis,y_dis

top_plate_holes = [13.5,16,20];
servo_d = {SM40BL_dimensions,SM85CL_dimensions,SM120BL_dimensions};
servo_ax = [SM40BL_axle_distance,SM85CL_axle_distance,SM120BL_axle_distance];

box_wall_thickness = 5;

%%Bottom
SG_servo_guides = [];
SG_side_holes = [];
SG_sm40_holes = SGofCPLy(CPLcopypattern(PLcircle(1.5),SM40BL_holes(1:2),SM40BL_holes(3:4)),20);
PL_sm85_holes = PLtrans0(CPLcopypattern(PLcircle(1.5),SM85CL_holes(1:2),SM85CL_holes(3:4)));
PL_sm120_holes = PLtrans0(CPLcopypattern(PLcircle(1.5),SM120BL_holes(1:2),SM120BL_holes(3:4)));
PL_sm85_holes = PLtrans(PL_sm85_holes,[0 -28-3]);
PL_sm120_holes = PLtrans(PL_sm120_holes,[0 -35-4]);
SG_text = {SGoftext("SM40BL",[20 8 1]),SGoftext("SM85CL",[20 8 1]),SGoftext("SM120BL",[20 8 1])};
servo_hol = {PL_sm85_holes,PL_sm120_holes};

sizex = 0;
box_height = servo_d{max(power)}(3);

dists = 0;
for i=1:dofs
    PL_side = PLtrans(CPLbool('-',PLsquare(10,servo_d{power(i)}(1)+5),PLtrans(PLsquare(5,servo_d{power(i)}(1)+5),[2.5 5])),[-servo_d{power(i)}(2)/2 0]);  
    PL_guide = [PL_side;NaN NaN;VLswapX(PL_side)];
    SG_guide = SGofCPLz(PL_guide,15);
    
    sizex_old = sizex;
    [sizex,sizey,~,~,~,~] = sizeVL(SG_guide);    
    if isempty(SG_servo_guides)
        SG_servo_guides = SGtrans(SG_guide,[0 -sizey/2 0]);
        sizex_first = sizex/2;
    else
        dists = [dists dists(i-1) + max(sizex_old/2,rotor_radius) + max(sizex/2,rotor_radius)+20];
        SG_guide = SGtransrelSG(SG_guide,SG_servo_guides,'transx',-dists(i),'alignback');
        SG_servo_guides = SGcat(SG_servo_guides,SG_guide);
    end
       
    %ScrewHoles
    if power(i) == 1       
        SG_side_holes = SGcat(SG_side_holes,SGtransrelSG(SG_sm40_holes,SG_guide,'centerx','aligntop',-9.5,'alignback',-15));
    end   
end
[sizex,sizey,~,~,~,~] = sizeVL(SG_servo_guides);
PL_main_frame = CPLbool('-',PLsquare(sizex+30,sizey+20),PLsquare(sizex+20,sizey+10));
PL_main_frame = PLroundcorners(PL_main_frame,2,10);
SG_main_frame = SGofCPLz(PL_main_frame,box_height+2);
SG_main_frame = SGtransrelSG(SG_main_frame,SG_servo_guides,'centerx','alignback',5,'aligntop');
SG_main_frame = SGboolh('-',SG_main_frame,SG_side_holes);

PL_main_frame_top = PLtrans(PLroundcorners(PLsquare(sizex+30,sizey+20),4,10),[(sizex+20)/-2,(sizey+10)/-2]);
for k=1:dofs    
    PL_main_frame_top = [PL_main_frame_top;NaN NaN;PLtrans(PLcircle(top_plate_holes(power(k))),[-dists(k)-(box_wall_thickness*2)-sizex_first;-servo_ax(power(k))])];
    if power(k)>1
        PL_main_frame_top = [PL_main_frame_top;NaN NaN;PLtrans(servo_hol{power(k)-1},[-dists(k)-(box_wall_thickness*2)-sizex_first;0])];
    end
end
SG_main_frame_top = SGofCPLz(PL_main_frame_top,5);
SG_main_frame_top = SGtransrelSG(SG_main_frame_top,SG_main_frame,'ontop','alignright','alignback');


PL_positioning = [PLsquare(sizex+15,sizey+10);NaN NaN;PLsquare(sizex+9,sizey+4)];
PL_positioning = CPLbool('-',PL_positioning,PLsquare(sizex-20,sizey*2));
PL_positioning = CPLbool('-',PL_positioning,PLsquare(sizex*2,sizey-20));
SG_positioning = SGofCPLz(PL_positioning,5);
SG_positioning = SGtransrelSG(SG_positioning,SG_main_frame_top,'centerx','centery','ontop');

PL_back_plate = CPLbool('+',PLsquare(30,box_height-10.5),PLtrans(PLsquare(40,13),[5 box_height/2+1.25]));
PL_back_plate = CPLbool('-',PL_back_plate,PLtrans(PLcircle(8),[5 -10]));
PL_back_plate = PLroundcorners(PL_back_plate,2,20);
SG_back_plate = SGofCPLx(PL_back_plate,5);
SG_back_plate = SGtransrelSG(SG_back_plate,SG_main_frame,'alignbottom','alignright','infront',-10);

% PL_guide = CPLbool('-',PLsquare(10,7),PLtrans(PLsquare(5),[2.5 -1.75]));
% SG_guide = SGofCPLx(PL_guide,170);

% PL_tool_move_connection = CPLbool('-',PLsquare(120,39),PLtrans(PLsquare(100,30),[0 -5]));
% PL_tool_move_connection = PLroundcorners(PL_tool_move_connection,[2,3,6,7],[10 10 5 5]);
% PL_tool_move_connection = CPLbool('-',PL_tool_move_connection,PLtrans(PLsquare(56.7,10),[-10 14.5]));
% SG_tool_mover_connection = SGofCPLx(PL_tool_move_connection,28.5);
% SG_tool_mover_connection = SGtransrelSG(SG_tool_mover_connection,SG_main_frame,'alignbottom','right','alignback');
% 
% SG_top_cover_side = SGtransrelSG(SGbox([57,66.5,5]),SG_tool_mover_connection,'aligntop',30,'alignleft','centery',-10);
% PL_side_covers = [PLsquare(42.75,25);NaN NaN;PLtrans(CPLcopypattern(PLcircle(1.5),[1 2],[0 12]),[13.375 -9.5])];
% SG_side_covers = SGofCPLy(PL_side_covers,4.7);
% SG_side_covers = SGtransrelSG(SG_side_covers,SG_top_cover_side,'under','alignleft','alignback');
% SG_side_covers_2 = SGtransrelSG(SG_side_covers,SG_top_cover_side,'under','alignleft','alignfront');
% SG_side_covers = SGcat(SG_side_covers,SG_side_covers_2);

PL_nut_holder = [PLsquare(12);NaN NaN;PLcircle(4.1,6)];
PL_nut_holder_base = [PLsquare(12);NaN NaN;PLcircle(2.5)];
SG_nut_holder = SGofCPLz(PL_nut_holder,6);
SG_nut_holder_base = SGofCPLz(PL_nut_holder_base,3);
SG_nut_holder = SGstack('z',SG_nut_holder_base,SG_nut_holder);
SG_nut_holder = SGpatternXY(SG_nut_holder,2,2,sizex+8,sizey-1);
SG_nut_holder = SGtransrelSG(SG_nut_holder,SG_main_frame,'centerx','centery','alignbottom');
SG_texts = {};
for i=1:dofs    
    SG_texts{end+1} = SGtransrelSG(SG_text{power(i)},SG_main_frame,'rotx',pi/2,'rotz',pi,'behind','transx',10,'alignbottom',-2.5,'transx',-dists(i));
end



%% Rotors
SG_rotors = {};

SG_sm40_rotor = SGofCPLcommand('c 25,d 3 7 0,d 3 -7 0,d 3 0 7,d 3 0 -7,c 8,h 7,enter,c 19,c 25,h 2.5,rel under 0,cat,col y');
SG_cover_rotor = SGofCPLz([PLcircle(17);NaN NaN;PLcircle(1.5);NaN NaN;CPLcopyradial(PLcircle(1.5),7,4)],2);
PL_rotor_conn_bot = [PLsquare(22);NaN NaN;PLcircle(1.5);NaN NaN;CPLcopyradial(PLcircle(1.5),7,4)];
SG_rotor_conn_bot = SGofCPLz(PL_rotor_conn_bot,3.5);
PL_rotor_conn_top = [PLsquare(22);NaN NaN;PLcircle(3);NaN NaN;CPLcopyradial(PLcircle(3),7,4)];
SG_rotor_conn_top = SGofCPLz(PL_rotor_conn_top,3.5);
SG_sm40_rotor = SGstack('z',SG_sm40_rotor,SG_cover_rotor,SG_rotor_conn_bot,SG_rotor_conn_top);

SG_sm85_rotor = SGofCPLcommand('c 30,d 3 10.5 0,d 3 -10.5 0,d 3 0 10.5,d 3 0 -10.5,c 13.5,h 7,enter,c 26,c 30,h 2.5,rel under 0,cat,col y');
SG_cover_rotor_85 = SGofCPLz([PLcircle(17);NaN NaN;PLcircle(1.5);NaN NaN;CPLcopyradial(PLcircle(1.5),10.5,4)],2);
PL_rotor_conn_bot_85 = [PLcircle(17,5);NaN NaN;PLcircle(1.5);NaN NaN;CPLcopyradial(PLcircle(1.5),10.5,4)];
SG_rotor_conn_bot_85 = SGofCPLz(PL_rotor_conn_bot_85,3.5);
PL_rotor_conn_top_85 = [PLcircle(17,5);NaN NaN;PLcircle(3);NaN NaN;CPLcopyradial(PLcircle(3),10.5,4)];
SG_rotor_conn_top_85 = SGofCPLz(PL_rotor_conn_top_85,3.5);
SG_sm85_rotor = SGstack('z',SG_sm85_rotor,SG_cover_rotor_85,SG_rotor_conn_bot_85,SG_rotor_conn_top_85);

SG_sm120_rotor = SGofCPLcommand('c 38,d 3 12.5 0,d 3 -12.5 0,d 3 0 12.5,d 3 0 -12.5,c 16,h 7,enter,c 34.5,c 38,h 2.5,rel under 0,cat,col y');
SG_cover_rotor_120 = SGofCPLz([PLcircle(21);NaN NaN;PLcircle(1.5);NaN NaN;CPLcopyradial(PLcircle(1.5),12.5,4)],2);
PL_rotor_conn_bot_120 = [PLcircle(21,5);NaN NaN;PLcircle(1.5);NaN NaN;CPLcopyradial(PLcircle(1.5),12.5,4)];
SG_rotor_conn_bot_120 = SGofCPLz(PL_rotor_conn_bot_120,3.5);
PL_rotor_conn_top_120 = [PLcircle(21,5);NaN NaN;PLcircle(3);NaN NaN;CPLcopyradial(PLcircle(3),12.5,4)];
SG_rotor_conn_top_120 = SGofCPLz(PL_rotor_conn_top_120,3.5);
SG_sm120_rotor = SGstack('z',SG_sm120_rotor,SG_cover_rotor_120,SG_rotor_conn_bot_120,SG_rotor_conn_top_120);


SG_rotors_o = {SG_sm40_rotor,SG_sm85_rotor,SG_sm120_rotor};

for i=1:dofs
        SG_rotors{end+1} = SGtransrelSG(SG_rotors_o{power(i)},SG_main_frame_top,'ontop',-8,'transy',-servo_ax(power(i)),'transx',-dists(i));
end


%% Top plate
SG_rotors_top_plate = {};
SG_rotor_top = SGtrans(SGservorotor(rotor_radius,'',''),TofR(rotz(110)));

PL_top_plate = PLsquare(sizex+30,sizey+20);
PL_top_plate = PLtrans(PL_top_plate,[(sizex+20)/-2,(sizey+20)/-2]);
for k=1:dofs    
   PL_top_plate = [PL_top_plate;NaN NaN;PLtrans(PLcircle(16),[-dists(k)-(2*box_wall_thickness)-sizex_first;-servo_ax(power(k))-box_wall_thickness])];
end
SG_top_plate = SGofCPLz(PLroundcorners(PL_top_plate,4,10),5);
SG_top_plate = SGtransrelSG(SG_top_plate,SG_main_frame,'ontop',18,'alignright','alignback');

PL_top_frame = CPLbool('-',PLsquare(sizex+30,sizey+20),PLsquare(sizex+20,sizey+10));
PL_top_frame =  PLroundcorners(PL_top_frame,2,10);
SG_top_frame = SGofCPLz(PL_top_frame,13);
SG_top_frame = SGtransrelSG(SG_top_frame,SG_top_plate,'under','alignright','alignback');

%top sm40
SG_connector = SGofCPLz([PLcircle(17);NaN NaN;PLsquare(22.1)],6.5);
SG_connection_top_top = SGofCPLz(PLcircle(17),2);
SG_connection_top_through = SGofCPLz(PLcircle(10),6);
SG_connection_top = SGstack('z',SG_connector,SG_connection_top_top,SG_connection_top_through,SG_rotor_top);

%top sm85
SG_connector_85 = SGofCPLz([PLcircle(17.1,5);NaN NaN;PLcircle(19)],6.5);
SG_connection_top_top_85 = SGofCPLz(PLcircle(19),2);
SG_connection_top_through_85 = SGofCPLz(PLcircle(10),6);
SG_connection_top_85 = SGstack('z',SG_connector_85,SG_connection_top_top_85,SG_connection_top_through_85,SG_rotor_top);

%top sm120
SG_connector_120 = SGofCPLz([PLcircle(21.1,5);NaN NaN;PLcircle(23)],6.5);
SG_connection_top_120 = SGofCPLz(PLcircle(21),2);
SG_connection_top_through_120 = SGofCPLz(PLcircle(10),6);
SG_connection_120 = SGstack('z',SG_connector_120,SG_connection_top_120,SG_connection_top_through_120,SG_rotor_top);

SG_connections = {SG_connection_top,SG_connection_top_85,SG_connection_120};

for i=1:dofs
        SG_rotors_top_plate{end+1} = SGtransrelSG(SG_connections{power(i)},SG_top_plate,'ontop',-14,'transy',-servo_ax(power(i)),'transx',-dists(i));
end

SG_tensioners = {};
SG_teeth_cut = {};
PL_tensionerblock = [PLtrans(PLsquare((rotor_radius+5)*2,8),[0 1]);NaN NaN;PLtrans(PLcircle(1.5),[rotor_radius 0]);NaN NaN;PLtrans(PLcircle(1.5),[-rotor_radius 0])];
SG_tensionerblock = SGofCPLy(PL_tensionerblock,10);
SG_tensionerblock = SGtransrelSG(SG_tensionerblock,SG_top_plate,'ontop','alignback');
PL_tensioner_teeth_block = PLsquare((rotor_radius+5.2)*2,8.2);
SG_tensioner_teeth_block = SGofCPLy(PL_tensioner_teeth_block,11);
SG_tensioner_teeth_block = SGtransrelSG(SG_tensioner_teeth_block,SG_top_plate,'ontop',-0.1,'alignback',0.5);
SG_crimp_tensioner = SGcrimptensioner(10,3,20,2);
SG_crimp_tensioner_1 = SGtransrelSG(SG_crimp_tensioner,SG_tensionerblock,'behind','centerz','transx',rotor_radius);
SG_crimp_tensioner_2 = SGtransrelSG(SG_crimp_tensioner,SG_tensionerblock,'behind','centerz','transx',-rotor_radius);
SG_crimp_tensioner = SGcat(SG_crimp_tensioner_1,SG_crimp_tensioner_2);
SG_crimp_tensioner = SGcat(SG_crimp_tensioner,SG_tensionerblock);

for i=1:dofs
        SG_tensioners{end+1} = SGtrans(SG_crimp_tensioner,[-dists(i) 0 0]);
        SG_teeth_cut{end+1} = SGtrans(SG_tensioner_teeth_block,[-dists(i) 0 0]);
end

PL_teeth = PLroundcorners(PLsquare(sizex+30,10),4,10);
SG_teeth = SGofCPLz(PL_teeth,10);
SG_teeth = SGtransrelSG(SG_teeth,SG_top_plate,'ontop','alignback','alignleft');
SG_teeth = SGboolh('-',SG_teeth,SGcat(SG_teeth_cut));

PL_top_cover = [sizey+20 10;sizey+10 17;10 17;0 0;3 0;13 14;sizey+5 14;sizey+10 10];
PL_top_cover = PLroundcorners(PL_top_cover,[2,3,6,7],[3,10,10,3]);
SG_top_cover = SGofCPLx(PL_top_cover,sizex+24);
PL_sides = [sizey+20 10;sizey+10 17;10 17;0 0;sizey+10 0;sizey+10 10];
PL_sides = PLroundcorners(PL_sides,[2,3],[3,10]);
SG_side_1 = SGtransrelSG(SGofCPLx(PL_sides,3),SG_top_cover,'left');
SG_side_2= SGtransrelSG(SGofCPLx(PL_sides,3),SG_top_cover,'right');
SG_top_cover = SGcat(SG_top_cover,SG_side_1,SG_side_2);
SG_top_cover = SGtransrelSG(SG_top_cover,SG_top_plate,'ontop','alignback','alignleft');

SG_hinge = SGrealhinge(50,6,8);
SG_hinge = SGtransrelSG(SG_hinge,SG_top_cover,'infront','aligntop',-11);
SG_hinge_2 = SGtrans(SG_hinge,[-sizex+60 0 0]);
PL_hinge_brace = [0 0;0 6;13 6;10 1];
PL_hinge_brace = CPLbool('-',PL_hinge_brace,PLcircle((6/2)+3));
SG_hinge_brace = SGofCPLx(PL_hinge_brace,42);
SG_hinge_brace = SGtransrelSG(SG_hinge_brace,SG_hinge,'centerx','behind',-8.1,'aligntop');
SG_hinge_brace = SGcat(SG_hinge_brace,SGtrans(SG_hinge_brace,[-sizex+60 0 0]));
SG_top_cover = SGcat(SG_top_cover,SG_hinge_brace,SG_teeth);
H = [rotx(0) [0;-92.5;43]; 0 0 0 1];
SG_top_cover =SGTset(SG_top_cover,'B',H);
% SG_top_cover = SGTrotate(SG_top_cover,'B','x',0.6);


%% Abdeckung
PL_cover = CPLbool('-',PLsquare(40,box_height+2.5),PLtrans(PLsquare(40,box_height+2.5),[3 -3]));
PL_cover = PLroundcorners(PL_cover,[2,5],20);
SG_cover = SGtransrelSG(SGofCPLx(PL_cover,170),SG_back_plate,'alignfront','aligntop','left');

PL_guide_cover = CPLbool('-',PLsquare(10,6.8),PLtrans(PLsquare(5.4,5),[2.3 -1.75]));
SG_guide_cover = SGmirror(SGmirror(SGofCPLx(PL_guide_cover,170),'xy'),'xz');
SG_guide_cover_top = SGtransrelSG(SG_guide_cover,SG_cover,'rotx',pi/2,'roty',pi,'alignback','aligntop',-3,'alignleft');

PL_cover_plate = PL_back_plate;
SG_cover_plate = SGtransrelSG(SGofCPLx(PL_cover_plate,10),SG_cover,'left','alignfront','alignbottom');

%% Platinenbox

PL_pin_base = PLcircle(2.5);
PL_pin_top = PLcircle(1.4);
PL_pin_counterp = [PLcircle(2.5);NaN NaN;PLcircle(3)];

SG_pin_fe = SGstack('z',SGofCPLz(PL_pin_base,20),SGofCPLz(PL_pin_top,5));
SG_pin_ard = SGstack('z',SGofCPLz(PL_pin_base,25),SGofCPLz(PL_pin_top,10));
SG_pin_counter = SGofCPLz(PL_pin_counterp,20);

SG_arduino_mounting = SGcat(SG_pin_ard,SGtrans(SG_pin_ard,[81.28 -48.26 0]),SGtrans(SG_pin_ard,[74.93 0 0]),SGtrans(SG_pin_ard,[-1.27 -48.26 0]));
SG_feetech_mounting = SGcat(SG_pin_fe,SGtrans(SG_pin_fe,[29 -49 0]),SGtrans(SG_pin_fe,[29 0 0]),SGtrans(SG_pin_fe,[0 -49 0]));

SG_base_plate = SGtrans(SGbox([103 60 3]),[0 0 1.5]);
SG_base_frame = SGofCPLz(CPLbool('-',PLsquare(109,66),PLtrans(PLsquare(106,60),[3 0])),27.5);
PL_power_16 = [PLtrans(PLcircle(1.6),[-9.5 0]);NaN NaN;PLcircle(6.5);NaN NaN;PLtrans(PLcircle(1.6),[9.5 0])];
PL_front_bot = [PLsquare(60,27.5);NaN NaN;PLtrans(PL_power_16,[-10 0]),;NaN NaN;PLtrans(PLsquare(14,7),[20 0])];
SG_front_bot = SGofCPLx(PL_front_bot,3);
SG_front_bot = SGtransrelSG(SG_front_bot,SG_base_plate,'alignbottom','right');
SG_arduino_mounting = SGtransrelSG(SG_arduino_mounting,SG_base_plate,'center','ontop');
SG_feetech_mounting = SGtransrelSG(SG_feetech_mounting,SG_base_plate,'rotz',pi/2,'center','ontop');

SG_top_frame = SGtransrelSG(SG_base_frame,SG_base_frame,'ontop',5);
SG_top_plate_f = SGtransrelSG(SG_base_plate,SG_top_frame,'aligntop');
SG_arduino_mounting_top = SGcat(SG_pin_counter,SGtrans(SG_pin_counter,[81.28 -48.26 0]),SGtrans(SG_pin_counter,[74.93 0 0]),SGtrans(SG_pin_counter,[-1.27 -48.26 0]));
SG_arduino_mounting_top = SGtransrelSG(SG_arduino_mounting_top,SG_top_plate_f,'center','under');

%% CAT
% SG_tool_mover = SGreadSTL("STLs\Assembly.STL");
% SG_tool_mover = SGtransrelSG(SG_tool_mover,SG_tool_mover_connection,'right',-23,'alignbottom','alignback',10);
SG_bottom = SGcat(SG_servo_guides,SG_main_frame,SG_main_frame_top,SG_positioning,SG_nut_holder);
SG_bottom = SGcat([{SG_bottom} SG_texts]);
SG_cover = SGcat(SG_cover,SG_guide_cover_top,SG_cover,SG_cover_plate);
SG_top_plate = SGcat([{SG_top_plate} SG_rotors_top_plate {SG_tensionerblock} SG_tensioners {SG_top_frame}]);
% SG = SGcat([{SG_bottom} SG_rotors {SG_top_plate} {SG_cover}]);
SG = SGcat([{SG_bottom} SG_rotors ]);

SG_electic_bot = SGcat(SG_base_plate,SG_arduino_mounting,SG_feetech_mounting,SG_base_frame,SG_front_bot);
SG_electric_top = SGcat(SG_top_frame,SG_arduino_mounting_top,SG_top_plate_f);

SG_electric = SGtransrelSG(SGcat(SG_electic_bot,SG_electric_top),SG,'rotz',-pi/2,'left',20,'alignbottom','alignback');
SG = SGcat(SG,SG_electric);


% SG = SG_top_plate;
% SG = SGcat(SG_bottom,SG_top_plate);
% SGwriteSTL(SG_cover,"SG_cover",'','y');
% SGwriteSTL(SG_bottom,"SG_bottom",'','y');
% SGwriteSTL(SG_top_plate,"SG_top_plate",'','y');
% for i=1:size(SG_rotors,2)
%     SGwriteSTL(SG_rotors{i},"Rotor"+i,'','y');
% end

% SGwriteSTL(SGcrimptensioner(10,3,20,3),"SG_nut",'','y');
% SGwriteSTL(SGcrimptensioner(10,3,20,1),"SG_big_nut",'','y');

SGwriteSTL(SG,"SG_box");
end