%%  [SG] = SGmanipulatorBox(dofs,varargin)
%	=== INPUT PARAMETERS ===
%	dofs:       number of degress of freedom
%   tensioner:  1xn matrix, defines which dofs need big motor
%	=== OUTPUT RESULTS ======
%
function [SG] = SGmanipulatorBox(dofs,varargin)
%%Varargin
tensioner=0;  if nargin>=2 && ~isempty(varargin{1}); tensioner=varargin{1}; end
%% Variables defining

rotor_radius = 26;

SM40BL_dimensions = [46.5,28.5,34.5]+0.5;   %depth,widht,height
SM85CL_dimensions = [64,34,47]+0.5;         %depth,widht,height
SM40BL_axle_distance = 35.25;
SM85CL_axle_distance = 47;
SM40BL_holes = [2,2,16,12];             %x_num,y_num,x_dis,y_dis
SM85CL_holes = [2,3,28,28];             %x_num,y_num,x_dis,y_dis

box_height = 70;
box_wall_thickness = 10;

%%Bottom
SG_servo_guides = [];
SG_side_holes = [];
PL_axle_positions = [];
SG_sm40_holes = SGofCPLy(CPLcopypattern(PLcircle(1.5),SM40BL_holes(1:2),SM40BL_holes(3:4)),20);
PL_sm85_holes = PLtrans0(CPLcopypattern(PLcircle(1.5),SM85CL_holes(1:2),SM85CL_holes(3:4)));
sizex = 0;
distance_to_next_mid_old = 0;
for i=1:dofs
    if ~ismember(i,tensioner)
        PL_side = PLtrans(CPLbool('-',PLsquare(20,SM40BL_dimensions(1)+10),PLtrans(PLsquare(10,SM40BL_dimensions(1)+10),[5 10])),[-SM40BL_dimensions(2)/2 0]);
    else
        PL_side = PLtrans(CPLbool('-',PLsquare(20,SM85CL_dimensions(1)+10),PLtrans(PLsquare(10,SM85CL_dimensions(1)+10),[5 10])),[-SM85CL_dimensions(2)/2 0]);
    end
    PL_guide = [PL_side;NaN NaN;VLswapX(PL_side)];
    SG_guide = SGofCPLz(PL_guide,15);
    
    sizex_old = sizex;
    [sizex,sizey,~,~,~,~] = sizeVL(SG_guide);
    distance_to_next_mid = max(rotor_radius-(sizex/2),0)+5+(sizex/2);
    if isempty(SG_servo_guides)
        SG_servo_guides = SGtrans(SG_guide,[0 -sizey/2 0]);
        sizex_first = sizex/2;
    else
        distance_next = max(rotor_radius-(sizex/2),0)+max(rotor_radius-(sizex_old/2),0)+10;
        SG_guide = SGtransrelSG(SG_guide,SG_servo_guides,'left',distance_next,'alignback');
        SG_servo_guides = SGcat(SG_servo_guides,SG_guide);
    end
    
    %  Kabelschacht 
    if(i==1)
        SG_cable_hole = SGtransrelSG(SGbox([30,40,box_height/2]),SG_guide,'centerx','aligntop',-box_height/5);
    end
    %
    %ScrewHoles
    if ismember(i,tensioner)
        axle_y_dis = SM85CL_axle_distance;
    else
        axle_y_dis = SM40BL_axle_distance;
        SG_side_holes = SGcat(SG_side_holes,SGtransrelSG(SG_sm40_holes,SG_guide,'centerx','aligntop',-9.5,'alignback',15));
    end
    
    if isempty(PL_axle_positions)
        PL_axle_positions = [PL_axle_positions;distance_to_next_mid_old,axle_y_dis];
    else
        PL_axle_positions = [PL_axle_positions;distance_to_next_mid_old+distance_to_next_mid+PL_axle_positions(end,1),axle_y_dis];
    end
    distance_to_next_mid_old = distance_to_next_mid;
end
[sizex,sizey,~,~,~,~] = sizeVL(SG_servo_guides);
PL_main_frame = CPLbool('-',PLsquare(sizex+20,sizey+20),PLsquare(sizex,sizey));
PL_main_frame = PLroundcorners(PL_main_frame,2,10);
SG_main_frame = SGofCPLz(PL_main_frame,box_height);
SG_main_frame = SGtransrelSG(SG_main_frame,SG_servo_guides,'centerx','alignback',10,'aligntop');
SG_side_holes = SGcat(SGtransrelSG(SG_cable_hole,SG_main_frame,'alignfront',1),SG_side_holes);
SG_main_frame = SGboolh('-',SG_main_frame,SG_side_holes);

PL_main_frame_top = PLtrans(PLroundcorners(PLsquare(sizex+20,sizey+10),4,10),[(sizex+20)/-2,(sizey+10)/-2]);
for k=1:size(PL_axle_positions,1)
    PL_main_frame_top = [PL_main_frame_top;NaN NaN;PLtrans(PLcircle(16),[-PL_axle_positions(k,1)-box_wall_thickness-sizex_first;-PL_axle_positions(k,2)-box_wall_thickness])];
     if ismember(k,tensioner)
        PL_main_frame_top = [PL_main_frame_top;NaN NaN;PLtrans(PL_sm85_holes,[-PL_axle_positions(k,1)-box_wall_thickness-sizex_first;-PL_axle_positions(k,2)+16-box_wall_thickness])];            
     end
end
SG_main_frame_top = SGofCPLz(PL_main_frame_top,10);
SG_main_frame_top = SGtransrelSG(SG_main_frame_top,SG_main_frame,'ontop','alignright','alignback');

PL_positioning = [PLsquare(sizex+9,sizey+9);NaN NaN;PLsquare(sizex+4,sizey+4)];
PL_positioning = CPLbool('-',PL_positioning,PLsquare(sizex-10,sizey*2));
PL_positioning = CPLbool('-',PL_positioning,PLsquare(sizex*2,sizey-20));
PL_positioning = CPLbool('-',PL_positioning,PLtrans(PLsquare(40,40),[-sizex/2,-sizey/2]));
SG_positioning = SGofCPLz(PL_positioning,5);
SG_positioning = SGtransrelSG(SG_positioning,SG_main_frame_top,'centerx','centery',-5,'ontop');

PL_back_plate = CPLbool('+',PLsquare(30,box_height),PLtrans(PLsquare(40,10),[5 box_height/2+5]));
PL_back_plate = CPLbool('-',PL_back_plate,PLtrans(PLcircle(8),[5 -10]));
PL_back_plate = PLroundcorners(PL_back_plate,2,20);
SG_back_plate = SGofCPLx(PL_back_plate,10);
SG_back_plate = SGtransrelSG(SG_back_plate,SG_main_frame_top,'aligntop','alignright','infront');

SG_bottom_plate = SGofCPLx(PLsquare(17,10),sizex);
SG_bottom_plate = SGtransrelSG(SG_bottom_plate,SG_back_plate,'left','alignbottom','alignfront',-13);

PL_guide = CPLbool('-',PLsquare(10,7),PLtrans(PLsquare(5),[2.5 -1.75]));
SG_guide = SGofCPLx(PL_guide,sizex);
SG_guide_bot = SGtransrelSG(SG_guide,SG_bottom_plate,'aligntop','alignleft','infront');
SG_guide_top = SGtransrelSG(SG_guide,SG_main_frame_top,'aligntop','alignleft',-10,'infront');

SG_pin = SGofCPLrot([0 0;5 0;5 4;1.5 4;1.5 10;0 10]);
SG_screw_pin = SGscrewDIN(-2,4,'',PLcircle(4));

SG_arduino_mounting = SGcat(SG_pin,SGtrans(SG_pin,[81.28 -48.26 0]),SGtrans(SG_screw_pin,[74.93 0 0]),SGtrans(SG_screw_pin,[-1.27 -48.26 0]));
SG_arduino_mounting = SGtransrelSG(SG_arduino_mounting,SG_main_frame,'rotx',pi/2,'infront','alignbottom',-10,'alignleft',-24);

SG_feetech_mounting = SGcat(SG_pin,SGtrans(SG_pin,[29 -49 0]),SGtrans(SG_screw_pin,[29 0 0]),SGtrans(SG_screw_pin,[0 -49 0]));
SG_feetech_mounting = SGtransrelSG(SG_feetech_mounting,SG_main_frame,'rotx',pi/2,'infront','alignbottom',-10,'alignright',-15);

PL_tool_move_connection = CPLbool('-',PLsquare(120,39),PLtrans(PLsquare(100,30),[0 -5]));
PL_tool_move_connection = PLroundcorners(PL_tool_move_connection,[2,3,6,7],[10 10 5 5]);
PL_tool_move_connection = CPLbool('-',PL_tool_move_connection,PLtrans(PLsquare(56.7,10),[-10 14.5]));
SG_tool_mover_connection = SGofCPLx(PL_tool_move_connection,28.5);
SG_tool_mover_connection = SGtransrelSG(SG_tool_mover_connection,SG_main_frame,'alignbottom','right','alignback');

SG_top_cover_side = SGtransrelSG(SGbox([57,66.5,5]),SG_tool_mover_connection,'aligntop',30,'alignleft','centery',-10);
PL_side_covers = [PLsquare(42.75,25);NaN NaN;PLtrans(CPLcopypattern(PLcircle(1.5),[1 2],[0 12]),[13.375 -9.5])];
SG_side_covers = SGofCPLy(PL_side_covers,4.7);
SG_side_covers = SGtransrelSG(SG_side_covers,SG_top_cover_side,'under','alignleft','alignback');
SG_side_covers_2 = SGtransrelSG(SG_side_covers,SG_top_cover_side,'under','alignleft','alignfront');
SG_side_covers = SGcat(SG_side_covers,SG_side_covers_2);

PL_nut_holder = [PLsquare(12);NaN NaN;PLcircle(4.1,6)];
PL_nut_holder_base = [PLsquare(12);NaN NaN;PLcircle(2.5)];
SG_nut_holder = SGofCPLz(PL_nut_holder,6);
SG_nut_holder_base = SGofCPLz(PL_nut_holder_base,3);
SG_nut_holder = SGstack('z',SG_nut_holder_base,SG_nut_holder);
SG_nut_holder = SGpatternXY(SG_nut_holder,2,2,sizex-12,sizey-12);
SG_nut_holder = SGtransrelSG(SG_nut_holder,SG_main_frame,'center','alignbottom');
SG_texts = {};
for i=1:dofs
    if ~ismember(i,tensioner)
        SG_text = SGoftext("SM40BL",[20 8 1]);
    else
        SG_text = SGoftext("SM85CL",[20 8 1]);
    end
    SG_texts{end+1} = SGtransrelSG(SG_text,SG_main_frame,'rotx',pi/2,'rotz',pi,'behind','transx',10,'alignbottom',-10,'transx',-PL_axle_positions(i,1));
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

for i=1:dofs
    if ~ismember(i,tensioner)
        SG_rotors{end+1} = SGtransrelSG(SG_sm40_rotor,SG_main_frame_top,'ontop',-8,'transy',-PL_axle_positions(i,2),'transx',-PL_axle_positions(i,1));
    else
        SG_rotors{end+1} = SGtransrelSG(SG_sm85_rotor,SG_main_frame_top,'ontop',-8,'transy',-PL_axle_positions(i,2),'transx',-PL_axle_positions(i,1));
    end
end


%% Top plate
SG_rotors_top_plate = {};
SG_rotor_top = SGtrans(SGservorotor(rotor_radius,'',''),TofR(rotz(110)));

PL_top_plate = PLsquare(sizex+20,sizey+20);
PL_top_plate = PLtrans(PL_top_plate,[(sizex+20)/-2,(sizey+20)/-2]);
PL_axle_positions_2 = PLtrans(PL_axle_positions,[(((sizex+20)/2)-34.5) ((sizey+20)/2)+6.75-45.25]);
for k=1:dofs
    PL_top_plate = [PL_top_plate;NaN NaN;PLtrans(PLcircle(16),[-PL_axle_positions(k,1)-box_wall_thickness-sizex_first;-PL_axle_positions(k,2)-box_wall_thickness])];  
end
SG_top_plate = SGofCPLz(PLroundcorners(PL_top_plate,4,10),5);
SG_top_plate = SGtransrelSG(SG_top_plate,SG_main_frame,'ontop',23,'alignright','alignback');

PL_top_frame = CPLbool('-',PLsquare(sizex+20,sizey+20),PLsquare(sizex+10,sizey+10));
PL_top_frame =  PLroundcorners(PL_top_frame,2,10);
SG_top_frame = SGofCPLz(PL_top_frame,13);
SG_top_frame = SGtransrelSG(SG_top_frame,SG_top_plate,'under','alignright','alignback');


%top sm40
SG_connection_top = SGofCPLz([PLcircle(17);NaN NaN;PLsquare(22.1)],6.5);
SG_connection_top_top = SGofCPLz(PLcircle(17),2);
SG_connection_top_through = SGofCPLz(PLcircle(10),6);
SG_connection_top = SGstack('z',SG_connection_top,SG_connection_top_top,SG_connection_top_through,SG_rotor_top);

%top sm85
SG_connection_top_85 = SGofCPLz([PLcircle(17.1,5);NaN NaN;PLcircle(19)],6.5);
SG_connection_top_top_85 = SGofCPLz(PLcircle(19),2);
SG_connection_top_through_85 = SGofCPLz(PLcircle(10),6);
SG_connection_top_85 = SGstack('z',SG_connection_top_85,SG_connection_top_top_85,SG_connection_top_through_85,SG_rotor_top);

for i=1:dofs
    if ~ismember(i,tensioner)
        SG_rotors_top_plate{end+1} = SGtransrelSG(SG_connection_top,SG_top_plate,'ontop',-14,'transy',-PL_axle_positions(i,2),'transx',-PL_axle_positions(i,1));
    else
        SG_rotors_top_plate{end+1} = SGtransrelSG(SG_connection_top_85,SG_top_plate,'ontop',-14,'transy',-PL_axle_positions(i,2),'transx',-PL_axle_positions(i,1));
    end
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
    SG_tensioners{end+1} = SGtrans(SG_crimp_tensioner,[-PL_axle_positions(i,1) 0 0]);
    SG_teeth_cut{end+1} = SGtrans(SG_tensioner_teeth_block,[-PL_axle_positions(i,1) 0 0]);
end

PL_teeth = PLroundcorners(PLsquare(sizex+20,10),4,10);
SG_teeth = SGofCPLz(PL_teeth,10);
SG_teeth = SGtransrelSG(SG_teeth,SG_top_plate,'ontop','alignback','alignleft');
SG_teeth = SGboolh('-',SG_teeth,SGcat(SG_teeth_cut));

PL_top_cover = [sizey+20 10;sizey+10 17;10 17;0 0;3 0;13 14;sizey+5 14;sizey+10 10];
PL_top_cover = PLroundcorners(PL_top_cover,[2,3,6,7],[3,10,10,3]);
SG_top_cover = SGofCPLx(PL_top_cover,sizex+14);
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
SG_top_cover = SGTrotate(SG_top_cover,'B','x',0.6);


%% Abdeckung
PL_cover = CPLbool('-',PLsquare(30,box_height+10),PLtrans(PLsquare(30,box_height+10),[3 -3]));
PL_cover = PLroundcorners(PL_cover,[2,5],20);
SG_cover = SGtransrelSG(SGofCPLx(PL_cover,sizex),SG_back_plate,'alignfront','aligntop','left');

PL_guide_cover = CPLbool('-',PLsquare(10,6.8),PLtrans(PLsquare(5.4,5),[2.3 -1.75]));
SG_guide_cover_top = SGmirror(SGmirror(SGofCPLx(PL_guide_cover,sizex),'xy'),'xz');
SG_guide_cover_top = SGtransrelSG(SG_guide_cover_top,SG_guide_top,'alignfront','aligntop',-3,'alignleft');
SG_guide_cover_top_add = SGtransrelSG(SGofCPLx([0 0;-7 7;0 7],sizex),SG_cover,'aligntop',-3,'alignback','alignleft');
SG_guide_cover_bot = SGtransrelSG(SG_guide_cover_top,SG_guide_bot,'alignfront','aligntop',-3,'alignleft');

PL_cover_plate = PL_back_plate;
SG_cover_plate = SGtransrelSG(SGofCPLx(PL_cover_plate,10),SG_cover,'left','alignfront','alignbottom');


%% CAT
% SG_tool_mover = SGreadSTL("STLs\Assembly.STL");
% SG_tool_mover = SGtransrelSG(SG_tool_mover,SG_tool_mover_connection,'right',-23,'alignbottom','alignback',10);
SG_bottom = SGcat(SG_servo_guides,SG_main_frame,SG_main_frame_top,SG_back_plate,SG_bottom_plate,SG_guide_bot,SG_guide_top,SG_arduino_mounting,SG_feetech_mounting,SG_tool_mover_connection,SG_side_covers,SG_positioning,SG_top_cover_side,SG_nut_holder);
SG_bottom = SGcat([{SG_bottom} SG_texts]);
SG_cover = SGcat(SG_cover,SG_guide_cover_top,SG_guide_cover_bot,SG_guide_cover_top_add,SG_cover_plate);
SG_top_plate = SGcat([{SG_top_plate} SG_rotors_top_plate {SG_tensionerblock} SG_tensioners {SG_top_frame} {SG_hinge} {SG_hinge_2} {SG_top_cover}]);
SG = SGcat([{SG_bottom} SG_rotors]);
SGwriteSTL(SG_cover,"SG_cover",'','y');
SGwriteSTL(SG_bottom,"SG_bottom",'','y');
SGwriteSTL(SG_top_plate,"SG_top_plate",'','y');
% for i=1:size(SG_rotors,2)
%     SGwriteSTL(SG_rotors{i},"Rotor"+i,'','y');
% end

% SGwriteSTL(SGcrimptensioner(10,3,20,3),"SG_nut",'','y');
% SGwriteSTL(SGcrimptensioner(10,3,20,1),"SG_big_nut",'','y');

% SGwriteSTL(SG,"SG_box");
end