%%  [SG] = SGmanipulatorBox(dofs,varargin)
%	=== INPUT PARAMETERS ===
%	dofs:       number of degress of freedom
%   power:      1xn Vector, 1 = 4Nm 2 = 8Nm 3 = 12Nm
%	=== OUTPUT RESULTS ======
%
function [SG] = SGmanipulatorBoxSimple(dofs,power)
%% Variables defining

rotor_radius = 25;

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

for i=1:size(power,2)
    if power(i) > 85
        power(i) = 3;
    elseif power(i) >40
        power(i) = 2;
    else        
        power(i) = 1;
    end
end


%%Bottom
SG_servo_guides = [];
SG_side_holes = [];
SG_texts = {};
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

SG_crimp_tensioner = SGcrimptensioner(10,3,20,2);
SG_crimp_tensioner = SGtrans0(SGcat(SG_crimp_tensioner,SGtransrelSG(SG_crimp_tensioner,SG_crimp_tensioner,'transx',rotor_radius*2)));
SG_crimp_tensioner = SGtransrelSG(SG_crimp_tensioner,SG_main_frame_top,'ontop','behind',10);
SG_crimp_tensioners = [];

PL_tensionerblock = [PLsquare((rotor_radius+5)*2,10);NaN NaN;PLtrans(PLcircle(1.5),[rotor_radius 0]);NaN NaN;PLtrans(PLcircle(1.5),[-rotor_radius 0])];
SG_tensionerblock = SGtransrelSG(SGofCPLy(PL_tensionerblock,10),SG_main_frame_top,'ontop','behind');
SG_tensionerblocks = [];

PL_nut_holder = [PLsquare(12);NaN NaN;PLcircle(4.1,6)];
PL_nut_holder_base = [PLsquare(12);NaN NaN;PLcircle(2.5)];
SG_nut_holder = SGofCPLz(PL_nut_holder,6);
SG_nut_holder_base = SGofCPLz(PL_nut_holder_base,3);
SG_nut_holder = SGstack('z',SG_nut_holder_base,SG_nut_holder);
SG_nut_holder = SGpatternXY(SG_nut_holder,2,2,sizex+8,sizey-1);
SG_nut_holder = SGtransrelSG(SG_nut_holder,SG_main_frame,'centerx','centery','alignbottom');

for i=1:dofs    
    SG_texts{end+1} = SGtransrelSG(SG_text{power(i)},SG_main_frame,'rotx',pi/2,'rotz',pi,'behind','transx',10,'alignbottom',-2.5,'transx',-dists(i));
    SG_crimp_tensioners = SGcat(SG_crimp_tensioners,SGtrans(SG_crimp_tensioner,[-dists(i) 0 0]));
    SG_tensionerblocks = SGcat(SG_tensionerblocks,SGtrans(SG_tensionerblock,[-dists(i) 0 0]));
end

[size_x,~,~,~,~,~] = sizeVL(SG_tensionerblocks.VL);
SG_tensioning_brace = SGof2CPLsz(PLtrans(PLsquare(size_x,0.2),[0 -4.9]),PLsquare(size_x,10),15);
SG_tensioning_brace = SGtransrelSG(SG_tensioning_brace,SG_tensionerblocks,'centerx','under','alignfront');

%%Rotors
SG_sm85_conn = SGofCPLcommand('c 30,d 3 10.5 0,d 3 -10.5 0,d 3 0 10.5,d 3 0 -10.5,c 13.5,h 7,enter,c 26,c 30,h 2.5,rel under 0,cat,col y');
SG_sm85_conn_rotor = SGservorotor(rotor_radius,SG_sm85_conn,[10.5 4 1.5]);
SG_sm85_conn_rotor = SGtransrelSG(SG_sm85_conn_rotor,SG_main_frame_top,'transy',-SM85CL_axle_distance,'ontop',-5);

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
SG_bottom = SGcat(SG_servo_guides,SG_main_frame,SG_main_frame_top,SG_nut_holder);
SG_bottom = SGcat([{SG_bottom} SG_texts {SG_crimp_tensioners} {SG_tensionerblocks} {SG_tensioning_brace} {SG_sm85_conn_rotor}]);

SG_electic_bot = SGcat(SG_base_plate,SG_arduino_mounting,SG_feetech_mounting,SG_base_frame,SG_front_bot);
SG_electric_top = SGcat(SG_top_frame,SG_arduino_mounting_top,SG_top_plate_f);

SG_electric = SGtransrelSG(SGcat(SG_electic_bot,SG_electric_top),SG_bottom,'rotz',-pi/2,'left',20,'alignbottom','alignback');
SG = SGcat(SG_bottom,SG_electric);

SGplot(SG);
SGwriteSTL(SG,"SG_box");
end