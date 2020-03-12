%%  [SG] = SGmanipulatorBox(dofs,varargin)
%	=== INPUT PARAMETERS ===
%	dofs:       number of degress of freedom
%   power:      1xn Vector, 1 = 4Nm 2 = 8Nm 3 = 12Nm
%	=== OUTPUT RESULTS ======
%
function [SG] = SGmanipulatorBoxSimple(power)
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
gap = 25;
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
SG_front_walls = [];
SG_back_walls = [];
SG_servo_guides = [];
SG_side_holes = [];
PL_top = [];
SG_cover = {};
SG_tension_back = {};
SG_texts = {};
SG_rotors_or = {};
SG_rotors = {};
SG_holes = [];
SG_sm40_holes = SGofCPLy(CPLcopypattern(PLcircle(1.5),SM40BL_holes(1:2),SM40BL_holes(3:4)),20);
PL_sm85_holes = PLtrans0(CPLcopypattern(PLcircle(1.5),SM85CL_holes(1:2),SM85CL_holes(3:4)));
PL_sm120_holes = PLtrans0(CPLcopypattern(PLcircle(1.5),SM120BL_holes(1:2),SM120BL_holes(3:4)));
PL_sm85_holes = [PLtrans(PL_sm85_holes,[0 -28-3]);NaN NaN;PLtrans(PLcircle(top_plate_holes(2)),[0 -servo_ax(2)])];
PL_sm120_holes = [PLtrans(PL_sm120_holes,[0 -35-4]);NaN NaN;PLtrans(PLcircle(top_plate_holes(3)),[0 -servo_ax(3)])];
SG_text = {SGoftext("SM40BL",[20 8 1]),SGoftext("SM85CL",[20 8 1]),SGoftext("SM120BL",[20 8 1])};
servo_hol = {PL_sm85_holes,PL_sm120_holes};

SG_servo_holes = {SGofCPLz(PLtrans(PLcircle(top_plate_holes(1)),[0 -servo_ax(1)]),4),SGofCPLz(servo_hol{1},4),SGofCPLz(servo_hol{2},4)};

dofs = size(power,2);
sizex = 0;
box_height = servo_d{max(power)}(3);

SG_crimp_tensioner = SGcrimptensioner(10,3,20,2);
SG_crimp_tensioner = SGtrans0(SGcat(SG_crimp_tensioner,SGtransrelSG(SG_crimp_tensioner,SG_crimp_tensioner,'transx',rotor_radius*2)));


PL_tensioner_front = PLroundcorners(CPLbool('+',PLsquare(rotor_radius*2,15),PLtrans(PLsquare((rotor_radius+8)*2,13),[0 2])),[4,5],10);
PL_tensionerblock = [PL_tensioner_front;NaN NaN;PLtrans(PLcircle(1.5),[rotor_radius -2.5]);NaN NaN;PLtrans(PLcircle(1.5),[-rotor_radius -2.5])];
SG_tensionerblock = SGofCPLy(PL_tensionerblock,10);
SG_tensioner = SGcat(SG_tensionerblock,SGtransrelSG(SG_crimp_tensioner,SG_tensionerblock,'behind','centerx','centerz',-2.5));
SG_tensioners = [];

PL_tensioner_back = CPLbool('+',PLtrans(PLsquare(rotor_radius*2,6),[0 -1]),PLsquare((rotor_radius+8)*2,4));
SG_tensioner_back = SGofCPLy(PL_tensioner_back,10);

PL_cover_outside =  PLroundcorners(PLtrans(PLsquare( (rotor_radius+10)*2 , 18 ),[0 2]),[3,4],10);
SG_cover_outside = SGofCPLy(PL_cover_outside,3);
PL_cover_mid = CPLbool('-', PL_cover_outside,PLroundcorners(PLtrans(PLsquare( (rotor_radius+8)*2 , 18 ),[0 0]),[3,4],10));
PL_cover_front = CPLbool('-', PL_cover_outside,PL_tensioner_front);
SG_cover_front = SGofCPLy(PL_cover_front,10);
               
SG_sm40_conn = SGofCPLcommand('c 25,d 3 7 0,d 3 -7 0,d 3 0 7,d 3 0 -7,c 8,h 2,enter,c 19,c 25,h 2.5,rel under 0,cat,col y');
SG_rotors_or{end+1} = SGservorotor(rotor_radius,SG_sm40_conn,[7 4 1.5],1);
SG_sm85_conn = SGofCPLcommand('c 30,d 3 10.5 0,d 3 -10.5 0,d 3 0 10.5,d 3 0 -10.5,c 13.5,h 1,enter,c 26,c 30,h 2.5,rel under 0,cat,col y');
SG_rotors_or{end+1} = SGservorotor(rotor_radius,SG_sm85_conn,[10.5 4 1.5],2);
SG_sm120_conn = SGofCPLcommand('c 38,d 3 12.5 0,d 3 -12.5 0,d 3 0 12.5,d 3 0 -12.5,c 16,h 3,enter,c 34.5,c 38,h 3,rel under 0,cat,col y');
SG_rotors_or{end+1} = SGservorotor(rotor_radius,SG_sm120_conn,[12.5 4 1.5],3);




dists = 0;
for i=1:dofs
    PL_side = PLtrans(CPLbool('-',PLsquare(10,servo_d{power(i)}(1)+5),PLtrans(PLsquare(5,servo_d{power(i)}(1)+5),[2.5 5])),[-servo_d{power(i)}(2)/2 0]);
    PL_guide = [PL_side;NaN NaN;VLswapX(PL_side)];
    SG_guide = SGtrans(SGofCPLz(PL_guide,15),[0 0 -(servo_d{3}(3)-servo_d{power(i)}(3))]);
    
    
    sizex_old = sizex;
    [sizex,sizey,~,~,~,~] = sizeVL(SG_guide);
    if isempty(SG_servo_guides)
        SG_servo_guides = SGtrans(SG_guide,[0 -sizey/2 0]);
        sizex_first = sizex/2;
    else
        dists = [dists dists(i-1) + max(sizex_old/2,rotor_radius) + max(sizex/2,rotor_radius)+gap];
        SG_guide = SGtransrelSG(SG_guide,SG_servo_guides,'transx',-dists(i),'alignback');
        SG_servo_guides = SGcat(SG_servo_guides,SG_guide);
    end
    PL_top = CPLbool('+',PL_top,PLtrans(PLsquare(max(sizex,rotor_radius*2),3),[-dists(i) -(servo_d{3}(3)-servo_d{power(i)}(3))]));
    if i<3
        if(power(i)==power(i+1))
            PL_top = CPLbool('+',PL_top,PLtrans(PLsquare(gap,3),[-max(sizex,rotor_radius*2)/2-gap/2 -(servo_d{3}(3)-servo_d{power(i)}(3))]));
        else
            PL_top_temp =  CPLbool('+',PLsquare(3,abs(servo_d{power(i+1)}(3)-servo_d{power(i)}(3))+3),PLtrans(PLsquare(gap/2,3),[gap/4 (servo_d{power(i+1)}(3)-servo_d{power(i)}(3))/2]));
            PL_top_temp = CPLbool('+',PL_top_temp,PLtrans(PLsquare(gap/2,3),[-gap/4 -(servo_d{power(i+1)}(3)-servo_d{power(i)}(3))/2]));
            
            if power(i) < power(i+1)
                PL_top_temp = VLswapX(PL_top_temp);
                PL_top_temp = PLtrans(PL_top_temp,[0 -0.5*abs(servo_d{power(i+1)}(3)-servo_d{power(i)}(3))]);
                PL_top_temp = PLroundcorners(PL_top_temp,[1],5);
            else
                PL_top_temp = VLswapX(PL_top_temp);
                PL_top_temp = PLtrans(PL_top_temp,[0 -0.5*abs(servo_d{power(i+1)}(3)-servo_d{power(i)}(3))]);
                PL_top_temp = PLroundcorners(PL_top_temp,[5],5);
            end
            PL_top = CPLbool('+',PL_top,PLtrans(PL_top_temp,[-max(sizex,rotor_radius*2)/2-gap/2-dists(i) 0]));
        end
    end
    SG_holes = SGcat(SG_holes,SGtransrelSG(SG_servo_holes{power(i)},SG_guide,'ontop','transx',-dists(i),'transz',-0.5));
    
    SG_tensioners = SGcat(SG_tensioners,SGtransrelSG(SG_tensioner,SG_guide,'ontop',3,'centerx'));
    
    width_wall = max(sizex,rotor_radius*2);
    offset = 0;
    if i<dofs
        if power(i+1)>power(i)
            width_wall = width_wall+gap/2+1.5;
            offset = offset-(gap/2+1.5)/2;
        elseif power(i+1)<power(i)
            width_wall = width_wall+gap/2-1.5;
            offset = offset-(gap/2-1.5)/2;
        else
            width_wall = width_wall+gap/2;
            offset = offset-(gap/2)/2;
        end
    end
    if i > 1
        if power(i-1)>power(i)
            width_wall = width_wall+gap/2+1.5;
            offset = offset+(gap/2+1.5)/2;
        elseif  power(i-1)<power(i)
            width_wall = width_wall+gap/2-1.5;
            offset = offset+(gap/2-1.5)/2;
        else
            width_wall = width_wall+gap/2;
            offset = offset+(gap/2)/2;
        end
    end
    SG_wall_temp_b = SGtransrelSG(SGbox([width_wall,3,servo_d{power(i)}(3)]),SG_servo_guides,'transx',-dists(i)+offset,'aligntop','behind');
    SG_wall_temp_b = SGtransrelSG(SG_wall_temp_b,SG_guide,'aligntop');
    
    SG_wall_temp_f = SGtransrelSG(SGbox([width_wall,10,servo_d{power(i)}(3)]),SG_servo_guides,'transx',-dists(i)+offset,'aligntop','behind');
    SG_wall_temp_f = SGtransrelSG(SG_wall_temp_f,SG_guide,'aligntop');
    
    SG_back_walls = SGcat(SG_back_walls,SG_wall_temp_b);
    if power(i) == 1
        SG_hole_cut = SGtransrelSG(SG_sm40_holes,SG_guide,'centerx','aligntop',-9.5);
        SG_wall_temp_f = SGboolh('-',SG_wall_temp_f,SGtransrelSG(SG_hole_cut,SG_wall_temp_f,'centery'));
    end
    SG_front_walls = SGcat(SG_front_walls,SG_wall_temp_f);
    
 
    
    
    SG_text_temp = SGtransrelSG(SG_text{power(i)},SG_guide,'rotx',pi/2,'rotz',pi,'centerx');
    SG_text_temp = SGtransrelSG(SG_text_temp,SG_wall_temp_f,'alignbottom','behind');
    SG_texts{end+1} = SG_text_temp;
    SG_rotors{end+1} = SGtransrelSG(SG_rotors_or{power(i)},SG_guide,'transx',-dists(i),'transy',-servo_ax(power(i)),'ontop',2);
    SG_cover{end+1} = SGtransrelSG(SGstack('y',SG_cover_front,SGofCPLy(PL_cover_mid, servo_d{power(i)}(3)+20 ),SG_cover_front,SG_cover_outside),SG_guide,'transx',-dists(i),'ontop',3,'transy',10,'transy',-(0)*60);
    SG_tension_back{end+1}  = SGtransrelSG(SG_tensioner_back,SG_guide,'ontop',3,'transx',-dists(i),'transy',-80.5);
end


[sizex,sizey,~,~,~,~] = sizeVL(SG_servo_guides);
SG_main_frame_top = SGtransrelSG(SGofCPLy(PL_top,sizey+20),SG_servo_guides,'aligntop',3,'alignfront',10);
SG_main_frame_top = SGbool5('-',SG_main_frame_top,SG_holes);
SG_back_walls = SGtransrelSG(SG_back_walls,SG_main_frame_top,'alignfront');
SG_tensioners = SGtransrelSG(SG_tensioners,SG_main_frame_top,'behind',-10);


PL_top_front_2 = CPLbool('-', PLsquare(10,sizey+20) , PLtrans( PLsquare(10,sizey+7) , [3 -3.5] ) );
SG_top_front_2 = SGofCPLz(PLroundcorners(PL_top_front_2,[1,2,5,6],5), servo_d{power(dofs)}(3)-10  );
SG_top_front_2_2 = SGofCPLz(CPLbool('-',PLroundcorners(PL_top_front_2,[1,2,5,6],5),PLsquare(50,50)),10);
SG_top_front_2 = SGstack('z',SG_top_front_2_2,SG_top_front_2);
SG_top_front_2 = SGtransrelSG(SG_top_front_2,SG_main_frame_top,'under',-max(0,servo_d{power(dofs)}(3)-servo_d{power(1)}(3) ),'alignback','left');
PL_top_front = PLroundcorners(PLsquare(10,sizey+20),[1,4],5);
SG_top_front = SGtransrelSG(SGofCPLz(PL_top_front,3),SG_top_front_2,'ontop','alignback','alignleft');
SG_top_front = SGcat(SG_top_front,SG_top_front_2);

PL_top_end_2 = CPLbool('-', PLsquare(10,sizey+20) , PLtrans( PLsquare(10,sizey+7) , [-3 -3.5] ));
SG_top_end_2 = SGofCPLz( PLroundcorners(PL_top_end_2,[3,4,7,8],5), servo_d{power(1)}(3) -10  );
SG_top_end_2_2 = SGofCPLz( CPLbool('-',PLroundcorners(PL_top_end_2,[3,4,7,8],5),PLsquare(50,50)), 10  );
SG_top_end_2 = SGstack('z',SG_top_end_2_2,SG_top_end_2);
SG_top_end_2 = SGtransrelSG(SG_top_end_2,SG_main_frame_top,'under',-max(0,servo_d{power(1)}(3)-servo_d{power(dofs)}(3) ),'alignback','right');
PL_top_end = PLroundcorners(PLsquare(10,sizey+20),[2,3],5);
SG_top_end = SGtransrelSG(SGofCPLz(PL_top_end,3),SG_top_end_2,'ontop','alignback','alignleft');
SG_top_end = SGcat(SG_top_end,SG_top_end_2);







SG = SGcat(SG_back_walls,SG_main_frame_top,SG_front_walls,SG_servo_guides,SG_tensioners,SG_top_end,SG_top_front);
SG = SGcat([{SG} SG_texts]);
SGwriteSTL(SG,"SG-Box");
SG = SGcat([{SG} SG_rotors]);




% PL_nut_holder = [PLsquare(12);NaN NaN;PLcircle(4.1,6)];
% PL_nut_holder_base = [PLsquare(12);NaN NaN;PLcircle(2.5)];
% SG_nut_holder = SGofCPLz(PL_nut_holder,6);
% SG_nut_holder_base = SGofCPLz(PL_nut_holder_base,3);
% SG_nut_holder = SGstack('z',SG_nut_holder_base,SG_nut_holder);
% SG_nut_holder = SGpatternXY(SG_nut_holder,2,2,sizex+8,sizey-1);
% SG_nut_holder = SGtransrelSG(SG_nut_holder,SG_main_frame,'centerx','centery','alignbottom');
%
% %%Rotors
%
% 

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
PL_base_frame = CPLbool('-',PLsquare(109,66),PLtrans(PLsquare(106,60),[3 0]));
SG_base_frame = SGofCPLz(PL_base_frame,17.5);
SG_base_frame_2 = SGofCPLz(CPLbool('-',PL_base_frame,PLtrans(PLsquare(20,100),[-20 50])),10);
SG_base_frame = SGstack('z',SG_base_frame_2,SG_base_frame);

PL_clips = CPLbool('-',PLsquare(8,15),PLtrans(PLsquare(8,15),[-5 5]));
SG_clips = SGofCPLx(PL_clips,15);
SG_clips = SGtransrelSG(SG_clips,SG_base_frame,'alignbottom',-4.8,'behind','alignleft',-10);
SG_clips = SGcat(SG_clips,SGright(SG_clips,SG_clips,20));


PL_power_16 = [PLtrans(PLcircle(1.6),[-9.5 0]);NaN NaN;PLcircle(6.5);NaN NaN;PLtrans(PLcircle(1.6),[9.5 0])];
PL_front_bot = [PLsquare(60,27.5);NaN NaN;PLtrans(PL_power_16,[-10 0]),;NaN NaN;PLtrans(PLsquare(14,7),[20 0])];
SG_front_bot = SGofCPLx(PL_front_bot,3);
SG_front_bot = SGtransrelSG(SG_front_bot,SG_base_plate,'alignbottom','right');
SG_arduino_mounting = SGtransrelSG(SG_arduino_mounting,SG_base_plate,'center','ontop');
SG_feetech_mounting = SGtransrelSG(SG_feetech_mounting,SG_base_plate,'rotz',pi/2,'center','ontop');

SG_top_frame = SGtransrelSG(SGofCPLz(PL_base_frame,27.5),SG_base_frame,'ontop',5);
SG_top_plate_f = SGtransrelSG(SG_base_plate,SG_top_frame,'aligntop');
SG_arduino_mounting_top = SGcat(SG_pin_counter,SGtrans(SG_pin_counter,[81.28 -48.26 0]),SGtrans(SG_pin_counter,[74.93 0 0]),SGtrans(SG_pin_counter,[-1.27 -48.26 0]));
SG_arduino_mounting_top = SGtransrelSG(SG_arduino_mounting_top,SG_top_plate_f,'center','under');


%
% %% CAT
% % SG_tool_mover = SGreadSTL("STLs\Assembly.STL");
% % SG_tool_mover = SGtransrelSG(SG_tool_mover,SG_tool_mover_connection,'right',-23,'alignbottom','alignback',10);
% SG_bottom = SGcat(SG_servo_guides,SG_main_frame,SG_main_frame_top,SG_nut_holder);
% SG_bottom = SGcat([{SG_bottom} SG_texts {SG_crimp_tensioners} {SG_tensionerblocks} SG_rotors {SG_holes} {SG_back_walls}]);
%
SG_electic_bot = SGcat(SG_base_plate,SG_arduino_mounting,SG_feetech_mounting,SG_base_frame,SG_front_bot,SG_clips);
SG_electric_top = SGcat(SG_top_frame,SG_arduino_mounting_top,SG_top_plate_f);
%
SG_electric = SGtransrelSG(SGcat(SG_electic_bot,SG_electric_top),SG,'rotz',-pi/2,'left',20,'alignbottom','alignback',-25);
SG = SGcat(SG,SG_electric);
%
% SGplot(SG);
SGwriteSTL(SG,"SG_box");
end