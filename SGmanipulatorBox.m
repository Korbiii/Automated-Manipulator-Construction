%%  [SG] = SGmanipulatorBox(dofs,varargin)
%	=== INPUT PARAMETERS ===
%	dofs:       number of degress of freedom
%   power:      1xn Vector, 1 = 4Nm 2 = 8Nm 3 = 12Nm
%	=== OUTPUT RESULTS ======
%
function [SG] = SGmanipulatorBoxSimple(power)
%% Variables defining

rotor_radius = 25;

SM40BL_dimensions = [46.5,28.5,36.5]+0.5;   %depth,widht,height
SM85CL_dimensions = [62,34,49]+0.5;         %depth,widht,height
SM120BL_dimensions = [78 43 67.5]+0.5;      %depth,widht,height
SM40BL_axle_distance = 35.25;
SM85CL_axle_distance = 47;
SM120BL_axle_distance = 56.5;
SM40BL_holes = [2,2,16,12];                 %x_num,y_num,x_dis,y_dis
SM85CL_holes = [2,3,28,28];                 %x_num,y_num,x_dis,y_dis
SM120BL_holes = [2,3,35,35];                %x_num,y_num,x_dis,y_dis

top_plate_holes = [13.5,16,20];
servo_d = {SM40BL_dimensions,SM85CL_dimensions,SM120BL_dimensions};
servo_ax = [SM40BL_axle_distance,SM85CL_axle_distance,SM120BL_axle_distance];

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
[SG_front_walls,SG_back_walls,SG_servo_guides,PL_top,SG_holes] = deal([]);
[SG_cover,SG_tension_back,SG_tensioner_holders,SG_texts,SG_rotors_or,SG_rotor_clickbase,SG_rotors,SG_rotor_clickbases] = deal({});


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

SG_crimp_tensioner = SGcrimptensioner(10,3,20,2);
SG_crimp_tensioner = SGtrans0(SGcat(SG_crimp_tensioner,SGtransrelSG(SG_crimp_tensioner,SG_crimp_tensioner,'transx',rotor_radius*2)));

PL_slot_tensioner = [-10 -5;-5 5;5 5;10 -5];
PL_tensioner_front = PLroundcorners(PLsquare((rotor_radius+8)*2,15),[1,2,3,4],5);
PL_tensionerblock = [PL_tensioner_front;NaN NaN;PLtrans(PLcircle(1.5),[rotor_radius -2.5]);NaN NaN;PLtrans(PLcircle(1.5),[-rotor_radius -2.5])];
SG_tensionerblock = SGofCPLy(PL_tensionerblock,10);
SG_slot_slot = SGofCPLy(PL_slot_tensioner,10);
SG_tensioner = SGcat(SG_tensionerblock,SGtransrelSG(SG_crimp_tensioner,SG_tensionerblock,'behind','centerx','centerz',-2.5));
SG_slot_slot = SGtransrelSG(SG_slot_slot,SG_tensioner,'alignbottom','infront');
SG_tensioner = SGcat(SG_slot_slot,SG_tensioner);
SG_tensioners = [];

PL_tensioner_back = CPLbool('+',PLtrans(PLsquare(rotor_radius*2,6),[0 -1]),PLsquare((rotor_radius+8)*2,4));
SG_tensioner_back = SGofCPLy(PL_tensioner_back,10);

PL_cover_outside =  PLroundcorners(PLtrans(PLsquare( (rotor_radius+10)*2 , 18 ),[0 2]),[3,4],10);
SG_cover_outside = SGofCPLy(PL_cover_outside,3);
PL_cover_mid = CPLbool('-', PL_cover_outside,PLroundcorners(PLtrans(PLsquare( (rotor_radius+8)*2 , 18 ),[0 0]),[3,4],10));
PL_cover_front = CPLbool('-', PL_cover_outside,PL_tensioner_front);
SG_cover_front = SGofCPLy(PL_cover_front,10);
               
SG_sm40_conn = SGofCPLcommand('c 25,d 3 7 0,d 3 -7 0,d 3 0 7,d 3 0 -7,c 8,h 2,enter,c 19,c 25,h 2.5,rel under 0,cat,col y');
SG_rotors_or{end+1} = SGservorotor(rotor_radius-3,'',[7 4 1.5],1,19);
SG_sm85_conn = SGofCPLcommand('c 30,d 3 10.5 0,d 3 -10.5 0,d 3 0 10.5,d 3 0 -10.5,c 13.5,h 1,enter,c 26,c 30,h 2.5,rel under 0,cat,col y');
SG_rotors_or{end+1} = SGservorotor(rotor_radius,'',[10.5 4 1.5],2,22);
SG_sm120_conn = SGofCPLcommand('c 38,d 3 12.5 0,d 3 -12.5 0,d 3 0 12.5,d 3 0 -12.5,c 16,h 3,enter,c 34.5,c 38,h 3,rel under 0,cat,col y');
SG_rotors_or{end+1} = SGservorotor(rotor_radius,'',[12.5 4 1.5],3,26);


PL_retainer = CPLbool('+',PLsquare(1.5,8),[0.75 4;0.75 1.1;2 1.1]);
SG_retainer = SGofCPLx(PL_retainer,6);


PL_sm40_holes_rotor = CPLcopyradial(PLcircle(1.5),7,4,pi/4);
SG_disk_sm40 = SGofCPLz([PLcircle(15);NaN NaN;PLcircle(1.5);NaN NaN;PL_sm40_holes_rotor],1.5);
PL_sm40_top = [PLroundcorners(PLsquare(19),[1,2,3,4],1.5);NaN NaN;PLcircle(1.5)];
PL_cut_sm40 = CPLbool('-',PLsquare(8,24),PLsquare(8,12));
PL_sm40_top =CPLbool('-',PL_sm40_top,PL_cut_sm40);
PL_sm40_top = CPLbool('-',PL_sm40_top,PL_sm40_holes_rotor);
SG_sm40_top = SGofCPLz(PL_sm40_top,5.1);

SG_retainer_40 = SGtransrelSG(SG_retainer,SG_sm40_top,'center','alignbottom','behind',-1.5);
SG_retainer_40 = SGcat(SG_retainer_40,SGmirror(SG_retainer_40,'xz'));
SG_sm40_top = SGcat(SG_sm40_top,SG_retainer_40);
SG_sm40_click = SGstack('z',SGtransR(SG_sm40_conn,rot(0,0,pi/4)),SG_disk_sm40,SG_sm40_top);

SG_rotor_clickbase{1} = SG_sm40_click;


PL_sm85_holes_rotor = CPLcopyradial(PLcircle(1.5),10.5,4,pi/4);
SG_disk = SGofCPLz([PLcircle(15);NaN NaN;PLcircle(1.5);NaN NaN;PL_sm85_holes_rotor],1.5);
PL_sm85_top = [PLroundcorners(PLsquare(22),[1,2,3,4],2);NaN NaN;PLcircle(1.5)];
PL_cut = CPLbool('-',PLsquare(8,24),PLsquare(8,12));
PL_sm85_top =CPLbool('-',PL_sm85_top,PL_cut);
PL_sm85_top = CPLbool('-',PL_sm85_top,PL_sm85_holes_rotor);
SG_sm85_top = SGofCPLz(PL_sm85_top,5.1);

SG_retainer_85 = SGtransrelSG(SG_retainer,SG_sm85_top,'center','alignbottom','behind',-1.5);
SG_retainer_85 = SGcat(SG_retainer_85,SGmirror(SG_retainer_85,'xz'));
SG_sm85_top = SGcat(SG_sm85_top,SG_retainer_85);
SG_sm85_click = SGstack('z',SGtransR(SG_sm85_conn,rot(0,0,pi/4)),SG_disk,SG_sm85_top);

SG_rotor_clickbase{2} = SG_sm85_click;


PL_sm120_holes_rotor = CPLcopyradial(PLcircle(1.5),12.5,4,pi/4);
SG_disk_120 = SGofCPLz([PLcircle(19);NaN NaN;PLcircle(1.5);NaN NaN;PL_sm120_holes_rotor],1.5);
PL_sm120_top = [PLroundcorners(PLsquare(26),[1,2,3,4],2);NaN NaN;PLcircle(1.5)];
PL_cut_120 = CPLbool('-',PLsquare(8,26),PLsquare(8,16));
PL_sm120_top =CPLbool('-',PL_sm120_top,PL_cut_120);
PL_sm120_top = CPLbool('-',PL_sm120_top,PL_sm120_holes_rotor);
SG_sm120_top = SGofCPLz(PL_sm120_top,5.1);

SG_retainer_120 = SGtransrelSG(SG_retainer,SG_sm120_top,'center','alignbottom','behind',-1.5);
SG_retainer_120 = SGcat(SG_retainer_120,SGmirror(SG_retainer_120,'xz'));
SG_sm120_top = SGcat(SG_sm120_top,SG_retainer_120);
SG_sm120_click = SGstack('z',SGtransR(SG_sm120_conn,rot(0,0,pi/4)),SG_disk_120,SG_sm120_top);
SG_rotor_clickbase{3} = SG_sm120_click;

PL_tensioner_holder = CPLbool('-',PLsquare(rotor_radius*2-5,10),CPLgrow(PL_slot_tensioner,-0.1));
SG_tensioner_holder = SGofCPLy(PL_tensioner_holder,10);



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
    PL_top = CPLbool('+',PL_top,PLtrans(PLsquare(max(sizex,rotor_radius*2),3),[-dists(i) -(servo_d{max(power)}(3)-servo_d{power(i)}(3))]));
    if i<dofs
        if(power(i)==power(i+1))
            PL_top = CPLbool('+',PL_top,PLtrans(PLsquare(gap,3),[-max(sizex,rotor_radius*2)/2-gap/2-dists(i) -(servo_d{max(power)}(3)-servo_d{power(i)}(3))]));
        else
            PL_top_temp =  CPLbool('+',PLsquare(3,abs(servo_d{power(i+1)}(3)-servo_d{power(i)}(3))+3),PLtrans(PLsquare(gap/2,3),[gap/4 (servo_d{power(i+1)}(3)-servo_d{power(i)}(3))/2]));
            PL_top_temp = CPLbool('+',PL_top_temp,PLtrans(PLsquare(gap/2,3),[-gap/4 -(servo_d{power(i+1)}(3)-servo_d{power(i)}(3))/2]));
            
            if power(i) < power(i+1)
                PL_top_temp = VLswapX(PL_top_temp);
                PL_top_temp = PLtrans(PL_top_temp,[0 0.5*(abs(servo_d{power(i+1)}(3)-servo_d{power(i)}(3)))]);
                PL_top_temp = PLroundcorners(PL_top_temp,1,5);
            else
                PL_top_temp = VLswapX(PL_top_temp);
                PL_top_temp = PLtrans(PL_top_temp,[0 -0.5*(abs(servo_d{power(i+1)}(3)-servo_d{power(i)}(3)))]);
                PL_top_temp = PLroundcorners(PL_top_temp,5,5);
            end
            PL_top = CPLbool('+',PL_top,PLtrans(PL_top_temp,[-max(sizex,rotor_radius*2)/2-gap/2-dists(i) -abs(servo_d{max(power)}(3)-servo_d{power(i)}(3))]));
        end
    end
    SG_holes = SGcat(SG_holes,SGtransrelSG(SG_servo_holes{power(i)},SG_guide,'ontop','transx',-dists(i),'transz',-0.5));
    
    SG_tensioners = SGcat(SG_tensioners,SGtransrelSG(SG_tensioner,SG_guide,'ontop','centerx'));
    
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
    SG_rotor_clickbases{end+1} = SGtransrelSG(SG_rotor_clickbase{power(i)},SG_guide,'transx',-dists(i),'transy',-servo_ax(power(i)),'ontop');    
    SG_rotors{end+1} = SGtransrelSG(SG_rotors_or{power(i)},SG_guide,'transx',-dists(i),'transy',-servo_ax(power(i)),'ontop',5);
    SG_cover{end+1} = SGtransrelSG(SGstack('y',SG_cover_front,SGofCPLy(PL_cover_mid, servo_d{power(i)}(3)+20 ),SG_cover_front,SG_cover_outside),SG_guide,'transx',-dists(i),'ontop',3,'transy',10,'transy',-(0)*60);
    SG_tension_back{end+1}  = SGtransrelSG(SG_tensioner_back,SG_guide,'ontop',3,'transx',-dists(i),'transy',-80.5);
    SG_tensioner_holders{end+1} =SGtransrelSG(SG_tensioner_holder,SG_guide,'ontop',3,'transx',-dists(i));
end


[~,sizey,~,~,~,~] = sizeVL(SG_servo_guides);
SG_main_frame_top = SGtransrelSG(SGofCPLy(PL_top,sizey+20),SG_servo_guides,'aligntop',3,'alignfront',10);
SG_main_frame_top = SGbool5('-',SG_main_frame_top,SG_holes);
SG_back_walls = SGtransrelSG(SG_back_walls,SG_main_frame_top,'alignfront');
SG_tensioners = SGtransrelSG(SG_tensioners,SG_main_frame_top,'behind',-10);

SG_tensioner_holders = SGtransrelSG(SG_tensioner_holders,SG_main_frame_top,'alignback');

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
SG_base_box = SGcat(SG_back_walls,SG_main_frame_top,SG_front_walls,SG_servo_guides,SG_top_end,SG_top_front);

SG_tensioners = SGtransrelSG(SG_tensioners,SG_base_box,'transy',15);
SG_base_box = SGcat([{SG_base_box} SG_texts SG_tensioner_holders]);


%% Platinenbox
% 25
PL_pin_base = PLcircle(2.5);
PL_pin_top = PLcircle(1.4);
PL_pin_counterp = [PLcircle(1.6);NaN NaN;PLcircle(3)];
SG_pin_fe = SGstack('z',SGofCPLz(PL_pin_base,13),SGofCPLz(PL_pin_top,3));
SG_pin_ard = SGstack('z',SGofCPLz(PL_pin_base,15),SGofCPLz(PL_pin_top,10));
SG_pin_USB_shield_top = SGstack('z',SGofCPLz(PL_pin_base,8),SGofCPLz(PL_pin_top,10));
SG_pin_counter = SGofCPLz(PL_pin_counterp,20);

SG_arduino_mounting = SGcat(SG_pin_ard,SGtrans(SG_pin_ard,[82.55 0 0]),SGtrans(SG_pin_ard,[1.27 48.26 0]),SGtrans(SG_pin_ard,[76.2 48.26 0]));
SG_feetech_mounting = SGcat(SG_pin_fe,SGtrans(SG_pin_fe,[29 -49 0]),SGtrans(SG_pin_fe,[29 0 0]),SGtrans(SG_pin_fe,[0 -49 0]));
SG_arduino_mounting = SGmirror(SG_arduino_mounting,'xz');
SG_arduino_mounting = SGmirror(SG_arduino_mounting,'yz');

PL_base_plate = PLsquare(106,60);
SG_base_plate = SGofCPLz(PL_base_plate,3);
PL_base_frame = CPLbool('-',PLsquare(112,66),PLtrans(PLsquare(112,60),[3 0]));
SG_base_frame = SGofCPLz(PLroundcorners(PL_base_frame,[1,2],5),10);
SG_base_frame_2 = SGofCPLz(CPLbool('-',PLroundcorners(PL_base_frame,[1,2],5),PLtrans(PLsquare(20,100),[-20 50])),10);
SG_base_frame = SGstack('z',SG_base_frame_2,SG_base_frame);

PL_clips_corner = CPLbool('-',PLsquare(106,60),PLsquare(102,60));
PL_clips_corner = CPLbool('-',PL_clips_corner,PLsquare(109,54));
SG_clips_corner = SGofCPLz(PL_clips_corner,10);
SG_clips_corner = SGtransrelSG(SG_clips_corner,SG_base_frame,'ontop',-5);

PL_clips = CPLbool('-',PLsquare(8,15),PLtrans(PLsquare(8,15),[-5 5]));
SG_clips = SGofCPLx(PL_clips,15);
SG_clips = SGtransrelSG(SG_clips,SG_base_frame,'alignbottom',-4.8,'behind','alignleft',-10);
SG_clips = SGcat(SG_clips,SGright(SG_clips,SG_clips,20));

PL_power_16 = [PLtrans(PLcircle(2.1),[-9.5 0]);NaN NaN;PLcircle(6.5);NaN NaN;PLtrans(PLcircle(2.1),[9.5 0])];
PL_power_16 = CPLconvexhull(PL_power_16);
PL_front_bot = [PLsquare(60,20);NaN NaN;PLtrans(PL_power_16,[-15 0]),;NaN NaN;PLtrans(PLsquare(14,8),[20 0])];
SG_front_bot = SGofCPLx(PL_front_bot,3);
SG_front_bot = SGtransrelSG(SG_front_bot,SG_base_plate,'alignbottom','right');
SG_arduino_mounting = SGtransrelSG(SG_arduino_mounting,SG_base_plate,'center','ontop','transx',-3);
SG_feetech_mounting = SGtransrelSG(SG_feetech_mounting,SG_base_plate,'rotz',pi/2,'center','ontop','alignfront',-5,'transx',-4);

SG_top_frame = SGtransrelSG(SGofCPLz(PLroundcorners(PL_base_frame,[1,2],5),28),SG_base_frame,'ontop');
SG_top_plate_f = SGtransrelSG(SG_base_plate,SG_top_frame,'aligntop');
SG_pin_ard = SGtransrelSG(SGmirror(SG_pin_USB_shield_top,'xy'),SG_pin_counter,'aligntop');
SG_arduino_mounting_top =SGcat(SGtrans(SG_pin_ard,[52.07 32.98 0]),SGtrans(SG_pin_counter,[82.55 0 0]),SGtrans(SG_pin_ard,[52.07 5.08 0]),SGtrans(SG_pin_counter,[76.2 48.26 0]));

SG_arduino_mounting_top = SGmirror(SGmirror(SG_arduino_mounting_top,'yz'),'xz');
SG_arduino_mounting_top = SGtransrelSG(SG_arduino_mounting_top,SG_top_plate_f,'center','under','alignleft',-5.75);

SG_electric_bot = SGcat(SG_base_plate,SG_arduino_mounting,SG_feetech_mounting,SG_base_frame,SG_front_bot,SG_clips,SG_clips_corner);
SG_electric_top = SGcat(SG_top_frame,SG_arduino_mounting_top,SG_top_plate_f);

%% Matlab Plot
SG_electric = SGtransrelSG(SGcat(SG_electric_bot,SG_electric_top),SG_base_box,'rotz',-pi/2,'left',20,'alignbottom','alignback','left',-10);
SG = SGcat(SG_electric,SG_base_box);
SG = SGcat([{SG} SG_rotors SG_tensioners SG_rotor_clickbases]);
SGplot(SG,'w');

%% Output
SGwriteSTL(SG_electric_bot,"1x Electric Box Bottom",'','y');
SGwriteSTL(SG_electric_top,"1x Electric Box Top",'','y');
SGwriteSTL(SG_base_box,"1x Servobase",'','y');
SGwriteSTL(SG_tensioners,dofs+"x Tensioner",'','y');
for i=1:dofs
    SGwriteSTL(SG_rotor_clickbases{i},"1x Rotorclickbase "+i,'','y');
    SGwriteSTL(SG_rotors{power(i)},"1x Rotor "+i,'','y');
end


ServoNames = {"SM40BL","SM85CL","SM120BL"};
[~, userdir] = system('echo %USERPROFILE%');
userdir(end) = [];
path = strsplit(userdir, '\');
path{end+1} = 'Desktop\Manipulatorbox.txt';
path = strjoin(path,'\'); 
fileID = fopen(path,'w');

fprintf(fileID,'\n');
fprintf(fileID,'Elektronik:\n');
fprintf(fileID,'\n');
fprintf(fileID,'1x Arduino MEGA 2560\n');
fprintf(fileID,'Webseite: https://store.arduino.cc/arduino-mega-2560-rev3\n');
fprintf(fileID,'1x Feetech FE URT 1\n');
fprintf(fileID,'Webseite: https://www.premium-modellbau.de/feetech-fe-urt-1-fuer-sms-rs485-und-scs-ttl-smart-control-servos\n');
fprintf(fileID,'1x Arduino USB Host Shield\n');
fprintf(fileID,'Webseite: https://store.arduino.cc/arduino-usb-host-shield\n');
fprintf(fileID,'\n');
fprintf(fileID,'Servomotoren:\n');
fprintf(fileID,'\n');
for k=1:dofs
    fprintf(fileID,"1x Feetech "+ServoNames{power(i)}+" Servomotor\n");
end

fprintf(fileID,'\n');
fprintf(fileID,'Zubehör:\n');
fprintf(fileID,'\n');
fprintf(fileID,'Arduinojumper\n');
fprintf(fileID,'9V Netzteil für Arduino\n');
fprintf(fileID,'12-16V Netzteil für Servomotoren\n');
fprintf(fileID,'Mini-USB Kabel\n');

fclose(fileID);




end