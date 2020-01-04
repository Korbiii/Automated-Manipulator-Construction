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

rotor_radius = 25;

SM40BL_dimensions = [46.5,28.5,34.5]+0.5;   %depth,widht,height
SM85CL_dimensions = [64,34,47]+0.5;         %depth,widht,height
SM40BL_holes = [2,2,16,12];             %x_num,y_num,x_dis,y_dis
SM85CL_holes = [2,3,28,28];             %x_num,y_num,x_dis,y_dis

box_height = 70;


%%Bottom
SG_servo_guides = [];
SG_side_holes = [];
PL_top_holes = [];
SG_top_holes = [];
SG_sm40_holes = SGofCPLy(CPLcopypattern(PLcircle(1.5),SM40BL_holes(1:2),SM40BL_holes(3:4)),20);
SG_sm85_holes = SGofCPLz(CPLcopypattern(PLcircle(1.5),SM85CL_holes(1:2),SM85CL_holes(3:4)),20);
distance_next_y = 0;
distance_next_x = 0;
for i=1:dofs
    if ~ismember(i,tensioner)
        PL_side = PLtrans(CPLbool('-',PLsquare(20,SM40BL_dimensions(1)+10),PLtrans(PLsquare(10,SM40BL_dimensions(1)+10),[5 10])),[-SM40BL_dimensions(2)/2 0]);
    else
        PL_side = PLtrans(CPLbool('-',PLsquare(20,SM85CL_dimensions(1)+10),PLtrans(PLsquare(10,SM85CL_dimensions(1)+10),[5 10])),[-SM85CL_dimensions(2)/2 0]);
    end
    PL_guide = [PL_side;NaN NaN;VLswapX(PL_side)];
    SG_guide = SGofCPLz(PL_guide,15);
    if(i==1)
        SG_side_holes = SGtransrelSG(SGbox([30,20,box_height/2]),SG_guide,'centerx','aligntop',-box_height/5,'infront',-2);
    end
    [sizex,~,~,~,~,~] = sizeVL(SG_guide);
    distance_next = (2*(rotor_radius-(sizex/2)))+10;
    if ~isempty(SG_servo_guides)
        SG_guide = SGtransrelSG(SG_guide,SG_servo_guides,'left',distance_next,'alignback');
        SG_servo_guides = SGcat(SG_servo_guides,SG_guide);
        distance_next_x = distance_next_x+rotor_radius*2+10;
    else
        SG_servo_guides = SG_guide;
    end
    if ~ismember(i,tensioner)
        distance_next_y = -7.25;
        if ~isempty(SG_side_holes)
            SG_holes = SGtransrelSG(SG_sm40_holes,SG_guide,'centerx','aligntop',-9.5,'alignback',15);
            SG_side_holes = SGcat(SG_side_holes,SG_holes);
        else
            SG_side_holes =  SGtransrelSG(SG_sm40_holes,SG_guide,'centerx','aligntop',-9.5,'alignback',15);
        end
        SG_hole_t = SGtransrelSG(SGofCPLz(PLcircle(14),20),SG_guide,'ontop',-5,'centerx','alignfront',-7.25);
    else        
        distance_next_y = -9;
        SG_hole_t = SGtransrelSG(SGofCPLz(PLcircle(16),20),SG_guide,'ontop',-2.5,'centerx','alignfront',-9);
        SG_holes = SGtransrelSG(SG_sm85_holes,SG_hole_t,'aligntop','centerx','centery',14);
        SG_hole_t = SGcat(SG_hole_t,SG_holes);        
    end
     if isempty(SG_top_holes)            
           SG_top_holes = SG_hole_t;
        else
           SG_top_holes = SGcat(SG_top_holes,SG_hole_t);
     end    
     PL_top_holes = [PL_top_holes;distance_next_x distance_next_y];
end
[sizex,sizey,~,~,~,~] = sizeVL(SG_servo_guides);
PL_main_frame = CPLbool('-',PLsquare(sizex+20,sizey+20),PLsquare(sizex,sizey));
PL_main_frame = PLroundcorners(PL_main_frame,2,10);
SG_main_frame = SGofCPLz(PL_main_frame,box_height);
SG_main_frame = SGtransrelSG(SG_main_frame,SG_servo_guides,'centerx','alignback',10,'aligntop');
SG_main_frame = SGboolh('-',SG_main_frame,SG_side_holes);
SG_main_frame_top = SGofCPLz(PLroundcorners(PLsquare(sizex+20,sizey+10),4,10),10);
SG_main_frame_top = SGtransrelSG(SG_main_frame_top,SG_main_frame,'ontop','alignright','alignback');
SG_main_frame_top = SGboolh('-',SG_main_frame_top,SG_top_holes);


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

SG_top_cover = SGtransrelSG(SGbox([57,66.5,5]),SG_tool_mover_connection,'aligntop',30,'alignleft','centery',-10);
PL_side_covers = [PLsquare(42.75,25);NaN NaN;PLtrans(CPLcopypattern(PLcircle(1.5),[1 2],[0 12]),[13.375 -9.5])];
SG_side_covers = SGofCPLy(PL_side_covers,4.7);
SG_side_covers = SGtransrelSG(SG_side_covers,SG_top_cover,'under','alignleft','alignback');
SG_side_covers_2 = SGtransrelSG(SG_side_covers,SG_top_cover,'under','alignleft','alignfront');
SG_side_covers = SGcat(SG_side_covers,SG_side_covers_2);



%% Rotors

SG_sm40_rotor = SGofCPLcommand('c 25,d 3 7 0,d 3 -7 0,d 3 0 7,d 3 0 -7,c 8,h 7,enter,c 19,c 25,h 2.5,rel under 0,cat,col y');
SG_cover_rotor = SGofCPLz([PLcircle(17);NaN NaN;PLcircle(1.5);NaN NaN;CPLcopyradial(PLcircle(1.5),7,4)],2);
PL_rotor_conn_bot = [PLsquare(22);NaN NaN;PLcircle(1.5);NaN NaN;CPLcopyradial(PLcircle(1.5),7,4)];
SG_rotor_conn_bot = SGofCPLz(PL_rotor_conn_bot,2.5);
PL_rotor_conn_top = [PLsquare(22);NaN NaN;PLcircle(3);NaN NaN;CPLcopyradial(PLcircle(3),7,4)];
SG_rotor_conn_top = SGofCPLz(PL_rotor_conn_top,2.5);
SG_sm40_rotor = SGstack('z',SG_sm40_rotor,SG_cover_rotor,SG_rotor_conn_bot,SG_rotor_conn_top);

SG_sm40_rotor = SGtransrelSG(SG_sm40_rotor,SG_main_frame_top,'ontop',-8,'transy',-7.5);

SG_sm85_rotor = SGofCPLcommand('c 30,d 3 10.5 0,d 3 -10.5 0,d 3 0 10.5,d 3 0 -10.5,c 13.5,h 7,enter,c 26,c 30,h 2.5,rel under 0,cat,col y');
SG_cover_rotor_85 = SGofCPLz([PLcircle(17);NaN NaN;PLcircle(1.5);NaN NaN;CPLcopyradial(PLcircle(1.5),10.5,4)],2);
PL_rotor_conn_bot_85 = [PLcircle(17,5);NaN NaN;PLcircle(1.5);NaN NaN;CPLcopyradial(PLcircle(1.5),10.5,4)];
SG_rotor_conn_bot_85 = SGofCPLz(PL_rotor_conn_bot_85,2.5);
PL_rotor_conn_top_85 = [PLcircle(17,5);NaN NaN;PLcircle(3);NaN NaN;CPLcopyradial(PLcircle(3),10.5,4)];
SG_rotor_conn_top_85 = SGofCPLz(PL_rotor_conn_top_85,2.5);

SG_sm85_rotor = SGstack('z',SG_sm85_rotor,SG_cover_rotor_85,SG_rotor_conn_bot_85,SG_rotor_conn_top_85);
SG_sm85_rotor = SGtransrelSG(SG_sm85_rotor,SG_main_frame_top,'ontop',-8,'transy',-21,'transx',-117.25);


%% Top plate
SG_rotor_top = SGtrans(SGservorotor(25,'',''),TofR(rotz(110)));
%top sm40
SG_connection_top = SGofCPLz([PLcircle(17);NaN NaN;PLsquare(22.1)],4.5);
SG_connection_top_top = SGofCPLz(PLcircle(17),2);
SG_connection_top_through = SGofCPLz(PLcircle(10),6);
SG_connection_top = SGstack('z',SG_connection_top,SG_connection_top_top,SG_connection_top_through,SG_rotor_top);
SG_connection_top = SGtransrelSG(SG_connection_top,SG_sm40_rotor,'center','ontop',-4);

%top sm85
SG_connection_top_85 = SGofCPLz([PLcircle(17.1,5);NaN NaN;PLcircle(19)],4.5);
SG_connection_top_top_85 = SGofCPLz(PLcircle(19),2);
SG_connection_top_through_85 = SGofCPLz(PLcircle(10),6);
SG_connection_top_85 = SGstack('z',SG_connection_top_85,SG_connection_top_top_85,SG_connection_top_through_85,SG_rotor_top);
SG_connection_top_85 = SGtransrelSG(SG_connection_top_85,SG_sm40_rotor,'center','ontop',-4,'transx',-120,'transy',-13.5);


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
SG_bottom = SGcat(SG_servo_guides,SG_main_frame,SG_main_frame_top,SG_back_plate,SG_bottom_plate,SG_guide_bot,SG_guide_top,SG_arduino_mounting,SG_feetech_mounting,SG_tool_mover_connection,SG_top_cover,SG_side_covers);
SG_cover = SGcat(SG_cover,SG_guide_cover_top,SG_guide_cover_bot,SG_guide_cover_top_add,SG_cover_plate);
SG_top_plate = SGcat(SG_connection_top,SG_connection_top_85);
SG = SGcat(SG_bottom,SG_sm40_rotor,SG_sm85_rotor,SG_top_plate);
% SGwriteSTL(SG_sm40_rotor,"SG_rotor_sm40");
% SGwriteSTL(SG_cover,"SG_cover");
% SGwriteSTL(SG_bottom,"SG_bottom");
SGwriteSTL(SG,"SG_box");
end