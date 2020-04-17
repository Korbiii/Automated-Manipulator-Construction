%%   [SG] = SGmanipulatorbase(CPL,middle_tool_r,tool_pos)
%	=== INPUT PARAMETERS ===
%	CPL:            CPL of bottomelements
%   CPL_out:        CPL of outer contour of elements 
%   tool_radius:    Radius for tool
%   tool_pos:       0: middle 1: top
%	=== OUTPUT RESULTS ======
%	SG:             SG of Manipulatorbase
function [SG] = SGmanipulatorbase(CPL,varargin)
optic_radius=3.25;      if nargin>=2 && ~isempty(varargin{1});  optic_radius=varargin{1};   end
first_positions=[];           if nargin>=3 && ~isempty(varargin{2});  first_positions=varargin{2};     end
sensor_channel=0;       if nargin>=4 && ~isempty(varargin{3});  sensor_channel=varargin{3}; end
optic_channel = 0;      if nargin>=5 && ~isempty(varargin{4});  optic_channel=varargin{4};  end
single = 0;             if nargin>=6 && ~isempty(varargin{5});  single=varargin{5};         end
length = 50;             if nargin>=7 && ~isempty(varargin{6});  length=varargin{6};         end
seal = 0;               if nargin>=8 && ~isempty(varargin{7});  seal=varargin{7};           end
radial = 0;             if nargin>=9 && ~isempty(varargin{8});  radial=varargin{8};           end
%%Generate base CPL based on CPL of bottom element

offset = [0 0];
mid_points = [];
CPL_optic = [];
if optic_channel ~= 0
    CPL_optic = PLcircle(optic_radius,optic_radius*20);
    if optic_channel == 1
        offset = [optic_radius 0];  
    elseif optic_channel == 2
        offset = [0 optic_radius];
    end
end

num_arms = size(CPL,2);
if ~radial
    [x1,y1,~,~,~,~] = sizeVL(CPL{1});    
    CPL{1} = PLtransR(CPL{1},rot(pi));  
    mid_points = [-x1/2-offset(1)-0.5 offset(2)];
    CPL{1} = PLtrans(CPL{1},mid_points);
    if optic_channel == 2  
        CPL{1} = PLtrans(CPL{1},[0 y1/2]);
        mid_points = mid_points + [0 y1/2];
    end
    if size(CPL,2)>1
        [x2,y2,~,~,~,~] = sizeVL(CPL{2});
        mid_points = [mid_points;x2/2+offset(1)+0.5 offset(2)];
        CPL{2} = PLtrans(CPL{2},mid_points(2,:));          
        if optic_channel == 2   
            CPL{2} = PLtrans(CPL{2},[0 y2/2]);
            mid_points(2,:) = mid_points(2,:)+[0 y2/2];
        end
        if y2<y1
            CPL{2} = PLtrans(CPL{2},[0 abs(y2-y1)/2]);             
            mid_points(2,:) = mid_points(2,:)+[0 abs(y2-y1)/2];
        elseif y2>y1
            CPL{1} = PLtrans(CPL{1},[0 abs(y2-y1)/2]); 
            mid_points(1,:) = mid_points(1,:)+[0 abs(y2-y1)/2];
        end
    end
    if size(CPL,2)>2
        CPL{3} = PLtransR(CPL{3},rot(pi/2));   
        [~,y3,~,~,~,~] = sizeVL(CPL{3});
        mid_points = [mid_points;0 max(y1/2,y2/2)+y3/2];
        CPL{3} = PLtrans(CPL{3},mid_points(3,:)); 
        if optic_channel == 2   
            CPL{3} = PLtrans(CPL{3},[0 +offset(2)+0.5+max(y1/2,y2/2)]); 
            mid_points(3,:) = mid_points(3,:)+[0 +offset(2)+0.5+max(y1/2,y2/2)];
        end
    end
    if size(CPL,2)>3
        CPL{4} = PLtransR(CPL{4},rot(-pi/2));
        [~,y4,~,~,~,~] = sizeVL(CPL{4});       
        mid_points = [mid_points;0 -max(y1/2,y2/2)-y4/2];
        CPL{4} = PLtrans(CPL{4},mid_points(4,:));  
    end
else
    edges = []; edges_x = [];
    for i=1:num_arms
        [~,y_size,~,~,~,~] = sizeVL(CPL{i});
        edges = [edges; y_size];
        edges_x = [edges_x; 0];
    end 
    max_size = max(edges);
    radius_poly = max_size/(2*sind(180/size(edges,1)));
    for i=1:num_arms
        CPL{i} = PLtrans(CPL{i},[radius_poly+edges_x(i)/2 0]);
        CPL{i} = PLtransC(CPL{i},[0 0],rot((i-1)*(2*pi)/size(edges,1)));        
        mid_points =[mid_points; PLtransC([+radius_poly+edges_x(i)/2 0],[0 0],rot((i-1)*(2*pi)/size(edges,1)))];  
    end
    
    
end
if ~isempty(CPL_optic)
CPL{end+1} = PLgrow(CPL_optic,0.5);
end
CPL_base =[];
for i=1:size(CPL,2)
    CPL_base = CPLbool('+',CPL_base,CPL{i});
end
CPL_only_holes = CPLselectinout(CPL_base,1);
CPL_holes = CPLbool('+',CPL_only_holes,CPL_optic);
CPL_out = CPLconvexhull(CPL_base);
CPL = CPLbool('-',CPL_out,CPL_holes);

for i=1:num_arms
    PL_crimp_holes = PLtrans(PLcircle(1.2),-first_positions(i,:));
    if ~single
        PL_crimp_holes = [PL_crimp_holes;NaN NaN;PLtransR(PL_crimp_holes,rot(pi))];
    end
    PL_crimp_holes = PLtrans(PL_crimp_holes,mid_points(1,:));
    PL_crimp_holes = PLtransR( PL_crimp_holes,rot((i-1)*(2*pi)/size(edges,1)));    
    CPL = CPLbool('-',CPL,PL_crimp_holes);
end




%%
[sizex,sizey,~,minx,miny,~] = sizeVL(CPL);
mid_point = [minx+sizex/2 miny+sizex/2];
sizeCPL = sizeVL(CPL);
PL_cut_x = PLsquare(sizex+10,(sizey-3)/3);
PL_cut_x = PLtrans([PLtrans(PL_cut_x,[0 (sizey-9)/3]);NaN NaN;PLtrans(PL_cut_x,[0 -(sizey-9)/3])],[0 mean(sizeCPL(3:4))]);
PL_cut_x = PLtrans(PL_cut_x,mid_point);

PL_cut_y = [sizex/6 sizey+10;-sizex/6 sizey;-sizex/6 -sizey;sizex/6 -sizey-10];
PL_cut_y = [PLtrans(PL_cut_y,[sizex/4 0]);NaN NaN;PLtrans(PL_cut_y,[-sizex/4 0])];
PL_cut_y = PLtrans(PL_cut_y,mid_point);


if length > 1
    CPL_w_cuts_x = CPLbool('-',CPL,PL_cut_x);
    CPL_w_cuts_y = CPLbool('-',CPL,PL_cut_y);
else
    CPL_w_cuts_y = CPL;
    CPL_w_cuts_x = CPL;
end

SG_cutless = SGofCPLz(CPL,2);
SG_w_cuts_x = SGofCPLz(CPL_w_cuts_x,2);
SG_w_cuts_y = SGofCPLz(CPL_w_cuts_y,2);

SG = SGstack('z',SG_cutless,SG_w_cuts_x,SG_cutless,SG_w_cuts_y,SG_cutless);
SG = SGstackn(SG,length,0);

if seal
    max_value = max(sizex,sizey)/2;
    CPL_out = CPLselectinout(CPL,0);
    CPL_add_on = [CPL_out;NaN NaN;PLcircle(max_value+2)];
    CPL_bayo_holder = [CPL_out;NaN NaN;PLcircle(max_value+4)];
    CPL_add_on_w_cuts_y = CPLbool('-',CPL_add_on,PL_cut_x);
    CPL_add_on_w_cuts_x = CPLbool('-',CPL_add_on,PL_cut_y);
    SG_add_on = SGofCPLz(CPL_add_on,2);
    SG_add_on_w_cuts_y = SGofCPLz(CPL_add_on_w_cuts_y,2);
    SG_add_on_w_cuts_x = SGofCPLz(CPL_add_on_w_cuts_x,2);
    SG_bayo_holder = SGofCPLz(CPL_bayo_holder,4);
    SG_add_on_stack = SGstack('z',SG_add_on,SG_add_on_w_cuts_y,SG_add_on,SG_add_on_w_cuts_x,SG_add_on,SG_bayo_holder);
    SG_add_on_stack = SGtransrelSG(SG_add_on_stack,SG,'alignbottom');
    
    
    PL_bayo_grip = [PLcircle(max_value+8);NaN NaN;PLcircle(max_value+12)];
    PL_bayo_grip = CPLbool('-',PL_bayo_grip,CPLcopyradial(PLcircle(8),max_value+18,8));
    SG_bayo_grip = SGofCPLz(PL_bayo_grip,12);
    SG_bayo_top_ring = SGofCPLz([PLcircle(max_value+2);NaN NaN;PLcircle(max_value+8)],2);
    SG_bayo_top_ring = SGontop(SG_bayo_top_ring,SG_add_on_stack);
    SG_bayo_grip = SGaligntop(SG_bayo_grip,SG_bayo_top_ring);
    
    PL_square = PLtrans(PLsquare(max_value*10,max_value/2),[max_value*10/2,max_value/4]);
    PL_bayo_middle = PL_square;
    for k = 1:4
        PL_bayo_middle =  CPLbool('+',PL_bayo_middle,PLtransC(PL_square,[0 0],rot((pi/2)*k)));
    end
    
    PL_bayo_middle = CPLbool('-',PL_bayo_middle,PLcircle(max_value+4.15));
    PL_bayo_middle = CPLbool('-',PL_bayo_middle,[PLcircle(max_value*100);NaN NaN;PLcircle(max_value+8)]);
    SG_bayo_middle = SGofCPLz(PL_bayo_middle,6);
    SG_bayo_middle = SGunder(SG_bayo_middle,SG_bayo_top_ring);
    
    
    PL_bayo_bottom = CPLbool('+',PLsquare(max_value*10,max_value),PLsquare(max_value,max_value*10));
    PL_bayo_bottom = CPLbool('-',PL_bayo_bottom,PLcircle(max_value+4.15));
    PL_bayo_bottom = CPLbool('-',PL_bayo_bottom,[PLcircle(max_value*100);NaN NaN;PLcircle(max_value+8)]);
    SG_bayo_bottom = SGofCPLz(PL_bayo_bottom,4);
    SG_bayo_bottom = SGunder(SG_bayo_bottom,SG_bayo_middle);
    
    SG_bayonett = SGcat(SG_bayo_grip,SG_bayo_bottom,SG_bayo_middle,SG_bayo_top_ring);
    
    SG = SGcat(SG,SG_add_on_stack);   
    
    
    
    CPL_seal_cup = [PLcircle(max_value+2.15);NaN NaN;PLcircle(max_value+4)];
    CPL_seal_cup_bottom = CPLbool('-',PLcircle(max_value+4),CPL_only_holes);
    SG_seal_cup = SGofCPLz(CPL_seal_cup,8);
    SG_seal_cup_bottom = SGofCPLz(CPL_seal_cup_bottom,3);
    SG_seal_cup = SGstack('z',SG_seal_cup_bottom,SG_seal_cup);
    SG_seal_cup = SGtransrelSG(SG_seal_cup,SG,'alignbottom',2);


end



%% Add Frames
height_SG = abs(max(SG.VL(:,3))-min(SG.VL(:,3)));

H_b_b = [rotx(0) [0;0;0]; 0 0 0 1];
SG = SGTset(SG,'B',H_b_b);
H_f_b = [rotx(90)*roty(180) [mid_points(1,:)';height_SG]; 0 0 0 1];
SG = SGTset(SG,'F',H_f_b);
if ~radial
if size(mid_points,1)>1
    H_f1_b = [rotx(90) [mid_points(2,:)';height_SG]; 0 0 0 1];
    SG = SGTset(SG,'F1',H_f1_b);
end
if size(mid_points,1)>2    
    H_f2_b = [rotx(90) [mid_points(3,:)';height_SG]; 0 0 0 1];
    SG = SGTset(SG,'F2',H_f2_b);
end
if size(mid_points,1)>3
    H_f3_b = [rotx(90)*roty(-90) [mid_points(4,:)';height_SG]; 0 0 0 1];
    SG = SGTset(SG,'F3',H_f3_b);
end
else
    for i=2:num_arms
        H_f_b = [rotx(90)*roty(180)*roty((i-1)*(360/num_arms)) [mid_points(i,:)';height_SG]; 0 0 0 1];
        SG = SGTset(SG,['F' num2str(i-1)],H_f_b);
    end
end

 
if seal
    SG = SGcat(SG,SG_seal_cup);
end
end