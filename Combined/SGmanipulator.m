%%   [SG,SGc,ranges] = SGmanipulator(CPL_out,tool_d,angle_p,length_p,flags) 
%	=== INPUT PARAMETERS ===
%	CPL_out:            Nested Cell list with CPLs of every Section
%   tool_d:             1xn Array of diameters of tools used per arm
%	angle_p:            Nested cell list with nx4 matrices containing angle information per section [xy-angle,positive_angle,negative_angle,optimization_p]
%                       optimization_p: 1/-1 moving hinge to edges; 2 Creating 2 axis section
%   length_p:           Nested celllist with nx5 matrices containing length information per section [section_length element_height,hinge_width,hinge_length,hinge_height]
%   === FLAGS ==============
%   'single'            Creates Manipulator with push-pull Configuration
%   'first_single'      Creates Manipulator with double-pull COnfiguration and first section as single pull
%   'optic_mid'         Creates a channel for a rigid optic in the middle
%   'optic_top'         Creates a channel for a rigid optic on top
%   'optic_radius'      Following value defines radius for optic channel
%   'length'            Sets the length of the rigid base
%   'seal'              Adds a seal 
%   'torsion'           Optimizes channels for torsion
%   'bottom_up'         Starts channel optimization at the bottom of the arms
%   'angles'            Following matrix sets angles of SGTchain in percent
%   'hole_radius'       Sets Bowdencable radius
%   'symmetric'         Creates symmetric two armed manipulator based on one arm definition
%   'radial'            Radial placement of arms
%   'sensor_channel':   Creates a sesor channel for EMT
%   'side_stabi':       Creating side stabilizing elements
%   'tip':              Creates only arms with minimal base
%   'flex'              Creates a flexbile base
%	=== OUTPUT RESULTS =====
%	SG:                 SG of Manipulator
%   SGc:                SGTchain of Manipulator
function [SG,SGc,ranges,framechain,phis] = SGmanipulator(CPL_out,tool_d,angle_p,length_p,varargin) 
base_length = 7;optic_radius = 2.2;hole_r = 0.7; num_arms = 2;
angle_defaults = [90 90 0];
length_defaults = [2 1 0 0.5];
angles = []; c_inputs = {}; CPLs = {};
if ~iscell(CPL_out)
    CPL_out = {CPL_out};
end
if ~iscell(angle_p)
    angle_p = {angle_p};
end
if ~iscell(length_p)
    length_p = {length_p};
end

[single,side_stabi,flex,flex_angle,sensor_channel,optic,seal,symmetric,torsion,bottom_up,radial] = deal(0);
for f=1:size(varargin,2)
if ~ischar(varargin{f}), continue; end
      switch varargin{f}
          case 'single'
              c_inputs{end+1} = 'single';
              single = 1;
              crimp = 0;
          case 'side_stabi'
              side_stabi = 1;
          case 'first_single'
              single = 2;
              crimp = 1;
              c_inputs{end+1} = 'crimp';
          case 'tip'
              base_length = 1;
          case 'optic_mid'
              optic = 1;
          case 'optic_top'
              optic = 2;     
          case 'optic_radius'
              optic_radius = varargin{f+1};
          case 'length'
              base_length = varargin{f+1};
          case 'seal'
              seal = 1;
          case 'torsion'
              torsion = 1;
          case 'bottom_up'
              bottom_up = 1;    
          case 'angles'
              angles = varargin{f+1};
          case 'hole_radius'
              hole_r = varargin{f+1};
          case 'symmetric'
              symmetric = 1;
              num_arms = 2;
          case 'radial'
              radial = 1;
          case 'flex'
              flex = 1;
              flex_angle = varargin{f+1};
          otherwise
              error(varargin{f} + " isn't a valid flag!!!!");            
      end   
end

%% Setting up variables

if symmetric
    angle_p = {angle_p{1},angle_p{1}};
    length_p = {length_p{1},length_p{1}};
    CPL_out = {CPL_out(1),CPL_out(1)};
    tool_d = [tool_d;tool_d];
end

num_sections = [];
tool_r = tool_d/2;
num_arms = size(angle_p,2);
for i=1:num_arms
    num_sections = [num_sections size(angle_p{i},1)];
end


angle_p = cell2mat(angle_p');
length_p = cell2mat(length_p');
angle_p = [angle_p repelem(angle_defaults(1,size(angle_p,2):end),size(angle_p,1),1)];
length_p = [length_p repelem(length_defaults(1,size(length_p,2):end),size(length_p,1),1)];

if num_arms == 1
    CPL_out = {[CPL_out;repmat(CPL_out(end),size(angle_p,1)-size(CPL_out,1),1)]};
    CPL_in = {PLcircle(tool_r,tool_r*20)};
    if num_arms == 2
        CPL_out{end+1} = CPL_out{end};
        CPL_in{end+1} = CPL_in{end};
        length_p = [length_p;length_p];
        angle_p = [angle_p;angle_p];
        tool_r = [tool_r;tool_r];
    end
else
    CPL_in = {};
    for i = 1:num_arms
        CPL_out{i} = [CPL_out{i};repmat(CPL_out{1,i}(end),num_sections(i)-size(CPL_out{1,i},1),1)];
        CPL_in{end+1} =  PLcircle(tool_r(i),tool_r(i)*20);
    end
end
for i=1:num_arms  
    CPLs_temp = {};
    for j=1:size(CPL_out{1,i},1)
        CPLs_temp{end+1,1} = CPLbool('-',CPL_out{1,i}{j},CPL_in{i});
    end
    CPLs{end+1} = CPLs_temp;
end                         
%% Initializing arrays and variables
length_p_temp = length_p;
angle_p_temp = angle_p; 
[arms,SG_elements,CPL_combis,CPLs_holes,offsets,positions,SG_conns,angle_p,length_p] = deal({});
start =1; ende = num_sections(1);
for i=1:num_arms          
    length_p{end+1} = length_p_temp(start:ende,:);    
    angle_p{end+1} = angle_p_temp(start:ende,:);
    start = ende+1;
    if i < num_arms ende = ende+num_sections(i+1); end
end
%% Finding Positions of holes and creating CPLs
for i=1:num_arms
    [CPLs_holes{end+1},positions_temp] = PLholeFinder(CPL_out{i},tool_r(i),angle_p{i}(:,[1,4]),length_p{i}(:,3:5),hole_r,single,torsion,bottom_up); 
    positions{end+1} = positions_temp;
end
disp("Bowden cable channels created");
%%  Creating elements and connectors
for k=1:num_arms
    SG_elements_temp = {}; 
    offsets_temp = [];
    SG_bottom = SGelements(CPLbool('-',CPLs{k}{1},CPLs_holes{k}{1}),angle_p{k}(1,[1,4]),length_p{k}(1,2:5),'bottom_element');
    SG_bottom.hole_positions = positions{k}{1};
    SG_bottom.angle = angle_p{k}(1,1);
    SG_conns_temp = {SG_bottom};
    for i=1:num_sections(k)
        CPL_combined = CPLbool('-',CPLs{k}{i},CPLs_holes{k}{i});
        if i==1 CPL_combis{end+1} = CPL_combined; end
         if angle_p{k}(i,4)==2
             angle_p{k}(i,1) = angle_p{k}(i,1)+90;
         end
        if i == num_sections(k)   %% Top of arms
            if single == 1
                SG_conn_temp = SGconnector(CPLs{k}(num_sections(k)),CPLs_holes{k}(end),positions{k}(end),[angle_p{k}(end,[1,4]);angle_p{k}(end,[1,4])],length_p{k}(i,3:5),hole_r,tool_r(k),{'end_cap','single'});
            else
                SG_conn_temp = SGconnector(CPLs{k}(num_sections(k)),CPLs_holes{k}(end),positions{k}(end),[angle_p{k}(end,[1,4]);angle_p{k}(end,[1,4])],length_p{k}(i,3:5),hole_r,tool_r(k),{'end_cap'});
            end
%             SG_conn_temp.hole_positions = positions{end,:}{1};   
            SG_conn_temp.angle = angle_p{k}(i,1);
        else
            SG_conn_temp = SGconnector(CPLs{k}(i:i+1),CPLs_holes{k}(i:i+1),positions{k}(i:i+1),angle_p{k}(i:i+1,[1,4]),length_p{k}(i:i+1,3:5),hole_r,tool_r(k),c_inputs);
%             SG_conn_temp = SGconnector(CPLs{k}(i:i+1),CPLs_holes{k}(i:i+1),positions{k}(i:i+1),angle_p{k}(i:i+1,[1,4]),hole_r,tool_r(k),length_p{k}(i,3),length_p{k}(i+1,3),length_p{k}(i,4),length_p{k}(i,5),length_p{k}(i+1,5),c_inputs);
%             SG_conn_temp.hole_positions = positions{k}{i:i+1}{1};
            SG_conn_temp.angle = [angle_p{k}(i,1);angle_p{k}(i+1,1)];
        end
        if angle_p{k}(i,4)==2
            angle_p{k}(i,1) = angle_p{k}(i,1)-90;
        end
        
        [SG_ele_temp, offset] = SGelements(CPL_combined,angle_p{k}(i,[1,4]),length_p{k}(i,2:5));
        if size(SG_ele_temp,2) == 1
            if i == num_sections(k)
                SG_ele_temp.hole_positions = positions{k}{end};
                SG_ele_temp.angle = angle_p{k}(i,1);
            else
                SG_ele_temp.hole_positions = positions{k}{i:i+1};
                SG_ele_temp.angle = angle_p{k}(i,1);
            end
        else
            SG_ele_temp{1}.hole_positions = positions{k}{end};            
            SG_ele_temp{2}.hole_positions = positions{k}{end};
            SG_ele_temp{1}.angle = angle_p{k}(i,1);
            SG_ele_temp{2}.angle = angle_p{k}(i,1)-90;
        end
        if ~isempty(SG_conn_temp) SG_conns_temp{end+1} = SG_conn_temp;   SG_conn_temp = []; end
        SG_elements_temp{end+1} = SG_ele_temp;
        offsets_temp = [offsets_temp offset];
        disp("Element "+ i + " and Connector " + i + " for arm "+k+" created!");
    end
    SG_elements{end+1} = SG_elements_temp;
    SG_conns{end+1} = SG_conns_temp;
    offsets{end+1} = offsets_temp;
end


%% Calculating number of elements && adding stops
[ele_num,angles_sections] = deal({});
ranges = [];
for k = 1:num_arms
    length_p{k} = [0 0 0 0 0;length_p{k};0 0 0 0 0];
    angle_p{k} = [0 0 0 0;angle_p{k};0 0 0 0];
end
for k = 1:num_arms
    ele_num_temp = [];
    angles_sections_temp = [];
    for i=2:num_sections(k)+1
        extra_element = 0;
        if angle_p{k}(i,4) == 2
            extra_element =1;
        end
        ele_num_temp = [ele_num_temp extra_element + floor(length_p{k}(i,1)/(length_p{k}(i,2)+(2*length_p{k}(i,5))))];
        max_dim = max(sizeVL(CPL_out{k}{i-1}))+1;
        middle_axis = PLtransR(PLtrans([-max_dim 0;max_dim 0],[0 offsets{k}(i-1)]),rot(-deg2rad(angle_p{k}(i,1))));
        
        phi_left = max(1,(angle_p{k}(i,2)/ele_num_temp(i-1)-1)/2);
        phi_right = max(1,(angle_p{k}(i,3)/ele_num_temp(i-1)-1)/2);
        if angle_p{k}(i,4) == 2
            SG_el = SGstops(flip(SG_elements{k}{i-1}),CPL_out{k}{i-1},-angle_p{k}(i,1),offsets{k}(i-1),phi_left,phi_right,repelem(length_p{k}(i,[3,5]),3,1));
            SG_el = SGstops(flip(SG_el'),CPL_out{k}{i-1},-angle_p{k}(i,1)+90,offsets{k}(i-1),phi_left,phi_right,repelem(length_p{k}(i,[3,5]),3,1));
            SG_elements{k}{i-1} = SG_el;          
            
            [SG_con] = SGstops(SG_conns{k}(i-1:i),CPL_out{k}{i-1},-angle_p{k}(i,1),offsets{k}(i-1),phi_left,phi_right,length_p{k}(i-1:i+1,[3,5]),2);
      
        else
            SG_el = SGstops(SG_elements{k}{i-1},CPL_out{k}{i-1},-angle_p{k}(i,1),offsets{k}(i-1),phi_left,phi_right,length_p{k}(i,[3,5]));
            SG_elements{k}{i-1} = SG_el;
             [SG_con] = SGstops(SG_conns{k}(i-1:i),CPL_out{k}{i-1},-angle_p{k}(i,1),offsets{k}(i-1),phi_left,phi_right,length_p{k}(i-1:i+1,[3,5]));
      
        end
        SG_conns{k}{i-1} = SG_con{1};
        SG_conns{k}{i} = SG_con{2};
        if angle_p{k}(i,4) == 2            
            %ranges = [ranges; []]
            dis_axis_pos_1 = distPointLine(middle_axis,positions{k}{i-1}(1,:));
            dis_axis_pos_2 = distPointLine(middle_axis*rot(pi/2),positions{k}{i-1}(2,:));
            height_1 = 2*(tand(phi_right)*dis_axis_pos_1);
            height_2 = 2*(tand(phi_left)*dis_axis_pos_2);
            
            ranges = [ranges;max(0,height_1*ele_num_temp(i-1)),max(0,height_1*ele_num_temp(i-1));max(0,height_2*ele_num_temp(i-1)),max(0,height_2*ele_num_temp(i-1))];
            SG_elements{k}{i-1}{1}.phi =  [phi_left*2,phi_right*2];
             SG_elements{k}{i-1}{2}.phi =  [phi_left*2,phi_right*2];            
        else
            dis_axis_pos = distPointLine(middle_axis,positions{k}{i-1}(1,:));
            height_l = 2*(tand(phi_right)*dis_axis_pos);
            height_r = 2*(tand(phi_left)*dis_axis_pos);
            
            ranges = [ranges;max(0,height_l*ele_num_temp(i-1)),max(0,height_r*ele_num_temp(i-1))];
            SG_elements{k}{i-1}.phi =  [phi_left*2,phi_right*2];
        end        
            SG_conns{k}{i-1}.phi_t = [phi_left*2,phi_right*2];
            SG_conns{k}{i}.phi_b = [phi_left*2,phi_right*2];
            angles_sections_temp = [angles_sections_temp;phi_left*2,phi_right*2];        
    end
    disp("Added Stops to arm " + k);
    angles_sections{end+1} = angles_sections_temp;
    ele_num{end+1} = ele_num_temp;
end
%% Filling cell list with elements for a single arm
for k=1:num_arms
    arm_temp = [];
    for j=1:num_sections(k)
        if angle_p{k}(j+1,4) == 2
            arm_temp = [arm_temp repmat(SG_elements{k}{j}',1,floor(ele_num{k}(j)/2))];
            arm_temp = [arm_temp SG_elements{k}{j}(1)];
        else
            arm_temp = [arm_temp repelem(SG_elements{k}(j),ele_num{k}(j))];
        end
        arm_temp = [arm_temp  SG_conns{k}(j+1)];
    end    
    arms{end+1} = [SG_conns{k}(1) arm_temp];
end
%% Adding base to arms in a cell list
first_positions = [];
% for i=1:num_arms
%     first_positions = [first_positions;positions{i}{1}];
% end
base = SGmanipulatorbase(CPL_combis,optic_radius,positions,sensor_channel,optic,single,base_length,seal,radial,flex,flex_angle);
base.numArms = num_arms;
base.numSections =  num_sections;
base.numElements = ele_num;
SGs = [{base} arms{:}];
num = size(SGs,2);
disp("Created Base");
%% Generating full manipulator with framechain
phis = {};
if isempty(angles)
    for i=1:num_arms
        phis{end+1} = zeros(1,size(arms{i},2));
    end
else
    for k=1:num_arms
        phis{end+1} = 0;
        for i=1:num_sections(k)
            if angles(1,i)>=0                
                phis{end+1} = repmat(angles_sections{k}(i,2)*angles(k,i),1,ele_num{k}(i)+1);                
            else
                if angle_p{k}(i+1,4) == 2
                    phis{end+1} = repmat([angles_sections{k}(i,2)*angles(k,i),angles_sections{k}(i,2)*angles(k,i)*-1],1,(ele_num{k}(i)/2)+1);
                else
                    phis{end+1} = repmat(angles_sections{k}(i,1)*angles(k,i),1,ele_num{k}(i)+1);
                end
            end
        end
    end
end
ele_num_sections = [];
for k=1:num_arms
    ele_num_sections = [ele_num_sections sum(ele_num{k})+num_sections(k)];
end
if num_arms > 1
    framechain = SGTframeChain2(num,ele_num_sections(1:end-1)+1);
else
    framechain = SGTframeChain(num);
end

phis = deg2rad(cell2mat(phis));
SGc = SGTchain(SGs,[0 phis],'',framechain);
disp("Created SGTchain");
SG = SGc;

[~, userdir] = system('echo %USERPROFILE%');
userdir(end) = [];
path = strsplit(userdir, '\');
path{end+1} = 'Desktop\Manipulator.txt';
path = strjoin(path,'\'); 
fileID = fopen(path,'w');

fprintf(fileID,'\n');
fprintf(fileID,'IDs der Servomotoren mit folgendem Code durch Matlab ersetzen\n');
fprintf(fileID,'\n');
fprintf(fileID,'Bus = FTBus("COMXX",BAUDRATE); \t// BAUDRATE = 115200 standardmäßig/ COM-Port aus Gerätemanager  \n');
fprintf(fileID,'Servo = SM[C/B]LServo(Bus,CURRENT_ID); \t//Je nach Servotyp C oder B einfügen \n');
fprintf(fileID,'Servo.cfgID = NEW_ID; \n');
fprintf(fileID,'Servo.Output(); \n');
fprintf(fileID,'\n\n');

fprintf(fileID,'Arduino Code mit folgendem Initialisierungscode ersetzen:\n\n');

id_start = 1;
fprintf(fileID,'int rotor_radius = 25;\n');
fprintf(fileID,'double ranges_mm[][2] = {');
for k=1:size(ranges,1)
    fprintf(fileID,'{%.2f,%.2f}',ranges(k,1),ranges(k,1));
end
fprintf(fileID,'};\n');
fprintf(fileID,'int ids[] = {');
for k=1:size(ranges,1)
    if k >1 fprintf(fileID,','); end
    fprintf(fileID,'%i',id_start+k);
end
fprintf(fileID,'};\n');

fprintf(fileID,'int fixed[] = {');
for k=1:size(ranges,1)
    if k >1 fprintf(fileID,','); end
    fprintf(fileID,'%i',0);
end
fprintf(fileID,'};\n');
fprintf(fileID,'int ord[] = {1,2,3,4,5,6};\n');
fprintf(fileID,'int servo_speed = 2000;\n');
fprintf(fileID,'\n\n');

fprintf(fileID,'\n');
fprintf(fileID,'Befehl zur Ansteuerung per MATLAB:\n');
fprintf(fileID,'\n');
fprintf(fileID,'SMControl([');
for k=1:size(ranges,1)    
    if k >1 fprintf(fileID,';'); end
    fprintf(fileID,'[%.2f,%.2f 0]',ranges(k,1),ranges(k,1));
end
fprintf(fileID,'],[');
id_start = 1;
for k=1:size(ranges,1)
    if k >1 fprintf(fileID,','); end
    fprintf(fileID,'%i',id_start+k);
end

fprintf(fileID,'],25,1,[1,2,3,4,5,6],1)\n');
fclose(fileID);

% SGplot(SG);
end

%%  [CPLs,positions] = PLholeFinder(CPL_out,CPL_in,h_axis,hole_r,push_rod)
%	=== INPUT PARAMETERS ===
%	CPL_out:    CPL of outer contour of elements
%	CPL_in:     CPL of tool hole
%	h_axis:   	2xn Vector of DoF axis
%   hole_r:     Radius for actuator rope
%   single:     1 = single;0 = double pull rope
%	=== OUTPUT RESULTS ======
%	CPLs:         SG of Manipulator
%	positions:    2xn vector of positions of holes
function [CPLs,positions] = PLholeFinder(CPL_out,tool_r,angle_p,length_p,hole_r,varargin)
single = 0; if nargin>=6 && ~isempty(varargin{1}); single=varargin{1}; end
torsion = 0; if nargin>=7 && ~isempty(varargin{2}); torsion=varargin{2}; end
bottom_up = 0; if nargin>=8 && ~isempty(varargin{3}); bottom_up=varargin{3}; end
%% Initializing
hinge_w = length_p(:,1)+(2*length_p(:,3));
min_len= length_p(:,2);
CPL_no_go_area = [];
CPLs = {};
CPL_holes = [];
CPL_holes_2 = [];
positions = {};
CPL_small_holes = {};
CPL_big_holes = {};
CPL_in = PLcircle(tool_r);
CPL_no_go_areas = {};
PL_hole = PLcircle(hole_r,40);
PL_hole_small = PLcircle(0.4);
if single PL_hole = PL_hole_small; end
%% Generating CPL of area where no holes can go based on axis constraints
for i=2:size(angle_p,1)
    if abs(angle_p(i,2)) ~= 1
        if torsion == 0
            CPL_axis_constraint = [tool_r+min_len(i) hinge_w(i)/2;tool_r+min_len(i) -hinge_w(i)/2;-tool_r-min_len(i) -hinge_w(i)/2;-tool_r-min_len(i) hinge_w(i)/2];
            CPL_axis_constraint = PLtransR(CPL_axis_constraint,rot(deg2rad(angle_p(i,1))));
            CPL_only_out = CPLselectinout(CPL_out{i},0);
            inside = inside2C(CPL_only_out,CPL_axis_constraint);
            if (inside(2) ~= 0 || inside(4) ~= 0  || inside(5) ~= 0)
                error("Not enough room for hinge in section " + i);
            end
        else
            curr_axis = PLtransR([-100 0;+100 0],rot(deg2rad(angle_p(i,1))));
            e_dir = curr_axis/norm(curr_axis);
            CPL_axis_constraint_n = [tool_r+min_len(i) hinge_w(i)/2;tool_r+min_len(i) -hinge_w(i)/2;tool_r -hinge_w(i)/2;tool_r hinge_w(i)/2];
            CPL_axis_constraint_n = PLtransR(CPL_axis_constraint_n,rot(deg2rad(angle_p(i,1))));
            CPL_axis_constraint_p = [-tool_r-min_len(i) hinge_w(i)/2;-tool_r-min_len(i) -hinge_w(i)/2;-tool_r -hinge_w(i)/2;-tool_r hinge_w(i)/2];
            CPL_axis_constraint_p = PLtransR(CPL_axis_constraint_p,rot(deg2rad(angle_p(i,1))));            
            hit = 0;
            while ~hit
                inside_n = inside2C(CPL_out{i},CPL_axis_constraint_n);
                inside_p =  inside2C(CPL_out{i},CPL_axis_constraint_p);
                if (inside_p(2) ~= 0 || inside_p(4) ~= 0  || inside_p(5) ~= 0)
                    CPL_axis_constraint_p = PLtrans(CPL_axis_constraint_p,-e_dir(1,:)*0.25);
                    CPL_axis_constraint_n = PLtrans(CPL_axis_constraint_n,-e_dir(2,:)*0.25);
                    hit = 1;
                else
                    if (inside_n(2) ~= 0 || inside_n(4) ~= 0  || inside_n(5) ~= 0)
                        CPL_axis_constraint_p = PLtrans(CPL_axis_constraint_p,-e_dir(1,:)*0.25);
                        CPL_axis_constraint_n = PLtrans(CPL_axis_constraint_n,-e_dir(2,:)*0.25);
                        hit = 1;
                    else
                        CPL_axis_constraint_n = PLtrans(CPL_axis_constraint_n,e_dir(2,:)*0.25);
                        CPL_axis_constraint_p = PLtrans(CPL_axis_constraint_p,e_dir(1,:)*0.25);
                    end
                end
            end
            CPL_axis_constraint = CPLbool('+',CPL_axis_constraint_p,CPL_axis_constraint_n);
        end
        CPL_no_go_area = CPLbool('+',CPL_no_go_area,CPL_axis_constraint);
        CPL_no_go_areas{end+1} = CPLgrow(CPL_no_go_area,-hole_r);
    else 
        CPL_no_go_areas{end+1} = [];
    end
end

max_dim = 2*max(sizeVL(CPL_out{size(angle_p,1)}));

%%Looping over each section
if bottom_up     
    CPL_no_go_areas{end+1} = [];
    start_value = 1;
    step = 1;
    end_value = size(angle_p,1);
else    
    CPL_no_go_areas =[{[]},CPL_no_go_areas];
    start_value = size(angle_p,1);
    step = -1;
    end_value = 1;
end

for i=start_value:step:end_value
    first_section = (i==1);
    %% calculating general values
    CPL_opti_area = [];
    curr_axis = PLtransR([-100 0;+100 0],rot(deg2rad(angle_p(i,1))));
    curr_mid_point = [0 0];
    %% Generating CPL half for hinge optimization
    if abs(angle_p(i,2)) == 1
        CPL_opti_area = [max_dim 0;-max_dim 0;-max_dim max_dim;max_dim max_dim];
        CPL_opti_area = PLtransC(CPL_opti_area,[0 0],rot(deg2rad(angle_p(i,1))));
        if angle_p(i,2) == 1
            CPL_opti_area = PLtransC(CPL_opti_area,[0 0],pi);
        end
    end
    %% Generating CPL of points where holes currently can go
    CPL_hull = flip(CPLconvexhull(CPL_out{i}));
    CPL_hull_stamp = CPLgrow(CPL_hull,-0.2);
    CPL_inner_CPLs = CPLbool('-',CPL_hull_stamp,CPL_out{i});
    CPL_o_in = CPLgrow(CPL_hull,-0.7-hole_r);
    CPL_all_in = [CPL_in;NaN NaN;flip(CPL_inner_CPLs)];
%     [~,pos]=separateNaN(CPL_all_in);
%     if size(pos,1) > 2
%         CPL_1 = CPLgrow(CPL_all_in(1:pos(2)-1,:),-0.5-hole_r);
%         CPL_2 = CPLgrow(CPL_all_in(pos(2)+1:end,:),+0.5+hole_r);
%         CPL_i_out = CPLbool('+',CPL_1,CPL_2);
%     else
%         CPL_i_out = CPLgrow(CPL_all_in,+0.3+hole_r);
%     end
    CPL_limit = CPL_o_in;
    CPL_limit = CPLbool('-',CPL_limit,CPL_no_go_areas{i});
    CPL_limit = CPLbool('-',CPL_limit,CPL_opti_area);
     for u=1:separateNaN(CPL_all_in)
            CPL_limit = CPLbool('-',CPL_limit,CPLgrow(separateNaN(CPL_all_in,u),+0.5+hole_r));
     end
     if ~isempty(CPL_holes)
         for u=1:separateNaN(CPL_holes)
             CPL_limit = CPLbool('-',CPL_limit,CPLgrow(separateNaN(CPL_holes,u),+0.5+hole_r));
         end
    end
    %% Searching point with furthest distant to axis that still can be mirrored to the other side
    if angle_p(i,2) == 2 
        num_iterations = 2; 
    else
        num_iterations = 1;
    end
  
    CPL_hole_positions_temp =[];
    for k=1:num_iterations
        
        if k == 2 && angle_p(i,2) == 2
            curr_axis = PLtransR(curr_axis,rot(pi/2));
        end
        %% Searching points    
        ordered_limit_points = [CPL_limit distLinePoint(curr_axis,CPL_limit)];
        ordered_limit_points = sortrows(ordered_limit_points,3,'descend');
        hole_position = [];
        CPL_limit_tol = CPLgrow(CPL_limit,-0.1);
        while isempty(hole_position) && size(CPL_limit,2) > 0            
            if ~isnan(ordered_limit_points(1,1))
                if single == 1 || (single == 2 && first_section)
                    hole_position = ordered_limit_points(1,[1,2]);
                else
                    is_inside = insideCPS(CPL_limit_tol,PLtransC(ordered_limit_points(1,[1,2]),curr_mid_point,pi));
                    if is_inside >= 0
                        hole_position = ordered_limit_points(1,[1,2]);
                    end
                end
            end
            ordered_limit_points(1,:) = [];
        end
        
        
        if isempty(hole_position) error("CPL für Sektion " + i + " zu klein"); end
        
        %% Adding Points
        if single == 1 || (single == 2 && first_section)
            if angle_p(i,2) == 2
                hole_positions = [hole_position;PLtransC(hole_position,curr_mid_point,pi)];
%                 hole_positions = hole_position;
            else                
                hole_positions = hole_position;
            end
        else
            hole_positions = [hole_position;PLtransC(hole_position,curr_mid_point,pi)];
        end
        
        if isempty(CPL_holes)
            if single == 1 || (single == 2 && first_section)
                CPL_holes = PLtrans(PL_hole,hole_positions(1,:));                
                CPL_holes_2 = PLtrans(PL_hole_small,hole_positions(1,:));
            else
                CPL_holes = [PLtrans(PL_hole,hole_positions(1,:));NaN NaN;PLtrans(PL_hole,hole_positions(2,:))];                
                CPL_holes_2 = [PLtrans(PL_hole_small,hole_positions(1,:));NaN NaN;PLtrans(PL_hole_small,hole_positions(2,:))];                
            end  
            if bottom_up
                CPL_small_holes{end+1} = CPL_holes_2;
            end
        else
            if single == 1 || (single == 2 && first_section) 
                PL_hole_small_temp = PLtrans(PL_hole_small,hole_positions(1,:));
                if k>1
                    CPL_holes_2 = [CPL_holes_2;NaN NaN;PL_hole_small_temp];
                else
                    CPL_holes_2 = [CPL_holes;NaN NaN;PL_hole_small_temp];
                end
                CPL_holes = [CPL_holes;NaN NaN;PLtrans(PL_hole,hole_positions(1,:))];           
            else
                PL_hole_small_temp = [PLtrans(PL_hole_small,hole_positions(1,:));NaN NaN;PLtrans(PL_hole_small,hole_positions(2,:))];
                if k>1
                    CPL_holes_2 = [CPL_holes_2;NaN NaN;PL_hole_small_temp];
                else
                    CPL_holes_2 = [CPL_holes;NaN NaN;PL_hole_small_temp];
                end
                CPL_holes = [CPL_holes;NaN NaN;PLtrans(PL_hole,hole_positions(1,:));NaN NaN;PLtrans(PL_hole,hole_positions(2,:))];
            end
            if bottom_up
                 CPL_small_holes{end+1} = PL_hole_small_temp;
            end
        end
          if angle_p(i,2) == 2          
              CPL_hole_positions_temp = [CPL_hole_positions_temp;hole_positions([1,2],:)];              
%               CPL_hole_positions_temp = [CPL_hole_positions_temp;hole_positions(1,:)];
          else              
              CPL_hole_positions_temp = [CPL_hole_positions_temp;hole_positions(1,:)];
          end
    end
    
    CPLs{end+1} = CPL_holes_2;
    positions{end+1} = CPL_hole_positions_temp;
end
if ~bottom_up
    CPLs = flip(CPLs);
    positions = flip(positions);
else
    CPLs = CPL_small_holes;
    for i=1:size(angle_p,1)
        for j = i+1:size(angle_p,1)
            CPLs{i} = CPLbool('+',CPLs{i},CPLgrow(CPLs{j},hole_r-0.4));
        end
    end    
end
end

%%  [SG] =  SGelements(CPL,angle_p,length_p,flags)
%	=== INPUT PARAMETERS ===
%	CPL:                CPL of element with tool hole
%   angle_p:            [hinge_dir, hinge_optimization]
%   length_p:           [element_height,hinge_width,min_length,hinge_height]
%   === FLAGS ==============
%   'bottom_element':   Creates element with only top hinge
%   'non_torsion':      
%	=== OUTPUT RESULTS =====
%	SG:             SG of element
function [SG,offset] = SGelements(CPL,angle_p,length_p,varargin)
%% Initializing
bottom_ele = 0; torsion = 1;
for f=1:size(varargin,2)
   switch varargin{f}
       case 'bottom_element'
           bottom_ele = 1;     
           if angle_p(2) == 2, angle_p(2) = 0; end
%            length_p(1) = 5;
       case 'non_torsion'
           torsion = 0;
   end
end

%% Creating base and zeroing it in z
SG = SGofCPLzdelaunayGrid(CPL, length_p(1),0.5,0.5);
hight_SG = abs(max(SG.VL(:,3))-min(SG.VL(:,3)));
SG = SGtrans(SG,[0 0 -hight_SG/2]);

%% Add hinge to base
SG_hinge = SGhingeround(length_p(3),length_p(2), length_p(4));

SG_hinge = SGtransR(SG_hinge,rotz(angle_p(1)));
SG_hinge_2 = SGtransR(SG_hinge,rotz(-90));
[SG_hinge,offset]= SGcreateHinge(CPL,SG_hinge,angle_p(1),angle_p(2),length_p(2),length_p(3), length_p(4));

if length_p(3) ~= 0
    cp = PLcrossCPLLine2(PLtransC([-500 0;500 0],[0 0],deg2rad(angle_p(1))),CPL);
    dis_in_lef = pdist2(cp(3,:),[0 0]);
    dis_in_rig = pdist2(cp(4,:),[0 0]);
    dis_out_lef = pdist2(cp(1,:),[0 0]);
    dis_out_rig = pdist2(cp(2,:),[0 0]);
    SG_hinge =  SGtransR(SG_hinge,rotz(-angle_p(1)));
    [distance_whole,~,~,~,~,~] = sizeVL(SG_hinge);
    if torsion
%         distance_whole = dis_out_lef+dis_out_rig;
        SG_hinge_l = SGcutend(SG_hinge,'right',distance_whole-length_p(3));
        SG_hinge_r =  SGcutend(SG_hinge,'left',distance_whole-length_p(3));
        SG_hinge = SGcat(SG_hinge_l,SG_hinge_r);
    else
        left_cut_distance = (dis_out_lef-dis_in_lef);
        right_cut_distance = (dis_out_rig-dis_in_rig);
        SG_hinge = SGcutend(SG_hinge,'left',left_cut_distance-length_p(3));
        SG_hinge = SGcutend(SG_hinge,'right',right_cut_distance-length_p(3));
    end
    SG_hinge =  SGtransR(SG_hinge,rotz(angle_p(1)));
end



SG_hinge_top = SGontop(SG_hinge,SG);
SG_hinge_bottom = SGmirror(SG_hinge_top,'xy');

if angle_p(2) == 2
    [SG_hinge_2,offset_2]= SGcreateHinge(CPL,SG_hinge_2,angle_p(1)-90,angle_p(2),length_p(2),length_p(3), length_p(4));   
    
    if length_p(3) ~= 0
        cp = PLcrossCPLLine2(PLtransC([-500 0;500 0],[0 0],deg2rad(-angle_p(1)+90)),CPL);
        dis_in_lef = pdist2(cp(3,:),[0 0]);
        dis_in_rig = pdist2(cp(4,:),[0 0]);
        dis_out_lef = pdist2(cp(1,:),[0 0]);
        dis_out_rig = pdist2(cp(2,:),[0 0]);
        SG_hinge_2 =  SGtransR(SG_hinge_2,rotz(-angle_p(1)+90));
        if torsion
            distance_whole = dis_out_lef+dis_out_rig;
            SG_hinge_l = SGcutend(SG_hinge_2,'right',distance_whole-length_p(3));
            SG_hinge_r =  SGcutend(SG_hinge_2,'left',distance_whole-length_p(3));
            SG_hinge_2 = SGcat(SG_hinge_l,SG_hinge_r);
        else
            left_cut_distance = (dis_out_lef-dis_in_lef);
            right_cut_distance = (dis_out_rig-dis_in_rig);
            SG_hinge_2 = SGcutend(SG_hinge_2,'left',left_cut_distance-length_p(3));
            SG_hinge_2 = SGcutend(SG_hinge_2,'right',right_cut_distance-length_p(3));
        end
        SG_hinge_2 =  SGtransR(SG_hinge_2,rotz(angle_p(1)-90));        
        
    end
    SG_hinge_2_top = SGontop(SG_hinge_2,SG);
    SG_hinge_2_bottom = SGmirror(SG_hinge_2_top,'xy');
end


if ~bottom_ele    
    if angle_p(2) == 2
        SG_base_ = SG;
        SG = SGcat(SG_base_,SG_hinge_bottom,SG_hinge_2_top);
        SG_2 = SGcat(SG_base_,SG_hinge_2_bottom,SG_hinge_top);
        
    else
        SG = SGcat(SG,SG_hinge_top,SG_hinge_bottom);
    end    
else
    SG = SGcat(SG,SG_hinge_top);
end

%% Add frames to element
e_dir = [sind(angle_p(1)) -cosd(angle_p(1))];
H_f = TofR(rotx(90)*roty(90+angle_p(1)),[offset*-e_dir  length_p(4)+(hight_SG/2)]);
H_f_2 = TofR(rotx(90)*roty(180+angle_p(1)),[offset*-e_dir length_p(4)+(hight_SG/2)]);
if ~bottom_ele
    H_b = TofR(rotx(90)*roty(-90+angle_p(1)),[offset*-e_dir  -(hight_SG/2)-length_p(4)]);
    H_b_2 = TofR(rotx(90)*roty(angle_p(1)),[offset*-e_dir  -(hight_SG/2)-length_p(4)]);
else
    H_b = [rotx(90)*roty(180) [0;0;(-(hight_SG/2))]; 0 0 0 1];
end
if angle_p(2) == 2    
    SG = SGTset(SG,'F',H_f_2);
    SG = SGTset(SG,'B',H_b);
    SG_2 = SGTset(SG_2,'F',H_f);
    SG_2 = SGTset(SG_2,'B',H_b_2);
    SG.offest = offset;
    SG_2.offset = offset_2;
    SG = {SG SG_2};
else
    SG = SGTset(SG,'F',H_f);
    SG = SGTset(SG,'B',H_b);
    SG.offest = offset;
end


end

%%   [SG] = SGconnector(CPL,CPL_holes,positions,angle_p,length_p,h_r,tool_radius,flags)
%	=== INPUT PARAMETERS ===
%	CPL:                CPL of sections to connect
%   CPL_holes:          CPL of holes of sections to connect
%	positions:          2xn vector of hole positions
%	angle_p:            2x2 Vector cotaining orientation of sections to connect [xy-Angle, optimization]
%	length_p:           3x2 Vector containing hinge parameters of sections to connect [Hinge_width, Mininum length of hinge, hinge_height]
%   h_r:                Radius of Bowdencable
%   tool_radius:        Radius of tool
%   === FLAGS ==============              
%   'end_cap':          Creates Connector with only bottom hinge;
%   'single':           Create Connector with only one Bowdencable per section; 
%   'crimp':            Creates slots for crimp connections; 
%   'x'/'y':            Orientation of slots for non crimp connection;   
%	=== OUTPUT RESULTS =====
%	SG:         SG of connector element
function [SG] = SGconnector(CPL,CPL_holes,positions,angle_p,length_p,h_r,tool_radius,varargin)
flags = {}; if nargin>=8 && ~isempty(varargin{1}); flags = varargin{1}; end
cut_orientation = 'y'; single = 0; end_cap = 0; crimp = 1; torsion = 1;

for f=1:size(flags,2)
   switch flags{f}
       case 'end_cap'
           end_cap = 1;
           length_p(2,3) = 0;
       case 'single'
           single = 1;
           crimp = 0;
       case 'x'
           cut_orientation = 'x';
       case 'y'
           cut_orientation = 'y';
       case 'crimp'
           crimp = 1;
   end
end

angles = {};
CPL_b = CPL{1};
if size(CPL,1) == 2
    CPL_f = CPL{2};
else
    CPL_f = CPL{1};
end
CPL_holes_b = CPL_holes{1};
if size(CPL_holes,2) == 2 
    CPL_holes_f = CPL_holes{2}; 
else
    CPL_holes_f = [];
end

%% Shifting all holes to positive x_values
if ~single
    for k = 1:size(positions,2)
        positions{k}(positions{k}(:,1)<0,:) = -positions{k}(positions{k}(:,1)<0,:);
    end
end

CPL_b = CPLbool('-',CPL_b,CPL_holes_b);
CPL_f_baseholes = CPLbool('-',CPL_f,CPL_holes_b);
CPL_f = CPLbool('-',CPL_f,CPL_holes_f);
PL_wireescape = CPLconvexhull([PLcircle(h_r*1.25);NaN NaN;PLtrans(PLsquare(h_r*2),[0 -10*h_r])]);
PL_crimp_hole = CPLconvexhull([PLcircle(h_r*1.5);NaN NaN;PLtrans(PLsquare(h_r*3),[0 -10*h_r])]);
[sizey,sizex,~,~,~,~] = sizeVL(CPL_b);
SG_bottom= SGofCPLzdelaunayGrid(CPL_b, 2,0.5,0.5);
CPL_b_wireescape = CPL_b;
CPL_f_wireescape = CPL_f;
for k=1:size(positions{1},1)
    angles{end+1} = atan2(positions{1}(k,1),positions{1}(k,2));  %double angle  pi-angle2 -angle2 Angle between centerpoint and holeposition
    CPL_b_wireescape = CPLbool('-',CPL_b_wireescape,PLtrans(PLtransR(PL_wireescape,rot(pi-angles{k})),positions{1}(k,:)));
    CPL_f_wireescape = CPLbool('-',CPL_f_wireescape,PLtrans(PLtransR(PL_wireescape,rot(pi-angles{k})),positions{1}(k,:)));
end


if crimp   
    if size(positions,2) == 2
        for k=1:size(positions{2},1)
            angle2 = atan2(positions{2}(k,1),positions{2}(k,2));
            CPL_b_wireescape = CPLbool('-',CPL_b_wireescape,PLtrans(PLtransR(PL_crimp_hole,rot(pi-angle2)),positions{2}(k,:)));
            CPL_b_wireescape = CPLbool('-',CPL_b_wireescape,PLtrans(PLtransR(PL_crimp_hole,rot(-angle2)),-positions{2}(k,:)));
            CPL_f_wireescape = CPLbool('-',CPL_f_wireescape,PLtrans(PLtransR(PL_crimp_hole,rot(pi-angle2)),positions{2}(k,:)));
            CPL_f_wireescape = CPLbool('-',CPL_f_wireescape,PLtrans(PLtransR(PL_crimp_hole,rot(-angle2)),-positions{2}(k,:)));
        end
    end
    
    if ~single
        for k=1:size(positions{1},1)
            CPL_b_wireescape = CPLbool('-',CPL_b_wireescape,PLtrans(PLtransR(PL_wireescape,rot(-angles{k})),-positions{1}(k,:)));
            CPL_f_wireescape = CPLbool('-',CPL_f_wireescape,PLtrans(PLtransR(PL_wireescape,rot(-angles{k})),-positions{1}(k,:)));
        end
    end
  
    if end_cap 
        CPL_f_wireescape= [PLcircle(tool_radius,50);NaN NaN;PLcircle(tool_radius+1,50)];
        CPL_f = CPL_f_wireescape;
    end
    SG_middle = SGof2CPLsz(CPL_b_wireescape,CPL_f_wireescape,10.5,'','rot');
    if end_cap 
           SG_top= SGofCPLzdelaunayGrid(CPL_f, 1,0.5,0.5);
    else
           SG_top= SGofCPLzdelaunayGrid(CPL_f, 2,0.5,0.5);
    end
    SG = SGstack('z',SG_bottom,SG_middle,SG_top);
    
else      
    PL_tool_guard = [PLcircle(tool_radius);NaN NaN;PLcircle(tool_radius+0.5)]; 
    if cut_orientation == 'x'
        width = (sizey/2)-abs(positions(1,1))+(h_r)+5;
        if positions{1}(1)>0
            offset = positions{1}(1)+width/2-h_r;
        else
            offset = positions{1}(1)-width/2+h_r;
        end
        CPL_b_wirechannels = CPLbool('-',CPL_b_wireescape,PLtrans(PLsquare(width,sizex*2),[offset 0]));
        if angle_p(1,2) == 2
            CPL_b_wirechannels = CPLbool('-',CPL_b_wirechannels,PLtrans(PLsquare(width,sizex*2),[-offset 0]));
        end
    else
        width = (sizex/2)-abs(positions{1}(2))+(h_r)+5;
        if positions{1}(2)>0
            offset = positions{1}(2)+width/2-h_r;
        else
            offset = positions{1}(2)-width/2+h_r;
        end
        CPL_b_wirechannels = CPLbool('-',CPL_b_wireescape,PLtrans(PLsquare(sizey*2,width),[0 offset]));
         if angle_p(1,2) == 2
            CPL_b_wirechannels = CPLbool('-',CPL_b_wirechannels,PLtrans(PLsquare(sizey*2,width),[0 -offset]));
        end
    end
    
    if positions{1}(1)>0
        CPL_b_wirechannels = CPLbool('-',CPL_b_wirechannels,PLtrans(PLsquare(sizey*2,0.8),[-sizey -positions{1}(2)]));
    else
        CPL_b_wirechannels = CPLbool('-',CPL_b_wirechannels,PLtrans(PLsquare(sizey*2,0.8),[sizey -positions{1}(2)]));
    end    
    CPL_b_wirechannels = CPLbool('+',CPL_b_wirechannels,PL_tool_guard);
    SG_wire_layer = SGofCPLz(CPL_b_wirechannels,0.6);
    SG_top_connector = SGof2CPLsz(CPL_b_wireescape,CPL_f_wireescape,2,'','miny');
    SG_top_layer = SGofCPLz(CPLbool('-',CPL_f,CPL_holes_f),1);
    SG = SGstack('z',SG_bottom,SG_wire_layer,SG_top_connector,SG_top_layer);    
end
height_SG = max(SG.VL(:,3))-min(SG.VL(:,3));
SG = SGtrans(SG,[0 0 (height_SG/2)-max(SG.VL(:,3))]);

%% add hinge

SG_hinge = SGhingeround(length_p(1,2),length_p(1,1),length_p(1,3));

SG_hinge_b = SGtransR(SG_hinge,rotz(angle_p(1,1)));
[SG_hinge_b,offset_b] = SGcreateHinge(CPL_b,SG_hinge_b,angle_p(1,1),angle_p(1,2),length_p(1,1),length_p(1,2),length_p(1,3));
SG_hinge_b = SGmirror(SG_hinge_b,'xy');

if length_p(1,2) ~= 0
    cp = PLcrossCPLLine2(PLtransC([-500 0;500 0],[0 0],deg2rad(angle_p(1,1))),CPL{1});
    dis_in_lef = pdist2(cp(3,:),[0 0]);
    dis_in_rig = pdist2(cp(4,:),[0 0]);
    dis_out_lef = pdist2(cp(1,:),[0 0]);
    dis_out_rig = pdist2(cp(2,:),[0 0]);
    SG_hinge_b =  SGtransR(SG_hinge_b,rotz(-angle_p(1,1)));    
    [distance_whole,~,~,~,~,~] = sizeVL(SG_hinge_b);
    if torsion
%         distance_whole = dis_out_lef+dis_out_rig;
        SG_hinge_l = SGcutend(SG_hinge_b,'right',distance_whole-length_p(1,2));
        SG_hinge_r =  SGcutend(SG_hinge_b,'left',distance_whole-length_p(1,2));
        SG_hinge_b = SGcat(SG_hinge_l,SG_hinge_r);
    else
        left_cut_distance = (dis_out_lef-dis_in_lef);
        right_cut_distance = (dis_out_rig-dis_in_rig);
        SG_hinge_b = SGcutend(SG_hinge_b,'left',left_cut_distance-length_p(1,2));
        SG_hinge_b = SGcutend(SG_hinge_b,'right',right_cut_distance-length_p(1,2));
    end
    SG_hinge_b =  SGtransR(SG_hinge_b,rotz(angle_p(1,1))); 
end


offset_t =0;
if ~end_cap
    SG_hinge_t = SGhingeround(length_p(1,2),length_p(2,1),length_p(2,3));
    SG_hinge_t = SGtransR(SG_hinge_t,rotz(angle_p(2,1)));
    [SG_hinge_t,offset_t] = SGcreateHinge(CPL_f,SG_hinge_t,angle_p(2,1),angle_p(2,2),length_p(2,1),length_p(1,2),length_p(2,3));
    
    if length_p(2,2) ~= 0
        cp = PLcrossCPLLine2(PLtransC([-500 0;500 0],[0 0],deg2rad(angle_p(2,1))),CPL{1});
        dis_in_lef = pdist2(cp(3,:),[0 0]);
        dis_in_rig = pdist2(cp(4,:),[0 0]);
        dis_out_lef = pdist2(cp(1,:),[0 0]);
        dis_out_rig = pdist2(cp(2,:),[0 0]);
        SG_hinge_t =  SGtransR(SG_hinge_t,rotz(-angle_p(2,1)));         
        [distance_whole,~,~,~,~,~] = sizeVL(SG_hinge_t);
        if torsion
%             distance_whole = dis_out_lef+dis_out_rig;
            SG_hinge_l = SGcutend(SG_hinge_t,'right',distance_whole-length_p(2,2));
            SG_hinge_r =  SGcutend(SG_hinge_t,'left',distance_whole-length_p(2,2));
            SG_hinge_t = SGcat(SG_hinge_l,SG_hinge_r);
        else
            left_cut_distance = (dis_out_lef-dis_in_lef);
            right_cut_distance = (dis_out_rig-dis_in_rig);
            SG_hinge_t = SGcutend(SG_hinge_t,'left',left_cut_distance-length_p(2,2));
            SG_hinge_t = SGcutend(SG_hinge_t,'right',right_cut_distance-length_p(2,2));
        end
        SG_hinge_t =  SGtransR(SG_hinge_t,rotz(angle_p(2,1)));
    end
    
    SG_hinge_b = SGunder(SG_hinge_b,SG);
    SG_hinge_t = SGontop(SG_hinge_t,SG);
    SG = SGcat(SG_hinge_b,SG_hinge_t,SG);
else
    SG = SGcat(SGunder(SG_hinge_b,SG),SG);
end

%% Setting Frames
e_dir_f = [sind(angle_p(2,1)) -cosd(angle_p(2,1))];
e_dir_b = [sind(angle_p(1,1)) -cosd(angle_p(1,1))];

H_f = [rotx(90)*roty(90+angle_p(2,1)) [offset_t*-e_dir_f';((height_SG/2)+length_p(2,3))]; 0 0 0 1];
H_b = [rotx(90)*roty(-90+angle_p(1,1)) [offset_b*-e_dir_b';(-(height_SG/2)-length_p(1,3))]; 0 0 0 1];

SG = SGTset(SG,'B',H_b);
SG = SGTset(SG,'F',H_f);
SG.offset = [offset_b,offset_t];
end

%%   [SG] = SGmanipulatorbase(CPL,middle_tool_r,tool_pos)
%	=== INPUT PARAMETERS ===
%	CPL:            CPL of bottomelements
%   CPL_out:        CPL of outer contour of elements
%   tool_radius:    Radius for tool
%   tool_pos:       0: middle 1: top
%	=== OUTPUT RESULTS ======
%	SG:             SG of Manipulatorbase
function [SG] = SGmanipulatorbase(CPL,varargin)
optic_radius=3.25;      if nargin>=2 && ~isempty(varargin{1});  optic_radius=varargin{1};       end
channel_positions=[];     if nargin>=3 && ~isempty(varargin{2});  channel_positions=varargin{2};    end
sensor_channel=0;       if nargin>=4 && ~isempty(varargin{3});  sensor_channel=varargin{3};     end
optic_channel = 0;      if nargin>=5 && ~isempty(varargin{4});  optic_channel=varargin{4};      end
single = 0;             if nargin>=6 && ~isempty(varargin{5});  single=varargin{5};             end
length = 50;            if nargin>=7 && ~isempty(varargin{6});  length=varargin{6};             end
seal = 0;               if nargin>=8 && ~isempty(varargin{7});  seal=varargin{7};               end
radial = 0;             if nargin>=9 && ~isempty(varargin{8});  radial=varargin{8};             end
flex = 0;             if nargin>=10 && ~isempty(varargin{9});  flex=varargin{9};             end
flexangle = 0;             if nargin>=11 && ~isempty(varargin{10});  flexangle=varargin{10};             end
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
    mid_points = [-x1/2-offset(1)-0.3 offset(2)];
    CPL{1} = PLtrans(CPL{1},mid_points);
    if optic_channel == 2
        CPL{1} = PLtrans(CPL{1},[0 y1/2]);
        mid_points = mid_points + [0 y1/2];
    end
    if size(CPL,2)>1
        [x2,y2,~,~,~,~] = sizeVL(CPL{2});
        mid_points = [mid_points;x2/2+offset(1)+0.3 offset(2)];
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
        CPL{3} = PLtransR(CPL{3},rot(0));
        [~,y3,~,~,~,~] = sizeVL(CPL{3});
        mid_points = [mid_points;0 max(y1/2,y2/2)+y3/2+0.3];
        %         mid_points = [mid_points;0 max(y1/2,y2/2)];
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
    PL_crimp_holes = PLtrans(PLcircle(0.7),-channel_positions{i}{1});
      
    if ~single
        PL_crimp_holes = [PL_crimp_holes;NaN NaN;PLtransR(PL_crimp_holes,rot(pi))];
    end
    if radial
        PL_crimp_holes = PLtrans(PL_crimp_holes,mid_points(1,:));
        PL_crimp_holes = PLtransR( PL_crimp_holes,rot((i-1)*(2*pi)/size(edges,1)));
    else
        if i==1
            PL_crimp_holes = PLtransC( PL_crimp_holes,[0 0],rot(pi));
        elseif i==2
            PL_crimp_holes = PLtransC( PL_crimp_holes,[0 0],rot(2*pi));
        elseif i==3
            PL_crimp_holes = PLtransC( PL_crimp_holes,[0 0],rot(2*pi));
        elseif i ==4
            PL_crimp_holes = PLtransC( PL_crimp_holes,[0 0],rot(-pi/2));
        end
        PL_crimp_holes = PLtrans(PL_crimp_holes,mid_points(i,:));
    end
    
    CPL = CPLbool('-',CPL,PL_crimp_holes);
end


PL_wire_escape = CPLconvexhull([PLcircle(0.6);NaN NaN;PLtrans(PLsquare(1.2),[0 -10])]);
PL_crimp_escape = CPLconvexhull([PLcircle(1.5);NaN NaN;PLtrans(PLsquare(3),[0 -10])]);
PL_hole = PLcircle(0.7);
CPLs_crimp_connector = CPL;
CPLs_wiree_connector = CPL;


SG_crimp_conn_top = SGofCPLz(CPL,2);
for k =1:num_arms
    temp_pos = cat(1,channel_positions{k}{:});
    if radial
        PL_temp_cut = [];
        PL_temp_cut_wire = [];
        PL_temp_cut_hole = [];
        if single 
            end_to = size(temp_pos,1);
        else
            end_to = 1;
        end          
        for i=1:end_to
            curr_temp_pos = mid_points(1,:)-temp_pos(i,:);              
            alpha = atan2(curr_temp_pos(1),curr_temp_pos(2));            
            PL_temp_c = PLtransR(PL_crimp_escape,rot(pi-alpha));
            PL_temp_c = PLtrans(PL_temp_c,curr_temp_pos);
            PL_temp_c = PLtransR( PL_temp_c,rot((k-1)*(2*pi)/size(edges,1)));
            
            
            PL_temp_cut_w = PLtransR(PL_wire_escape,rot(pi-alpha));
            PL_temp_cut_w = PLtrans(PL_temp_cut_w,curr_temp_pos);
            PL_temp_cut_w = PLtransR( PL_temp_cut_w,rot((k-1)*(2*pi)/size(edges,1)));
            
            PL_temp_cut_h = PLtransR(PL_hole,rot(pi-alpha));
            PL_temp_cut_h = PLtrans(PL_temp_cut_h,curr_temp_pos);
            PL_temp_cut_h = PLtransR( PL_temp_cut_h,rot((k-1)*(2*pi)/size(edges,1)));
            
             PL_temp_cut = [PL_temp_cut;NaN NaN;PL_temp_c];            
            PL_temp_cut_wire = [PL_temp_cut_wire;NaN NaN;PL_temp_cut_w];            
            PL_temp_cut_hole = [PL_temp_cut_hole;NaN NaN;PL_temp_cut_h];
            
            if single == 0
                curr_temp_pos = mid_points(1,:)+temp_pos(i,:);
                alpha = atan2(curr_temp_pos(1),curr_temp_pos(2));
                PL_temp_c = PLtransR(PL_crimp_escape,rot(pi-alpha));
                PL_temp_c = PLtrans(PL_temp_c,curr_temp_pos);
                PL_temp_c = PLtransR( PL_temp_c,rot((k-1)*(2*pi)/size(edges,1)));                
                
                PL_temp_cut_w = PLtransR(PL_wire_escape,rot(pi-alpha));
                PL_temp_cut_w = PLtrans(PL_temp_cut_w,curr_temp_pos);
                PL_temp_cut_w = PLtransR( PL_temp_cut_w,rot((k-1)*(2*pi)/size(edges,1)));
                
                PL_temp_cut_h = PLtransR(PL_hole,rot(pi-alpha));
                PL_temp_cut_h = PLtrans(PL_temp_cut_h,curr_temp_pos);
                PL_temp_cut_h = PLtransR( PL_temp_cut_h,rot((k-1)*(2*pi)/size(edges,1)));
            end
            
            PL_temp_cut = [PL_temp_cut;NaN NaN;PL_temp_c];            
            PL_temp_cut_wire = [PL_temp_cut_wire;NaN NaN;PL_temp_cut_w];            
            PL_temp_cut_hole = [PL_temp_cut_hole;NaN NaN;PL_temp_cut_h];
        end
    else        
        for i=1:size(temp_pos,1)
            if ~single && i > 1 continue; end
            if k==1
                curr_temp_pos = mid_points(k,:)-temp_pos(i,:);
                alpha = atan2(curr_temp_pos(1),curr_temp_pos(2));
                PL_temp_cut = PLtransR(PL_crimp_escape,rot(pi-alpha));
                PL_temp_cut = PLtrans(PL_temp_cut,curr_temp_pos);
                PL_temp_cut_wire = PLtransR(PL_wire_escape,rot(pi-alpha));
                PL_temp_cut_wire = PLtrans(PL_temp_cut_wire,curr_temp_pos);
                PL_temp_cut_hole = PLtrans(PL_hole,curr_temp_pos);
                if single == 0
                    curr_temp_pos_2 =  mid_points(k,:)+temp_pos(i,:);
                    alpha = atan2(curr_temp_pos_2(1),curr_temp_pos_2(2));
                    PL_temp_cut_2 = PLtransR(PL_crimp_escape,rot(pi-alpha));
                    PL_temp_cut_2 = PLtrans(PL_temp_cut_2,curr_temp_pos_2);
                    
                    PL_temp_cut_wire_2 = PLtransR(PL_wire_escape,rot(pi-alpha));
                    PL_temp_cut_wire_2 = PLtrans(PL_temp_cut_wire_2,curr_temp_pos_2);
                    PL_temp_cut_hole_2 = PLtrans(PL_hole,curr_temp_pos_2);
                    
                    PL_temp_cut = [PL_temp_cut;NaN NaN;PL_temp_cut_2];
                    PL_temp_cut_wire = [PL_temp_cut_wire;NaN NaN;PL_temp_cut_wire_2];
                    PL_temp_cut_hole = [PL_temp_cut_hole;NaN NaN;PL_temp_cut_hole_2];
                end
            elseif k==2
                curr_temp_pos = mid_points(k,:)+temp_pos(i,:);
                alpha = atan2(curr_temp_pos(1),curr_temp_pos(2));
                PL_temp_cut = PLtransR(PL_crimp_escape,rot(pi-alpha));
                PL_temp_cut = PLtrans(PL_temp_cut,curr_temp_pos);
                PL_temp_cut_wire = PLtransR(PL_wire_escape,rot(pi-alpha));
                PL_temp_cut_wire = PLtrans(PL_temp_cut_wire,curr_temp_pos);
                PL_temp_cut_hole = PLtrans(PL_hole,curr_temp_pos);
                if single == 0
                    curr_temp_pos_2 =  mid_points(k,:)-temp_pos(i,:);
                    alpha = atan2(curr_temp_pos_2(1),curr_temp_pos_2(2));
                    PL_temp_cut_2 = PLtransR(PL_crimp_escape,rot(pi-alpha));
                    PL_temp_cut_2 = PLtrans(PL_temp_cut_2,curr_temp_pos_2);
                    
                    PL_temp_cut_wire_2 = PLtransR(PL_wire_escape,rot(pi-alpha));
                    PL_temp_cut_wire_2 = PLtrans(PL_temp_cut_wire_2,curr_temp_pos_2);
                    PL_temp_cut_hole_2 = PLtrans(PL_hole,curr_temp_pos_2);
                    
                    PL_temp_cut = [PL_temp_cut;NaN NaN;PL_temp_cut_2];
                    PL_temp_cut_wire = [PL_temp_cut_wire;NaN NaN;PL_temp_cut_wire_2];
                    PL_temp_cut_hole = [PL_temp_cut_hole;NaN NaN;PL_temp_cut_hole_2];
                end
            elseif k==3
                curr_temp_pos = mid_points(k,:)-temp_pos(i,:)*rot(0);
                alpha = atan2(curr_temp_pos(1),curr_temp_pos(2));
                PL_temp_cut = PLtransR(PL_crimp_escape,rot(pi-alpha));
                PL_temp_cut = PLtrans(PL_temp_cut,curr_temp_pos);
                PL_temp_cut_wire = PLtransR(PL_wire_escape,rot(pi-alpha));
                PL_temp_cut_wire = PLtrans(PL_temp_cut_wire,curr_temp_pos);
                PL_temp_cut_hole = PLtrans(PL_hole,curr_temp_pos);
                if single == 0
                    curr_temp_pos_2 =  mid_points(k,:)+temp_pos(i,:)*rot(0);
                    alpha = atan2(curr_temp_pos_2(1),curr_temp_pos_2(2));
                    PL_temp_cut_2 = PLtransR(PL_crimp_escape,rot(pi-alpha));
                    PL_temp_cut_2 = PLtrans(PL_temp_cut_2,curr_temp_pos_2);
                    
                    PL_temp_cut_wire_2 = PLtransR(PL_wire_escape,rot(pi-alpha));
                    PL_temp_cut_wire_2 = PLtrans(PL_temp_cut_wire_2,curr_temp_pos_2);
                    PL_temp_cut_hole_2 = PLtrans(PL_hole,curr_temp_pos_2);
                    
                    PL_temp_cut = [PL_temp_cut;NaN NaN;PL_temp_cut_2];
                    PL_temp_cut_wire = [PL_temp_cut_wire;NaN NaN;PL_temp_cut_wire_2];
                    PL_temp_cut_hole = [PL_temp_cut_hole;NaN NaN;PL_temp_cut_hole_2];
                end
            elseif k ==4
                curr_temp_pos = mid_points(k,:)+temp_pos(i,:)*rot(pi/2);
                alpha = atan2(curr_temp_pos(1),curr_temp_pos(2));
                PL_temp_cut = PLtransR(PL_crimp_escape,rot(pi-alpha));
                PL_temp_cut = PLtrans(PL_temp_cut,curr_temp_pos);
                PL_temp_cut_wire = PLtransR(PL_wire_escape,rot(pi-alpha));
                PL_temp_cut_wire = PLtrans(PL_temp_cut_wire,curr_temp_pos);
                PL_temp_cut_hole = PLtrans(PL_hole,curr_temp_pos);
                if single == 0
                    curr_temp_pos_2 =  mid_points(k,:)-temp_pos(i,:)*rot(pi/2);
                    alpha = atan2(curr_temp_pos_2(1),curr_temp_pos_2(2));
                    PL_temp_cut_2 = PLtransR(PL_crimp_escape,rot(pi-alpha));
                    PL_temp_cut_2 = PLtrans(PL_temp_cut_2,curr_temp_pos_2);
                    
                    PL_temp_cut_wire_2 = PLtransR(PL_wire_escape,rot(pi-alpha));
                    PL_temp_cut_wire_2 = PLtrans(PL_temp_cut_wire_2,curr_temp_pos_2);
                    PL_temp_cut_hole_2 = PLtrans(PL_hole,curr_temp_pos_2);
                    
                    PL_temp_cut = [PL_temp_cut;NaN NaN;PL_temp_cut_2];
                    PL_temp_cut_wire = [PL_temp_cut_wire;NaN NaN;PL_temp_cut_wire_2];
                    PL_temp_cut_hole = [PL_temp_cut_hole;NaN NaN;PL_temp_cut_hole_2];
                end
            end
        end        
    end
    
        CPL= CPLbool('-',CPL,PL_temp_cut_hole);
        CPLs_crimp_connector = CPLbool('-',CPLs_crimp_connector,PL_temp_cut);
        CPLs_wiree_connector = CPLbool('-',CPLs_wiree_connector,PL_temp_cut_wire);
end

SG_wireescape = SGofCPLz(CPLs_wiree_connector,5);
SG_crimpescape = SGofCPLz(CPLs_crimp_connector,10.5);
SG_crimp_connector = SGstack('z',SG_wireescape,SG_crimpescape,SG_crimp_conn_top);


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
if flex    
    SG = SGstackn(SG,1,0);
else    
    SG = SGstackn(SG,length,0);
end
SG = SGstack('z',SG,SG_crimp_connector);

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

if flex
    SG_flex = SGfitToPrinterWorkspace(CPL,length,flexangle);
    SG_flex = SGcat(SG_flex);
end


%% Add Frames
height_SG = abs(max(SG.VL(:,3))-min(SG.VL(:,3)));
H_b_b = [rotx(-90) [0;0;-2]; 0 0 0 1];
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
if flex    
    SG = SGTchain({SG_flex,SG});
    x = SG{1};
    SG{1} = SG{2};
    SG{2} = x;
    SG = SGcat(SG);
end
end

%%   [SG]=SGhingeround(length, width, h_height)
%	=== INPUT PARAMETERS ===
%	length:		Hinge length
%	width:		Middle width of hinge
%	h_height:	Height of Hinge
%	=== OUTPUT RESULTS ======
%	SG:         SG of hinge with round contour
function [SG] = SGhingeround(length, width, h_height)
PL_hinge_top = PLcircseg(width/2,floor(width*10),pi,0);
PL_hinge_base = PLtrans(PLcircseg(h_height,floor(h_height*20),0,-pi/2),[-h_height-width/2 0]);
CPL = [PL_hinge_top;VLswapX(PL_hinge_base);flip(PL_hinge_base)];
% CPL = CPLaddauxpoints(CPL,0.125);

SG = SGofCPLzdelaunayGrid(uniquetol(CPL,'ByRows',0.01),length,0.125,0.125);
SG = SGtrans(SG,[0;h_height;-length/2]);
SG = SGtransR(SG,rot(pi/2,pi/2,0));
end

%%  [SG] = SGcreateHinge(CPL,SG_hinge,hinge_dir)
%	=== INPUT PARAMETERS ===
%	CPL:        CPL of element you want to create a hinge for
%	SG_hinge:   SG of your hinge
%	hinge_dir:  Direction of hinge
%   hinge_opti: 0 no opti; 1 opti in pos -1 in negative
%	=== OUTPUT RESULTS ======
%	SG:         SG of connector element
%   offset:     Calculated offset in positive y direction
function [SG,offset] = SGcreateHinge(CPL,SG_hinge,hinge_dir,hinge_opti,hinge_width,min_len,h_height)
%% Check if same hinge was already created once

global hinges;
already_existing = 0;
for i=1:size(hinges,2)
    if isequalwithequalnans(hinges{i}{3},CPL) && isequal(hinges{i}{4},[hinge_dir,hinge_opti,hinge_width,min_len])
        SG =  hinges{i}{1};
        offset = hinges{i}{2};
        already_existing = 1;
        break;
    end
end

if ~already_existing
    
    %% Initializing and generating general values
    offset = 0;
    max_dim = max(sizeVL(CPL))+1;
    middle_axis = PLtransR([-max_dim 0;max_dim 0],rot(deg2rad(hinge_dir)));
    e_dir = [sind(hinge_dir) -cosd(hinge_dir)];
    PL_offsetline = middle_axis;
    middle_axis = PLtransR(middle_axis,rot(pi/2));
    pos_plane = [flip(middle_axis);PLtrans(middle_axis,e_dir*rot(-pi/2)*max_dim*2)]; % Plane for finding points in positive area
    hinge_width = hinge_width+(2*h_height);
%     hinge_width = hinge_width+1;
    
    %% Calculating best offset
    if abs(hinge_opti) == 1
        PL_offsetline = PLtrans(PL_offsetline,-e_dir*max_dim*hinge_opti);
        size_h = 0; res = 0.3; offset = max_dim;
        while size_h < min_len && offset > 0
            size_h = 0;
            PL_offsetline = PLtrans(PL_offsetline,e_dir*res*hinge_opti);
            offset = offset-res;
            c_p = PLcrossCPLLine3(PL_offsetline,CPL);
            if ~isempty(c_p)
                c_p = sortrows(c_p);
                ex_1 = c_p(1,:);
                ex_2 = c_p(end,:);
                ex_1_n = floor(distPointLine(middle_axis,ex_1)/res)-1;
                ex_2_n = floor(distPointLine(middle_axis,ex_2)/res)-1;
                PL_ex_1 = [ex_1;PLtrans(ex_1,e_dir*rot(pi/2)*hinge_width)];
                PL_ex_2 = [ex_2;PLtrans(ex_2,-e_dir*rot(pi/2)*hinge_width)];
                counting = 0;
                for k=0:ex_1_n
                    inside_ex_1 = inside2C(CPL,PL_ex_1);
                    if (inside_ex_1(1)+inside_ex_1(3)) == size(PL_ex_1,1) && inside_ex_1(4) == 0  && counting
                        size_h = size_h + res;
                    elseif (inside_ex_1(1)+inside_ex_1(3)) == size(PL_ex_1,1) && inside_ex_1(4) == 0  && ~counting
                        counting = 1;
                    else
                        counting = 0;
                    end
                    PL_ex_1 = PLtrans(PL_ex_1,e_dir*res);
                end
                counting = 0;
                for k=0:ex_2_n
                    inside_ex_2 = inside2C(CPL,PL_ex_2);
                    if (inside_ex_2(1)+inside_ex_2(3)) == size(PL_ex_2,1) && inside_ex_2(4) == 0  && counting
                        size_h = size_h + res;
                    elseif (inside_ex_2(1)+inside_ex_2(3)) == size(PL_ex_2,1) && inside_ex_2(4) == 0  && ~counting
                        counting = 1;
                    else
                        counting = 0;
                    end
                    PL_ex_2 = PLtrans(PL_ex_2,-e_dir*res);
                end
            end
        end
        if offset>0
            PL_offsetline_mid_hinge = PLtrans(PL_offsetline,e_dir*rot(pi/2)*((hinge_width/2)));
            offset = (offset-(hinge_width/2));
            SG_hinge = SGtrans(SG_hinge,[-e_dir*offset*hinge_opti 0]);
            leftOf = leftOfLine(PL_offsetline_mid_hinge,[0 0]);
        else
            error("Not enough space for hinge");
        end
    end
    SG_hinge = SGtrans(SG_hinge,[e_dir*rot(-pi/2)*max_dim 0]);
    VL_hinge = SG_hinge.VL;
    if distLinePoint(middle_axis,VL_hinge(1,1:2))>distLinePoint(middle_axis,VL_hinge(end,1:2))
        error("Hinge the wrong way around would create inverted Normals");
    end
    
    SG =[];
    proj_points = {};
    %%  Generating all cross points between CPL and hingepoint projections
    
    unique_points = unique(VL_hinge(:,1:2),'rows');
    unique_cp = {};
    for o=1:size(unique_points,1)
        PL_cross_line = [unique_points(o,1:2);unique_points(o,1:2)+(e_dir*rot(pi/2)*max_dim*2)];
        c_p =  PLcrossCPLLine3(PL_cross_line,CPL);
        c_p(:,3) = pdist2(c_p,unique_points(o,1:2));
        c_p = sortrows(c_p,3);
        c_p = c_p(:,1:2);
        unique_cp{end+1} = c_p;
    end
    for o=1:size(VL_hinge,1)
        [~,index] = ismember(VL_hinge(o,1:2),unique_points,'rows');
        proj_points{end+1} = unique_cp{index};
    end
    
    %% Getting number of hinge elements below and above axis through [0 0]
    [proj_points_size, ~] = max(cellfun('size', proj_points, 1));
    proj_points_max_values = cellfun('size', proj_points, 1) == proj_points_size;
    
    max_axis = proj_points(proj_points_max_values);
    origin_axis = PLtrans(middle_axis,e_dir(1,:)*rot(pi/2)*max_dim);
    
    plains = {[e_dir;-e_dir]};
    plain_offsets = [distLinePoint(origin_axis,[0 0]) distLinePoint(origin_axis,[0 0])];    
    for j=1:size(max_axis,2)
        for k=1:2:proj_points_size-2
            axis_offset = [mean(max_axis{j}(k+1:k+2,1)),mean(max_axis{j}(k+1:k+2,2))];
            plain_temp = [axis_offset;axis_offset+(e_dir)];
            dis_temp =  distPointLine(origin_axis,plain_temp(1,:));
            if min(abs(plain_offsets-dis_temp))>0.1
                plains{end+1} = plain_temp;
                plain_offsets = [plain_offsets dis_temp dis_temp];
            end
        end
    end
    if size(plains,2) >1
        plains = sortrows([cell2mat(plains') plain_offsets'],3,'descend');
        plains = mat2cell(plains(:,1:2),repmat(2,1,size(plain_offsets,2)/2));
    end
    positive_gl = 1;
    negative_gl = 1;
    len = pdist2(PL_offsetline(1,:),PL_offsetline(2,:))/2;
    for o=1:size(plains,1)
        dist = distLinePoint(plains{o},PL_offsetline(2,:));
        if round(dist,1) < round(len,1)
            positive_gl = positive_gl+1;
        elseif round(dist,1) > round(len,1)
            negative_gl = negative_gl+1;
        end
    end
    positive_gl = positive_gl*2;
    negative_gl = negative_gl*2;
    proj_points_size = (size(plains,1)+1)*2;
    %% Filling up points list in a way that it can be iterated through easily afterwards to generate hinge elements
    for k=1:size(proj_points,2)
        if size(proj_points{k},1) < proj_points_size
            if size(proj_points{k},1) == 2
                proj_points{k}=[proj_points{k};repmat(proj_points{k}(end-1:end,:),(proj_points_size-size(proj_points{k},1))/2,1)];
            else
                num_pos_neg = proj_points{k};
                in_out = leftOfLine(middle_axis,num_pos_neg);
                positive = length(find(in_out>0));
                negative = length(find(in_out<0));
                positive_v = proj_points{k}(1:positive,:);
                negative_v = proj_points{k}(positive+1:end,:);
                if mod(positive,2) ~= 0
                    negative_temp = negative_v(1,:);
                    positive_temp = positive_v(end,:);
                    positive_v = [positive_v;negative_temp];
                    negative_v = [positive_temp;negative_v];
                    negative = negative+1;
                    positive = positive+1;
                end
                positive_v = [positive_v;repmat(positive_v(end-1:end,:),floor((positive_gl-positive)/2),1)];
                negative_v = [negative_v;repmat(negative_v(end-1:end,:),floor((negative_gl-negative)/2),1)];
                proj_points{k} = [positive_v;negative_v];
            end
        end
    end
    %% Replacing values to minimize overlapping triangles
    for u=1:size(proj_points,2)
        for j=1:size(plains,1)
            or_point = proj_points{u}(j*2-1,:);
            curr_point = proj_points{u}(j*2,:);
            distance_point_plain = distLinePoint(plains{j},or_point);
            if pdist2(or_point,curr_point)>distance_point_plain
                new_point = or_point+distance_point_plain*e_dir*rot(pi/2);
                proj_points{u}(j*2,:) = new_point;       
            end
            or_point_2 = proj_points{u}(j*2+2,:);
            curr_point_2 = proj_points{u}(j*2+1,:);
            distance_point_plain_2 = distPointLine(plains{j},or_point_2);
            if pdist2(or_point_2,curr_point_2)>distance_point_plain_2
                new_point = or_point+distance_point_plain*e_dir*rot(pi/2);
                proj_points{u}(j*2+1,:) = new_point;
            end
        end
    end
    %% Replacing values in hinge VLs with projected values
    for m=1:2:proj_points_size
        SG_hinge_new = SG_hinge;
        for n=1:size(SG_hinge.VL,1)/2
            SG_hinge_new.VL(n,1:2) = proj_points{n}(m+1,1:2);
        end
        for n=size(SG_hinge.VL,1)/2+1:size(SG_hinge.VL,1)
            SG_hinge_new.VL(n,1:2) = proj_points{n}(m,1:2);
        end
        SG = SGcat(SG,SG_hinge_new);
    end
    offset = offset*hinge_opti;
    hinges{end+1} = {SG;offset;CPL;[hinge_dir,hinge_opti,hinge_width-1,min_len]};
    
end
end

%%  [CP] = PLcrossCPLLine2(CPL_line,CPL)
%	=== INPUT PARAMETERS ===
%	CPL_line:   CPL of line to cross CPL
%	CPL:        CPL of CPL to cross CPL_line
%	M_paras:   	3xn Vector of DoFs [direction_angle total_angle offset]
%	=== OUTPUT RESULTS ======
%	CP:         List of all crosspoints
function [CP] = PLcrossCPLLine3(CPL_line,CPL)
pgon = polyshape(CPL(:,1),CPL(:,2));
lineseg = [CPL_line(1,1),CPL_line(1,2);CPL_line(2,1),CPL_line(2,2)];
[CP,~] = intersect(pgon,lineseg);
ind = ~isnan(CP(:,1));
CP = CP(ind,:);
end

%%   [SG] = SGelementstops(CPL)
%	=== INPUT PARAMETERS ===
%   CPL:  CPL of element
%	=== OUTPUT RESULTS ======
%	SG: 	SGstops
function [SG,left_height,right_height] =  SGstops(SGs,CPL_out,h_dir,offset,left_angle,right_angle,length_p,varargin)
dual_axis = 0; if nargin>=8 && ~isempty(varargin{1}); dual_axis=varargin{1}; end
if size(SGs,2) == 2 
    is_connector = 1; 
    SG = SGs{1};
    SG_2 = SGs{2};
else
    is_connector = 0;
    SG = SGs;
end
offset = -offset;

if h_dir <= 0 
%     offset = -offset;
end
hinge_width = length_p(:,1);
height = length_p(:,2);
height_SG = abs(max(SG.VL(:,3))-min(SG.VL(:,3)));

if is_connector
    height_SG = [height_SG;abs(max(SG_2.VL(:,3))-min(SG_2.VL(:,3)))];
    e_h_1 =  (height_SG(1) - sum(height(1:2)) -sum(hinge_width(1:2))/2)/2;
    e_h_2 =  (height_SG(2) - sum(height(2:3)) -sum(hinge_width(2:3))/2)/2;
    ele_height = [e_h_1,e_h_2];
    height = height(2);
    hinge_width = hinge_width(2);
else    
    ele_height = (height_SG - (2*height) -(hinge_width))/2;
end


max_dim = max(sizeVL(CPL_out))+1;
middle_axis = PLtransR(PLtrans([-max_dim 0;max_dim 0],[0 offset]),rot(deg2rad(180-h_dir)));
e_dir = (middle_axis/norm(middle_axis))*rot(pi/2);
e_dir = (e_dir(1,:)-e_dir(2,:))/norm(e_dir(1,:)-e_dir(2,:));
left_plane = [flip(middle_axis);PLtrans(middle_axis,e_dir*10*max_dim)]; % Plane for finding points in positive area
right_plane = [flip(middle_axis);PLtrans(middle_axis,e_dir*10*-max_dim)];

CPL_out_left = CPLbool('-',CPL_out,left_plane);
CPL_out_right = CPLbool('-',CPL_out,right_plane);

max_distance_left = 0;
for k=1:size(CPL_out_left,1)
    temp_dis = distPointLine(middle_axis,CPL_out_left(k,:));
    if temp_dis>max_distance_left
        max_distance_left = temp_dis;
    end
end
max_distance_right = 0;
for k=1:size(CPL_out_right,1)
     temp_dis = distPointLine(middle_axis,CPL_out_right(k,:));
    if temp_dis>max_distance_right
        max_distance_right = temp_dis;
    end
end


offset_p = floor((max_distance_right/(max_distance_right+max_distance_left))*500);

right_height = height-(tand(left_angle)*max_distance_left);
left_height = height-(tand(right_angle)*max_distance_right);

PLcontour = [(linspace(-max_distance_right,max_distance_left,500))-offset;linspace(left_height,height,offset_p) linspace(height,right_height,500-offset_p)]';

VLcontour = [PLcontour(:,1) zeros(size(PLcontour,1),1) PLcontour(:,2) ]; %create  VLcontour to plot it in vertical plane (x-z-plane)
VLcontour = VLtrans(VLcontour,TofR(rotz(90-h_dir)));
[~,max_row] = max(VLcontour);
dist_hinge = distPointLine(middle_axis,VLcontour(max_row(3),1:2));
if dist_hinge>0.1 
    VLcontour = VLtrans(VLcontour,TofR(rotz(180)));
end

if is_connector
    VLcontour_up = VLtrans(VLcontour,[0 0 ele_height(1)]);
    VLcontour_down = VLtrans(VLswapZ(VLcontour),[0 0 -ele_height(2)]);
    if dual_axis
        VLcontour_down = VLtrans(VLcontour_down,TofR(rotz(90)));
    end
    index_up = find(SG.VL(:,3) > ele_height(1)-0.1 & SG.VL(:,3) < ele_height(1)+0.1 );
    index_down = find(SG_2.VL(:,3) < -ele_height(2)+0.1 & SG_2.VL(:,3) > -ele_height(2)-0.1);
else
    VLcontour_up = VLtrans(VLcontour,[0 0 ele_height]);
    VLcontour_down = VLswapZ(VLcontour_up);
    index_down = find(SG.VL(:,3) > -ele_height-0.1 & SG.VL(:,3) < -ele_height+0.1 );
    index_up = find(SG.VL(:,3) < ele_height+0.1 & SG.VL(:,3) > ele_height-0.1);
end



for i=1:size(index_up,1)
    [~,idx] = min(pdist2(SG.VL(index_up(i),1:2),VLcontour_up(:,[1,2])));
    if distPointLine(middle_axis, SG.VL(index_up(i),1:2))>(hinge_width+1)
        SG.VL(index_up(i),3) = VLcontour_up(idx,3);
    end
end

if dual_axis
    middle_axis = PLtransR(middle_axis,rot(pi/2));
end
if is_connector    
    for i=1:size(index_down,1)
        [~,idx] = min(pdist2(SG_2.VL(index_down(i),1:2),VLcontour_down(:,[1,2])));
        if distPointLine(middle_axis,SG_2.VL(index_down(i),1:2))>(hinge_width+1)
            SG_2.VL(index_down(i),3) = VLcontour_down(idx,3);
        end
    end
    SG = {SG;SG_2};
else    
    for i=1:size(index_down,1)
        [~,idx] = min(pdist2(SG.VL(index_down(i),1:2),VLcontour_down(:,[1,2])));
        if distPointLine(middle_axis,SG.VL(index_down(i),1:2))>(hinge_width+1)
            SG.VL(index_down(i),3) = VLcontour_down(idx,3);
        end
    end
end


end

function [LFrm]=SGTframeChain2(nums,varargin)
split = ((nums-1)/2)+2; if nargin>=2 && ~isempty(varargin{1}); split=varargin{1}; end

SGt = cell(nums,5);
for i=1:nums
    SGt{i,1} = i;
    SGt{i,2} = 'F';
    SGt{i,3} = 'B';
    SGt{i,4} = i-1;
    SGt{i,5} = i;
end
SGt{1,2} = '_';
current = 2;
for i = 1: size(split,2)
    current = current +split(i);
    SGt(current,2) ={['F' num2str(i)]};
    SGt(current,4) = {[1]};
end

LFrm = SGt;
end

%%  [dist] = distLinePoint((PL_line,points)
%	=== INPUT PARAMETERS ===
%	PL_line:    PL of straight line
%	points:     nx2 points you want the distance to the line
%	=== OUTPUT RESULTS ======
%	dist:       shortest distance from point to line
function [dist] = distLinePoint(PL_line,points)
x1 =  PL_line(1,1);
x2 =  PL_line(2,1);
y1 =  PL_line(1,2);
y2 = PL_line(2,2);
x0 = points(:,1);
y0 = points(:,2);

nom = (y2-y1)*x0-(x2-x1)*y0+x2*y1-y2*x1;
denom = (y2-y1)^2+(x2-x1)^2;

dist = abs(nom)/sqrt(denom);


end

%%  [dist] = distPointLine(PL_line,point)
%	=== INPUT PARAMETERS ===
%	PL_line:    PL of straight line
%	point:      2x1 point you want the distance to the line
%	=== OUTPUT RESULTS ======
%	dist:       shortest distance from point to line
function [dist] = distPointLine(PL_line,point)

v1 = [PL_line(1,:) 0]';
v2 = [PL_line(2,:) 0]';
pt = [point 0]';
a = v1 - v2;
b = pt - v2;
dist = norm(cross(a,b)) / norm(a);

end

%%  [index] = leftOfLine(PL_line,points)
%	=== INPUT PARAMETERS ===
%	PL_line:    PL of straight line
%	points:     Point List
%	=== OUTPUT RESULTS ======
%	index:      -1 right of, 0 on 1 left
function [index] = leftOfLine(PL_line,points)

d = (points(:,1)-PL_line(1,1))*(PL_line(2,2)-PL_line(1,2))-(points(:,2)-PL_line(1,2))*(PL_line(2,1)-PL_line(1,1));
d= d./abs(d);
index = d;
end

%%  [CP] = PLcrossCPLLine2(CPL_line,CPL)
%	=== INPUT PARAMETERS ===
%	CPL_line:   CPL of line to cross CPL
%	CPL:        CPL of CPL to cross CPL_line
%	M_paras:   	3xn Vector of DoFs [direction_angle total_angle offset]
%	=== OUTPUT RESULTS ======
%	CP:         List of all crosspoints
function [CP] = PLcrossCPLLine2(CPL_line,CPL)
%%Initializing
sep = [0 find(isnan(CPL(:,1)),15)' size(CPL,1)+1];
CPLs = {};
CP = [];
%% Seperating CPL in its seperate CPLs
for i=1:size(sep,2)-1
    if size(sep,2)>2
        CPLs{end+1} = CPL(sep(i)+1:sep(i+1)-1,:);
    else
        CPLs{end+1} = CPL;
    end
end
%%Finding crosspoints
for k=1:size(CPLs,2)
    for n=1:size(CPLs{1,k},1)
        if n==size(CPLs{1,k},1) %% Wraparound for last to first point
            CP_temp = PLcrossCPLline(CPL_line,CPLs{1,k}(n,:),CPLs{1,k}(1,:));
            CP_temp_2 = PLcrossCPLline(CPL_line,CPLs{1,k}(n,:),CPLs{1,k}(1,:),true);
        else
            CP_temp = PLcrossCPLline(CPL_line,CPLs{1,k}(n,:),CPLs{1,k}(n+1,:));
            CP_temp_2 = PLcrossCPLline(CPL_line,CPLs{1,k}(n,:),CPLs{1,k}(n+1,:),true);
            
        end
               
        if isempty(CP_temp)
            if ~isempty(CP_temp_2)
                if sum(CP_temp_2 == CPL_line(1,:)) == 2 || sum(CP_temp_2 == CPL_line(2,:)) == 2
                    CP_temp_2 = [];
                end
            end
            CP_temp = CP_temp_2;
        end
        CP = [CP;CP_temp];
        
    end
end
  CP = unique(CP,'rows','stable');
end

%%   [SG] = SGmirror(SG,axis)
%	=== INPUT PARAMETERS ===
%	SG:    SG you want to mirror
%	plane: plane you want to mirror on
%	=== OUTPUT RESULTS ======
%	SG:         Mirrored SG
function [SG] = SGmirror(SG,plane)
switch plane
    case 'yz'
        SG.VL = VLswapX(SG.VL);
        SG.FL(:,[2 3]) = SG.FL(:,[3 2]);
    case 'xz'
        SG.VL = VLswapY(SG.VL);
        SG.FL(:,[2 3]) = SG.FL(:,[3 2]);
    case 'xy'
        SG.VL = VLswapZ(SG.VL);
        SG.FL(:,[1 3]) = SG.FL(:,[3 1]);
    otherwise
        error(plane + " plane doesnt exist");
end
end

%%  [SG] = SGstack(SGs)
%	=== INPUT PARAMETERS ===
%   dir:        Direction of stacking y,x,z
%	SGs:        Array of SGs

%	=== OUTPUT RESULTS ======
%	SG:         SG of stacked SGs
function [SG] = SGstack(dir,varargin)
SG = varargin{1};
for i=2:nargin-1
    switch dir
        case 'z'
            SG = SGcat(SG,SGontop(varargin{i},SG));            
        case 'y'
            SG = SGcat(SG,SGinfront(varargin{i},SG));       
        case 'x'
            SG = SGcat(SG,SGleft(varargin{i},SG));
    end
end
end

%%%TEST

function [ SGres ] = SGfitToPrinterWorkspace(CPL, length,angle )
%SGfitToPrinterWorkspace 
%
%   == INPUTS =============================================================
%   alpha       angle between element(j-1) and element(j) in degree, 
%               j = 2...n, (nx1 double)
%               - n defines number of elements
%               - first entry of alpha must be NaN
%               Example for three elements: [NaN; 5; 5]
%   == OUTPUTS ============================================================
%   SGres       solid geometry
%

%% geometries
alpha = [NaN];
for i=1:length
    alpha = [alpha;angle;0];
end

% CPL=CPLnew;
jointAngle=20;
jointThickness=0.75;
jointWidth=0.65;
jointRadius=1;
rigidThickness=1.4;

SG1 = SGovertubSection(CPL,jointAngle,jointThickness, jointWidth, jointRadius,rigidThickness);

SG2 = SGovertubSection(PLtransR(CPL,rot(pi/2)),jointAngle,jointThickness, jointWidth, jointRadius,rigidThickness);

% SG2 = SGtransT(SG2,[...
%                        cos(pi) 0 sin(pi) 0;...
%                        0 1 0 0;...
%                        -sin(pi) 0 cos(pi) 0;...
%                        0 0 0 1]);
SG2 = SGtransT(SG2,[cos(-pi/2) -sin(-pi/2) 0 0;...
                    sin(-pi/2) cos(-pi/2) 0  0;...
                    0  0  1  1e-03;...
                       0 0 0 1]);
SG2 = SGmirror(SG2,'xy');                
z1 = max(SG1.VL(:,3))-jointWidth/2;           
z2 = min(SG2.VL(:,3))+jointWidth/2;



%% get flexible geometry

% number of elements
n = max(size(alpha));

% ELEMENTS
% preallocation
SGel = cell(n,1);
% first element
% SGel{1} = SGcat(SGtrans(SG1,rot(0,0,pi/2)),SGstart);
SGel{1} = SGtrans(SGmirror(SG1,'xz'),rot(0,0,pi/2));
% i-th element
% combine upper and lower part and move lower axis to origin
SGel_1 = SGtrans(SGtrans(SGcat(SG1,SG2),rot(0,0,pi/2)),[0;0;abs(z2)]);
SGel_2 = SGtrans(SGtrans(SGcat(SG1,SG2),rot(pi,0,0)),[0;0;z1]);

% SGel_2 = SGmirror(SGel_2,'yz'); 
SGel_1 = SGmirror(SGel_1,'xz');  
for i = 2:n-1
    % save in cell
    if mod(i,2)==1
        SGel{i} = SGel_1;
    else
        SGel{i} = SGtrans(SGel_2,rot(0,0,pi));
    end
end
% last element
if mod(n,2)==1
    SGel{n} = SGtrans(SGtrans(  SGmirror(SG2,'xz'),rot(0,0,pi/2)),[0;0;abs(z2)]);
%     SGel{n} = SGcat(SGel{n},SGtrans(SGtrans(SGtip,[0,0,abs(z2)-1e-3]),rot(0,0,pi/2)));

%      SGel{n} = SGcat(SGel{n},SGtrans(SGtrans(SGbox([4,4,4]),[0,0,abs(z2)-1e-3]),rot(0,0,pi/2)));
else
    SGel{n} = SGtrans(SGtrans(SGtrans(SG1,rot(pi,0,0)),[0;0;z1]),rot(0,0,pi));
%     SGel{n} = SGcat(SGel{n},SGtrans(SGtip,[0,0,z1-1e-3]));
% SGel{n} = SGcat(SGel{n},SGtrans(SGbox([4,4,4]),[0,0,z1-1e-3]));
end


% HEIGHTS
heights = ones(n,1)*z1+abs(z2);
heights(1) = z1;

% PHI
phi = [NaN; zeros(n-1,1)];

for i = 2:n
    if mod(i,2)==1
        phi(i) = -pi/2;
    else
        phi(i) = pi/2;
    end
end

% ALPHA
alpha = rad(alpha);


% CREATE FLEXIBLE GEOMETRY
FG = FGcreate( ...
    SGel, ...
    zeros(n,1), ... % not needed here
    heights, ...
    alpha, ...
    [NaN NaN; zeros(n-1,2)], ... % not needed here
    [NaN; zeros(n-1,1)], ... % not needed here
    phi, ...
    [NaN; zeros(n-1,1)]... % not needed here
    );

%% add frames
FG = FGaddFrames( FG );

%% SGTchain
SGres = SGTchain( FG.SGel', FG.FJalpha' );


%% plot
% VLFLplotlight(1,0.5);
% SGplot(SGres,'s');
% FGplot(FG);

end

function [ FG ] = FGaddFrames( FG )
%FGaddFrames   Adds frames to flexible geometry (FG)
%
% == INPUT ================================================================
%   FG          flexible geometry
%
% == OUTPUT ===============================================================
%   FG          returns flexible geometry with transformation 
%               matrices for creation of KMchain or SGTchain  (nx1 struct)

%% get values from FG
n = size(FG.SGel,1);
ce_h = FG.CEh;
fj_r = FG.FJr;
fj_phi = FG.FJphi;
fj_deltaX = FG.FJdx;
%% calc B frame for first element
FG.SGel{1}.Tname{1} = 'B'; 
FG.SGel{1}.T{1} = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
%% calc frames i-th element
for i = 1:n-1
    % F frame
    Rz = HTrot(rot(0,0,fj_phi(i+1))); %[[rot(0,0,fj_phi(i+1)); 0 0 0] [0; 0; 0; 1]];
    Tr = HTtr([-fj_deltaX(i+1); 0; ce_h(i)+fj_r(i+1)]); %[[eye(3); 0 0 0] [-fj_deltaX(i+1); 0; ce_h(i)+fj_r(i+1); 1]];
    Rx = HTrot(rot(pi/2,0,0)); %[[rot(pi/2,0,0); 0 0 0] [0; 0; 0; 1]];
    FG.SGel{i}.Tname{2} = 'F'; 
    FG.SGel{i}.T{2} = Rz*Tr*Rx;
    % B frame
    Tr = HTtr([-fj_deltaX(i+1); 0; -fj_r(i+1)]); %[[eye(3); 0 0 0] [-fj_deltaX(i+1); 0; -fj_r(i+1); 1]];
    Rx = HTrot(rot(pi/2,pi,0)); %[[rot(pi/2,pi,0); 0 0 0] [0; 0; 0; 1]];
    FG.SGel{i+1}.Tname{1} = 'B'; 
    FG.SGel{i+1}.T{1} = Tr*Rx;
end
%% calc F frame for last element
Tr = HTtr([0; 0; ce_h(n)]); %[[eye(3); 0 0 0] [0; 0; ce_h(n); 1]];
FG.SGel{n}.Tname{2} = 'F'; 
FG.SGel{n}.T{2} = Tr;
end

function [ FG, lbl ]...
    = FGbuild( ...
                ce_typ, ...
                fj_typ, ...
                cp_typ, ...
                ce_r, ...
                ce_h, ...
                wh_d, ...
                wh_vp, ...
                fj_alpha, ...
                fj_dim, ...
                fj_phi, ...
                fj_deltaX )
%FGbuild   Builds flexure geometry (FG)
%           Specified by values for rigid connectors (radius,
%           diameter wireholes, position wireholes, heights) and flexure
%           joints (thickness, width, phi, alpha, deltax).
%           Creates n connector elements (CEs) and n-1 flexure joints (FJ).
%
%   == INPUTS =============================================================
%   ce_typ      defines which connector type is chosen
%               -> see function "connElem.m"
%               -----------------------------------------------------------
%   fj_typ      defines which flexure joint type is chosen
%               -> see function "halfFlexJoint.m"
%               -----------------------------------------------------------
%   cp_typ      defines which contact plate type is chosen:
%               -> see function "SGtopContPlate.m" or "SGbottContPlate.m"
%               -----------------------------------------------------------
%   ce_r        Connector element radius  (double)
%               -----------------------------------------------------------
%   wh_d        diameter of wire holes, first entry must belong to wirehole
%               which is responsible for rotation of Flexure Joints (FJ)
%               m=#wireholes  (mx1 double)
%               -----------------------------------------------------------
%   wh_vp       xy position of wire holes, first row must belong to 
%               wirehole which is responsible for rotation of FJs  
%               (mx2 double)
%               -----------------------------------------------------------
%   ce_h        heights of CE(i), i = 1...n, n=#CEs  (nx1 double)
%               -----------------------------------------------------------
%   fj_alpha    angle between CE(j-1) and CE(j), j = 2...n, 
%               first entry is NaN
%               (nx1 double)
%               -----------------------------------------------------------
%   fj_dim      dimensions of flexure joint crosssection,
%               first entry is [NaN NaN]  
%               fj_dim(:,1):  FJ thickness (if rectangular), 
%                           FJ diameter (if circular)
%               fj_dim(:,2):  width of half FJ, total width: 2*fj_w
%               (nx2 double)
%               -----------------------------------------------------------
%   fj_phi      rotation angle around z axis of FJ  
%               first entry is NaN
%               (nx1 double)
%               -----------------------------------------------------------
%   fj_deltaX   distance of FJ to center on x axis, absolut value  
%               (nx1 double)
%
%   == OUTPUTS ============================================================
%   FG          returns flexible geometry  (struct)
%               STRUCTURE:
%                   FG.SGtot        returns whole flexible geometry in one 
%                                   concatenated SG  (1x1 struct)
%                   FG.SGel
%                   FG.CEh
%                   FG.CEr
%                   FG.FJalpha
%                   FG.FJdim
%                   FG.FJr
%                   FG.FJphi
%                   FG.FJdx
%               -----------------------------------------------------------
%   lbl         label with information about FG parameters (string)
%               -----------------------------------------------------------

%% global variables
% overlap delta for concatenating solid geometry (SG) parts
global delta_ol;    
if isempty(delta_ol);   delta_ol = 1e-3;    end
% width of powder removal holes
global prh_w;       
if isempty(prh_w);      prh_w = 1.5;        end

%% calc variables
% get number of CEs (nonzero values)
n = size(fj_dim,1);

% get additional values if flexible connector with interlocking is created:
% additional values are needed for creation of teeth of first element and
% holes of last elemnent
if n==size(ce_h,1)-1
    % save values in extra variables
    ce_h_interlock = ce_h(1);
    fj_alphaTop_interlock = fj_alpha(n+2);
    fj_deltaXTop_interlock = fj_deltaX(n+2);
    if n==size(fj_alpha,1)-2
        fj_alphaBott_interlock = fj_alpha(1);
        fj_deltaXBott_interlock = fj_deltaX(1);
    end
    fj_phi_interlock = fj_phi(n+1);
    % remove values from arrays
    ce_h(1) = [];
    fj_alpha(1) = [];
    fj_alpha(n+1) = [];
    fj_deltaX(1) = [];
    fj_deltaX(n+1) = [];
    fj_phi(n+1) = [];
else
    % for normal creation of FG, set values to NaN
    ce_h_interlock = NaN;
    fj_deltaXTop_interlock = NaN;
    fj_deltaXBott_interlock = NaN;
    fj_alphaTop_interlock = NaN;
    fj_alphaBott_interlock = NaN;
    fj_phi_interlock = NaN;
end

%% design checks
designCheckTypeCombinations(ce_typ,fj_typ,cp_typ)

%% preallocation
fj_r = zeros(n,1);  
fj_r(1) = NaN;
SGel = cell(n,1); 
SGhfj_bot = SGempty();
SGhfj_top = SGempty();
SGhfj_top_next = SGempty();

%% calc elements:
% first:                CE(k) + hFJ_bott(k)
% k-th:     hFJ_top(k-1) + CE(k) + hFJ_bott(k)
% second iteration variable:
k = 1;
for i = 1:n
    % CREATE COMPONENTS
    if i ~= n
        % create connector element (CE)
        if k == 1
            % EXCEPTION: first element
            [ CE, fc_flag ]...
                = connElem(...
                ce_typ,fj_typ, cp_typ,...
                ce_r,...
                [ce_h_interlock; ce_h(k)],...
                wh_d,wh_vp,...
                fj_dim(k+1,:),...
                [fj_deltaXBott_interlock; fj_deltaX(k+1)],...
                [fj_alphaBott_interlock; fj_alpha(k+1)],...
                fj_phi(k+1));
        else
            [ CE, fc_flag ]...
                = connElem(...
                ce_typ,fj_typ, cp_typ,...
                ce_r,...
                [ce_h(k-1); ce_h(k)],...
                wh_d,wh_vp,...
                fj_dim(k+1,:),...
                [fj_deltaX(k); fj_deltaX(k+1)],...
                [fj_alpha(k); fj_alpha(k+1)],...
                fj_phi(k+1));
        end
        % calc radius for flexure joint (FJ)
        fj_r(k+1) = radiusFlexJoint( ce_r, fj_alpha(k+1), fj_deltaX(k+1) );
        % design check of FJ CE combination
        designCheckFJ( ce_r, fj_dim(k+1,:), fj_r(k+1), fj_deltaX(k+1), ...
            wh_d, wh_vp, k )
        % create bottom half FJ
        SGhfj_bot = SGhalfFlexJoint(fj_typ,fj_r(k+1),fj_dim(k+1,:),...
            fj_deltaX(k+1),ce_r);
        % create top half FJ
        SGhfj_top_next = SGhfj_bot;
        % create bottom contact plate (CP)
        SGcp_bot = SGbottContPlate(cp_typ,ce_r,fj_alpha(k+1),wh_d(1),wh_vp(:,1));
        % create top contact plate (CP)
        SGcp_top = SGtopContPlate(cp_typ,ce_r,fj_alpha(k+1),wh_d(1),wh_vp(:,1));
        
        % CONCATENATE HALF FLEXURE JOINTS WITH CONTACT PLATES
        % concatenate hFJs with hCPs and rotate top halfs
        SGhfj_bot = SGcat(SGhfj_bot,SGcp_bot);
        SGhfj_top_next = SGtransR(SGcat(SGhfj_top_next,SGcp_top),...
            rot(pi,0,0));
        % rotate FJ around z axis with fj_phi(i)
        SGhfj_bot = SGtransR(SGhfj_bot,rot(0,0,fj_phi(k+1)));
    else
        % EXCEPTION: last element
        % create connector element (CE)
        [ CE, fc_flag ]...
            = connElem(...
            ce_typ,fj_typ,cp_typ,...
            ce_r,...
            [ce_h(k-1); ce_h(k)],...
            wh_d,wh_vp,...
            fj_dim(k,:),...
            [fj_deltaX(k); fj_deltaXTop_interlock],...
            [fj_alpha(k); fj_alphaTop_interlock],...
            fj_phi_interlock);
    end
    
    if fc_flag==0
        % arrange CEs and half FJs
        SGhfj_bot = SGtransP(SGhfj_bot,[0;0;ce_h(k)]);
        % CREATE ELEMENT BY CONCATENATING CONNECTOR ELEMENT WITH HALF
        % FLEXURE JOINTS
        SGel{k} = SGcat(CE,SGhfj_top,SGhfj_bot);
    else

        % ADDITIONAL LIST ENTRIES IF CONNECTOR ELEMENT ITSELF IS FLEXIBLE
        % GEOMETRY (CONSISTS OF MORE THAN ONE ELEMENT)
        % n_ce_el describes number of elements in CE
        n_ce_el = size(CE.SGel,1);
        % number of additional elements
        dk = n_ce_el-1;
        % if CE itself is a flexible geometry (n_ce_el>1): 
        % add elements to FGel
        SGel{size(SGel,1)+dk,1} = [];
        for j = 0:dk
            SGel{k+j} = CE.SGel{j+1};
        end
        % update values in value lists of CE height, alpha, fj_dim, fj_r and phi
        ce_h = [ce_h(1:k-1);...
            CE.CEh; ...
            ce_h(k+1:size(ce_h,1))];
        fj_alpha = [fj_alpha(1:k);... 
            CE.FJalpha(2:size(CE.FJalpha,1)); ...
            fj_alpha(k+1:size(fj_alpha,1))];    %size(alpha,1)
        fj_dim = [fj_dim(1:k,:);...
            CE.FJdim(2:size(CE.FJdim,1),:);...
            fj_dim(k+1:size(fj_dim,1),:)];
        fj_r = [fj_r(1:k);... 
            CE.FJr(2:size(CE.FJr,1));...
            fj_r(k+1:size(fj_r,1))];         %size(fj_r,1)
        fj_phi = [fj_phi(1:k);...
            CE.FJphi(2:size(CE.FJphi,1));...
            fj_phi(k+1:size(fj_phi,1))];
        fj_deltaX = [fj_deltaX(1:k);...
            CE.FJdx(2:size(CE.FJdx,1));...
            fj_deltaX(k+1:size(fj_deltaX,1))];
        % arrange CEs and half FJs
        SGhfj_bot = SGtransP(SGhfj_bot,[0;0;ce_h(k+dk)]);
        % add hFJ to first and last element of CE
        SGel{k} = SGcat(SGel{k},SGhfj_top);
        SGel{k+dk} = SGcat(SGel{k+dk},SGhfj_bot);
        % update iteration variable
        k = k+dk;
    end
    
    % save top hFJ for next iteration
    SGhfj_top = SGhfj_top_next;
    SGhfj_bot = SGempty();
    
    % add 1 to second iteration variable
    k = k+1;
end

%% create label
% string for connector elements
char_ce_r = strcat('CEr:',num2str(ce_r));
% string for wire holes
char_wh_d = ' WHd:';
for i = 1:max(size(wh_d))
    char_wh_d = strcat(char_wh_d,num2str(wh_d(i)),' ');
end
% string for flexure joints
char_fj_dim = strcat(' FJdim:',num2str(fj_dim(2,1)));
% create char array
lbl = strcat(char_ce_r,...
    char_wh_d,...
    char_fj_dim);

%% builds FG structure
% concatenated elements: 1 SG
FG = FGcreate(SGel, ...
    ce_r*ones(size(SGel,1),1), ce_h, ...
    fj_alpha, fj_dim, fj_r, fj_phi, fj_deltaX);

end

function [ FG ] = FGcreate( ...
    SGel, ...
    ce_r, ce_h, ...
    fj_alpha, fj_dim, fj_r, fj_phi, fj_deltaX )

%FGcreate Creates flexible geometry structure
%   == INPUTS =============================================================
%   SGel        
%               -----------------------------------------------------------
%   ce_r        
%               -----------------------------------------------------------
%   ce_h        
%               -----------------------------------------------------------
%   fj_alpha        
%               -----------------------------------------------------------
%   fj_dim        
%               -----------------------------------------------------------
%   fj_r        
%               -----------------------------------------------------------
%   fj_phi        
%               -----------------------------------------------------------
%   fj_deltaX        
%
%   == OUTPUT ==============================================================
%   FG          flexible geometry  (struct)
%               STRUCTURE:
%                   FG.SGtot
%                   FG.SGel
%                   FG.CEr
%                   FG.CEh
%                   FG.FJalpha
%                   FG.FJdim
%                   FG.FJr
%                   FG.FJphi
%                   FG.FJdx

% FG = struct(...
%     'SGtot',SGempty,...
%     'SGel',cell(1,1),...
%     'CEh',zeros(0,1),...
%     'CEr',zeros(0,1),...
%     'FJalpha',zeros(0,1),...
%     'FJdim',zeros(0,1),...
%     'FJr',zeros(0,1),...
%     'FJphi',zeros(0,1),...
%     'FJdx',zeros(0,1));

%% check structure element lengths
if ~(size(SGel,1) == size(ce_r,1) ...
        && size(SGel,1) == size(ce_h,1) ...
        && size(SGel,1) == size(fj_alpha,1) ...
        && size(SGel,1) == size(fj_dim,1) ...
        && size(SGel,1) == size(fj_r,1) ...
        && size(SGel,1) == size(fj_phi,1) ...
        && size(SGel,1) == size(fj_deltaX,1) )
    error('Flexible geometry (FG) struct is inconsistent: array sizes are not equal')
end

%% build FG structure
% concatenated elements: 1 SG
FG.SGtot = SGcatEl(SGel,ce_h,fj_r,fj_phi);
% single elements: n SGs
FG.SGel = SGel;
% radii of CEs: n values
FG.CEr = ce_r;
% heights of CEs: n values
FG.CEh = ce_h;
% maximum deflection angles of FJs: (n-1) values
FG.FJalpha = fj_alpha;
% crosssection dimension of FJs: (n-1)x2 values
FG.FJdim = fj_dim;
% radii of FJs: (n-1) values
FG.FJr = fj_r;
% z rotation of FJs: (n-1) values
FG.FJphi = fj_phi;
% delta x of FJs: (n-1) values
FG.FJdx = fj_deltaX;


end
function [ ] = FGplot(FG, frameChain )
%FGplot Plots flexible geometry

n = size(FG.SGel,1);

% %%%YK
% SGfg_ch = SGTchain2(FG.SGel');
% figure; SGplot(SGfg_ch);
% SGfg_ch = SGTchain2(FG.SGel',FG.FJalpha');
% figure; SGplot(SGfg_ch);
% %%%

figure;
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);

%% kinematic chain straight
subplot(1,2,1);
SGplot(FG.SGtot,'b');
%SGplot(SGtrans(FG.SGtot,[0.01;0;0]),'b');
SGfg_ch = SGTchain2(FG.SGel');
SGplot(SGfg_ch);
title('Chained elements (alpha(i) = 0)');
SGplotSettings1();

%% kinematic chain crooked
subplot(1,2,2);
hold on;
SGfg_ch = SGTchain2(FG.SGel',FG.FJalpha');
SGplot(SGfg_ch);

title('Chained elements, crooked');
axis equal;
view(0,0);
ax = gca;
ax.Clipping = 'off';

end

function [T] = HTrot( R )
%HTrot Returns homogeneous transformation matrix for 3D rotation
%
%   == INPUTS =============================================================  
%   v       vector (3x1 double)
%   == OUTPUTS ============================================================
%   T       homogeneous transformation matrix for 3D rotation
%

T = [R zeros(3,1); 0 0 0 1];

end

function [T] = HTtr( v )
%HTtr Returns homogeneous transformation matrix for 3D translation
%
%   == INPUTS =============================================================  
%   v       vector (3x1 double)
%   == OUTPUTS ============================================================
%   T       homogeneous transformation matrix for 3D translation
%

T = [eye(3) v; 0 0 0 1];

end

function [ VLprojection ] = PLtoVLprojection(PL, PLcontour)
%PLtoVLprojection Projection of a 2D-PL (point list) towards a vertical 2D-contour. The result is a
%   3D-VL (vertex list) list of the projected PL (point list). 
%   PL: point list (2D; x-y-plane)
%   PLcontour: Contour to project on as PL. The contour will be transfered
%   into vertical plane (x-z-plane) for the projection.
%   
%   Example: 
%   PL=PLcircseg(3,100,0,2*pi);
%   PLcontour = PLcircseg(4,1000,0,pi);
%   PLcontour = [PLcontour(:,1) PLcontour(:,2)+1];
%   VLprojection = PLtoVLprojection(PL, PLcontour);
%   PLplot(PL); plot3(PLcontour(:,1),zeros(size(PLcontour,1),1), PLcontour(:,2),'b');VLplot(VLprojection);
% 



% %%%%%%EXAMPLE PARAMETERS
% phi=0;
% PL=PLcircseg(3,100,0,2*pi);
% % contourfunction = @(phi)sin(phi); 
% VLcontour = VLaddz(PLcircseg(10,50,0,pi),0);
% VLcontour =[VLcontour(:,1) VLcontour(:,3)+2 VLcontour(:,2)]; % VL has to be vertically to PL contour
% PLcontour = PLcircseg(4,1000,0,pi);
% PLcontour = [PLcontour(:,1) PLcontour(:,2)+1];
% %%%%%%

% PL=PLcircseg(3,100,0,2*pi);% 
% xPL= linspace(0,12,500);
% yPL= 5 - xPL*tand(20);
% xPLminus= -flip(xPL);
% yPLminus= flip(yPL);
% xPL = [xPLminus,xPL];
% yPL = [yPLminus,yPL];
% PLcontour = [xPL' yPL'];
% PLcontour = [PLcontour(:,1) PLcontour(:,2)+1];
% 
% direction = 1;
%%%%%%%%


%convert PL contour to VLcontour (contour in x-z-plane and y=0)
VLcontour = [PLcontour(:,1) zeros(size(PLcontour,1),1) PLcontour(:,2) ]; %create  VLcontour to plot it in vertical plane (x-z-plane)

%PL of contour, that shall be projected onto PLcontour, create VLprojection
VLprojection = [PL(:,1) PL(:,2) zeros(size(PL,1),1)]; %create VL

%find closest value in x for protection
for i = 1:size(VLprojection,1)
[~,idx]=min(abs(VLprojection(i,1)-VLcontour(:,1)));
Minidx(i)=idx; % gives index of the fittet fct x1 corresponding to the point x(i) 
% idx
VLprojection(i,3) = VLcontour(idx,3); %procetion by defining a z-value for the 2D-PL

end;

% FIND FUNCTION FOR CONTOUR
% % %find the edges of the given contour
% % xmin = min(PLcontour(:,1));
% % xmax = max(PLcontour(:,1));
% % 
% % if xmax >=0 xspace=(xmax-xmin); %find lenght 
% % elseif xmin<0 && xmax < 0 xspace=(abs(xmax)-abs(xmin)); 
% % end;
% % %fit curve
% % VLcontour = [PLcontour(:,1) zeros(size(PLcontour,1),1) PLcontour(:,2) ]; %create  VLcontour to plot it in vertical plane (x-z-plane)
% % x = VLcontour(:,1);
% % y = VLcontour(:,2);
% % z = VLcontour(:,3);
% % 
% % p = polyfit(x,z,20); % find a function (polynom grad 20 that defines the contour
% % x1 = linspace(xmin,xmax,xspace*100); % resolution 0.01mm
% % y1 = linspace(0,0,xspace*100); % define y-column with y=0
% % z1 = polyval(p,x1);
% % for i = 1:size(VLprojection,1)
% % [~,idx]=min(abs(VLprojection(i,1)-x1));
% % Minidx(i)=idx; % gives index of the fittet fct x1 corresponding to the point x(i) 
% % % idx
% % VLprojection(i,3) = z1(idx); %procetion by defining a z-value for the 2D-PL
% % 
% % end;


% hold on
% VLplot(VLcontour)
% plot3(x1,y1,z1,'b'); 

% PLplot(PL,'r');
% VLplot(VLprojection,'g');
% plot3(PLcontour(:,1),zeros(size(PLcontour,1),1), PLcontour(:,2),'b')
% view(0,0) ;
% axis equal 
end

function [ SGtot ] = SGcatEl(SGel,ce_h,fj_r,fj_phi)
%SGcatEl Concatenate Elements to get flexure geometry as one SG 
%
%   == INPUTS =============================================================
%   SGel        
%               -----------------------------------------------------------
%   ce_h        
%               -----------------------------------------------------------
%   fj_r        
%               -----------------------------------------------------------
%   fj_phi        
%
%   == OUTPUTS ============================================================
%   SGtot
%

%% concatenate Elements to get flexure geometry as one SG 
% cumulative sums of values
ce_h_added = cumsum(ce_h);
fj_r_added = cumsum(fj_r(2:size(fj_r,1)));
fj_phi_added = cumsum(fj_phi(2:size(fj_phi,1)));
% new number of elements
n = size(SGel,1);
% start element
SGtot = SGel{1};
for i = 2:n
    SGel_i = SGtransR(SGel{i},rot(0,0,fj_phi_added(i-1)));
    SGel_i = SGtransP(SGel_i,[0;0;ce_h_added(i-1)+2*fj_r_added(i-1)]);
    SGtot = SGcat(SGtot,SGel_i);
end

end

function [ SGfj ] = SGhalfflexurehingeelement( jointRadius,jointWidth, jointThickness, jointAngle)
%Designgs a SG (solid geometrie) of a flexure hinge element
%   Detailed explanation goes here

% jointRadius = 1.5; %xls-Table
% jointWidth = 0.75; %xls-Table
% jointThickness = 5; % Thickness of the flexure joint of section 2 of the manipulator arm %xls-Table
% jointAngle= 90;
diameterWirehole= 1; %xls-Table
overlapDelta = 1e-3; % fixed value has to be sored generally (.xls-Table) !!

%jointAngle between 0° and 90°
if jointAngle > 90
    msgbox("Maximum joint angle is 90°");
    jointAngle=90;
elseif jointAngle < 0
    jointAngle=0;
end

fhAngle = (90-jointAngle/2)/180*pi; % calculate angle to design FH and convert to rad

%PL: contour of the flexure joint
offsetX = (jointWidth/2 + jointRadius);

PLleftside = flip(PLtransP(PLcircseg(jointRadius,20,-fhAngle,0),[-offsetX;0;0]));%FH depening on jointRadius
VLrightside = VLswapX([PLleftside(:,1),PLleftside(:,2),0*PLleftside(:,2)]);

PLconnector = PLcircseg(jointWidth/2,180/9,pi,0);

% CPL = PLtransP(CPL,[0;jointRadius]); % set flexure joint tangential to x-axis

% CPL = [flipud(CPL(1:size(CPL,1)/2-1,:));PLconnector(2:end-1,:); flipud(CPL(size(CPL,1)/2:end-2,:))];
CPL=[flipud(PLleftside);PLconnector(2:end-1,:);VLrightside(:,1:2)];


%Create SG
SGfj = SGofCPLz(CPL,jointThickness); % Create Solid Geometry

%Orientation of SGfj
SGfj = SGtransT(SGfj,[1 0 0 0;...
                       0 cos(pi/2) -sin(pi/2) jointThickness/2;...
                       0 sin(pi/2) cos(pi/2) 0;...
                       0 0 0 1]); 
   
% % center of flexure hinge as origin
% SGfj = SGtrans(SGfj,[0 ;0 ;-max(SGfj.VL(:,3))]);

%  %TEST
% SGplot(SGfj);
end

function [SGsection1] = SGovertubSection(CPL,jointAngle,jointThickness, jointWidth, jointRadius,rigidThickness)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


% CPL = CPL_G;
% jointAngle = 20;
% rigidThickness = 1;
% jointRadius = 1.0; %xls-Table
% jointWidth = 0.75; %xls-Table
% jointThickness = 0.8;


height = max(CPL(:,1))*tand(jointAngle/2);

%PLcontour
xPL= linspace(0,12,500);
yPL= height - xPL*tand(jointAngle/2);
xPLminus= -flip(xPL);
yPLminus= flip(yPL);
xPL = [xPLminus,xPL];
yPL = [yPLminus,yPL];
PLcontour = [xPL' yPL'];
PLcontour = [PLcontour(:,1) PLcontour(:,2)+rigidThickness/2];

% SG = SGofCPLz(CPL,0.1);
SG =  SGofCPLzdelaunayGrid(CPL, 0.1,0.5,0.5);
n=size(SG.VL,1);
% PLup=[SG.VL(1:n/2,1) SG.VL(1:n/2,2)];
PLup=[SG.VL(n/2+1:end,1) SG.VL(n/2+1:end,2)];
VLprojection = PLtoVLprojection(PLup, PLcontour);
SG.VL = [SG.VL(1:n/2,1) SG.VL(1:n/2,2) SG.VL(1:n/2,3);VLprojection];





SGhfh = SGhalfflexurehingeelement(jointRadius,jointWidth, jointThickness, jointAngle);

SGhfh1 = SGalignbottom(SGhfh,SG);
SGhfh1 = SGtransP(SGhfh1,[0; 0 ;max(SG.VL(:,3))-max(SGhfh.VL(:,1))*tand(jointAngle/2)]);
SGhfh2 = SGalignback(SGhfh1,SG);
SGhfh1 = SGalignfront(SGhfh1,SG);

%Double the halfs
SGsection1 = SGcat(SG,SGhfh1,SGhfh2);
% SGsection1 = SGcat(SGsection1,  SGtransT(SGsection1,[...
%                        cos(pi) 0 sin(pi) 0;...
%                        0 1 0 0;...
%                        -sin(pi) 0 cos(pi) 2*max(SGsection1.VL(:,3))-jointThickness;...
%                        0 0 0 1]));
%                    

%PLOT
% SGplot(SGsection1)
% view (0,0)
% axis equal

end

function [ ] = SGplotSettings1()
%SGplotSettings1 Settings 1 for SGplot

axis equal;
view(0,0);
ax = gca;
ax.Clipping = 'off';

end







