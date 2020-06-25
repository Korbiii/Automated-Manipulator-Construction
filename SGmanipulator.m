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
%	=== OUTPUT RESULTS =====
%	SG:                 SG of Manipulator
%   SGc:                SGTchain of Manipulator
function [SG,SGc,ranges,framechain,phis] = SGmanipulator(CPL_out,tool_d,angle_p,length_p,varargin) 
base_length = 7;optic_radius = 2.2;hole_r = 0.7; num_arms = 2;
angle_defaults = [90 90 0];
length_defaults = [2 1 0.25 0.5];
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

[single,side_stabi,sensor_channel,optic,seal,symmetric,torsion,bottom_up,radial] = deal(0);
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
            SG_conn_temp.hole_positions = positions(end,:);
        else
            SG_conn_temp = SGconnector(CPLs{k}(i:i+1),CPLs_holes{k}(i:i+1),positions{k}(i:i+1),angle_p{k}(i:i+1,[1,4]),length_p{k}(i:i+1,3:5),hole_r,tool_r(k),c_inputs);
%             SG_conn_temp = SGconnector(CPLs{k}(i:i+1),CPLs_holes{k}(i:i+1),positions{k}(i:i+1),angle_p{k}(i:i+1,[1,4]),hole_r,tool_r(k),length_p{k}(i,3),length_p{k}(i+1,3),length_p{k}(i,4),length_p{k}(i,5),length_p{k}(i+1,5),c_inputs);
            SG_conn_temp.hole_positions = positions{k}(i:i+1);
        end
       if angle_p{k}(i,4)==2
             angle_p{k}(i,1) = angle_p{k}(i,1)-90;
         end
        
        [SG_ele_temp, offset] = SGelements(CPL_combined,angle_p{k}(i,[1,4]),length_p{k}(i,2:5));
        if size(SG_ele_temp,2) == 1
            if i == num_sections(k)
                SG_ele_temp.hole_positions = positions{k}(end);
            else
                SG_ele_temp.hole_positions = positions{k}(i:i+1);
            end
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
        ele_num_temp = [ele_num_temp floor(length_p{k}(i,1)/(length_p{k}(i,2)+(2*length_p{k}(i,5))))];
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
for i=1:num_arms
    first_positions = [first_positions;positions{i}{1}];
end
base = SGmanipulatorbase(CPL_combis,optic_radius,first_positions,sensor_channel,optic,single,base_length,seal,radial);
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

fprintf(fileID,'Arduino Code mit erzeugtem Code ersetzen!\n');

fclose(fileID);

% SGplot(SG);


end