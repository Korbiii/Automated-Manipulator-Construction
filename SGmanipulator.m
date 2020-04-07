%%   [SG]=SGmanipulator(CPL,tool_r,M_paras)
%	=== INPUT PARAMETERS ===
%	CPL_out:        CPL of outer contour of elements
%   tool_r:         tool hole at [0 0] with radius tool_r
%	M_paras:        3xn Vector of DoFs [direction_angle total_angle offset]
%   length_p:       2xn Vector section length [section_length element_height]
%   flags:          'single';'sensor_channel','side_stabi','first_single'
%	=== OUTPUT RESULTS ======
%	SG:         SG of Manipulator
%   SGc:        SGTchain of Manipulator
function [SG,SGc,ranges] = SGmanipulator(CPL_out,tool_d,angle_p,length_p,varargin) 
base_length = 7;optic_radius = 3;hole_r = 0.7; num_arms = 2;
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

[single,side_stabi,sensor_channel,optic,seal,symmetric,torsion,bottom_up] = deal(0);
for f=1:size(varargin,2)
if ~ischar(varargin{f}) continue; end
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
          case 'num_arms'
              num_arms = varargin{f+1};
          case 'symmetric'
              symmetric = 1;
              num_arms = 2;
      end   
end

%% Setting up variables
tool_r = tool_d/2;
angle_p = cell2mat(angle_p');
length_p = cell2mat(length_p');
angle_p = [angle_p repelem(angle_defaults(1,size(angle_p,2):end),size(angle_p,1),1)];
length_p = [length_p repelem(length_defaults(1,size(length_p,2):end),size(length_p,1),1)];

if symmetric || num_arms == 1
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
        CPL_out{i} = [CPL_out{i};repmat(CPL_out{1,i}(end),size(angle_p,1)/num_arms-size(CPL_out{1,i},1),1)];
        CPL_in{end+1} =  PLcircle(tool_r(i),tool_r(i)*20);
    end
end

num_sections = size(angle_p,1)/num_arms;
for i=1:num_arms
    if symmetric && i > 1 
         CPLs{end+1} = CPLs{end};
        continue; 
    end
    CPLs_temp = {};
    for j=1:size(CPL_out{1,i},1)
        CPLs_temp{end+1,1} = CPLbool('-',CPL_out{1,i}{j},CPL_in{i});
    end
    CPLs{end+1} = CPLs_temp;
end                         
%% Initializing arrays and variables
arms = {};
SG_elements = {};
offsets = {};

ranges = [];
CPLs_holes = {};
positions = [];
SG_arms = {};
SG_conns = {};
s_n = 0;
length_p = mat2cell(length_p,repmat(num_sections,1,num_arms));
angle_p = mat2cell(angle_p,repmat(num_sections,1,num_arms));

%% Finding Positions of holes and creating CPLs
for i=1:num_arms
    [CPLs_holes{end+1},positions_temp] = PLholeFinder(CPL_out{i},tool_r(i),angle_p{i}(:,[1,4]),length_p{i}(:,3:5),hole_r,single,torsion,bottom_up); 
    positions = [positions; positions_temp];
end
positions = mat2cell(positions,repmat(num_sections,1,num_arms));
updateProgress("Rope channels created");
%%  Creating elements and connectors

for k=1:num_arms
    SG_elements_temp = {}; 
    offsets_temp = [];
    SG_bottom = SGelements(CPLbool('-',CPLs{k}{1},CPLs_holes{k}{1}),angle_p{k}(1,[1,4]),length_p{k}(1,3),length_p{k}(1,3),length_p{k}(1,4),length_p{k}(1,5),'bottom_element');
    SG_conns_temp = {SG_bottom};
    for i=1:num_sections
        CPL_combined = CPLbool('-',CPLs{k}{i},CPLs_holes{k}{i});
        if i == num_sections   %% Top of arms
            if single == 1
                SG_conn_temp = SGconnector(CPLs{k}(num_sections),CPLs_holes{k}(end),positions{k}(end,:),[angle_p{k}(end,[1,4]);angle_p{k}(end,[1,4])],hole_r,tool_r(k),length_p{k}(i,3),'',length_p{k}(i,4),length_p{k}(i,5),'',{'end_cap' 'single'});
            else
                SG_conn_temp = SGconnector(CPLs{k}(num_sections),CPLs_holes{k}(end),positions{k}(end,:),[angle_p{k}(end,[1,4]);angle_p{k}(end,[1,4])],hole_r,tool_r(k),length_p{k}(i,3),'',length_p{k}(i,4),length_p{k}(i,5),'',{'end_cap'});
            end
            SG_conn_temp.hole_positions = positions(end,:);
        else
            SG_conn_temp = SGconnector(CPLs{k}(i:i+1),CPLs_holes{k}(i:i+1),positions{k}(i:i+1,:),angle_p{k}(i:i+1,[1,4]),hole_r,tool_r(k),length_p{k}(i,3),length_p{k}(i+1,3),length_p{k}(i,4),length_p{k}(i,5),length_p{k}(i+1,5),c_inputs);
            SG_conn_temp.hole_positions = positions{k}(i:i+1,:);
        end
        
        [SG_ele_temp, offset] = SGelements(CPL_combined,angle_p{k}(i,[1,4]),length_p{k}(i,3),length_p{k}(i,2),length_p{k}(i,4),length_p{k}(i,5));
        if i == num_sections
            SG_ele_temp.hole_positions = positions{k}(end,:);
        else
            SG_ele_temp.hole_positions = positions{k}(i:i+1,:);
        end
        if ~isempty(SG_conn_temp) SG_conns_temp{end+1} = SG_conn_temp;   SG_conn_temp = []; end
        SG_elements_temp{end+1} = SG_ele_temp;
        offsets_temp = [offsets_temp offset];
        updateProgress("Element "+ i + " and Connector " + i + " created!");
    end
    SG_elements{end+1} = SG_elements_temp;
    SG_conns{end+1} = SG_conns_temp;
    offsets{end+1} = offsets_temp;
end


%% Calculating number of elements && adding stops
ele_num = {};
for k = 1:num_arms
    length_p{k} = [0 0 0 0 0;length_p{k};0 0 0 0 0];
    angle_p{k} = [0 0 0 0;angle_p{k};0 0 0 0];
end
for k = 1:num_arms
    ele_num_temp = [];
    for i=2:num_sections+1
        ele_num_temp = [ele_num_temp floor(length_p{k}(i,1)/(length_p{k}(i,2)+(2*length_p{k}(i,5))))];
        max_dim = max(sizeVL(CPL_out{k}{i-1}))+1;
        middle_axis = PLtransR(PLtrans([-max_dim 0;max_dim 0],[0 offsets{k}(i-1)]),rot(-deg2rad(angle_p{k}(i,1))));
        
        phi_left = max(1,(angle_p{k}(i,2)/ele_num_temp(i-1)-1)/2);
        phi_right = max(1,(angle_p{k}(i,3)/ele_num_temp(i-1)-1)/2);
        
        SG_el = SGstops(SG_elements{k}{i-1},angle_p{k}(i,1),offsets{k}(i-1),phi_left,phi_right,length_p{k}(i,[3,5]));
        SG_elements{k}{i-1} = SG_el;
        [SG_con] = SGstops(SG_conns{k}(i-1:i),angle_p{k}(i,1),offsets{k}(i-1),phi_left,phi_right,length_p{k}(i-1:i+1,[3,5]));
        SG_conns{k}{i-1} = SG_con{1};
        SG_conns{k}{i} = SG_con{2};
        
        dis_axis_pos = distPointLine(middle_axis,positions{k}(i-1,:));
        height_l = 2*(tand(phi_right)*dis_axis_pos);
        height_r = 2*(tand(phi_left)*dis_axis_pos);
        
        ranges = [ranges;max(0,height_l*ele_num_temp(i-1)),max(0,height_r*ele_num_temp(i-1))];
        SG_elements{k}{i-1}.phi =  [phi_left*2,phi_right*2];
        SG_conns{k}{i-1}.phi_t = [phi_left*2,phi_right*2];
        SG_conns{k}{i}.phi_b = [phi_left*2,phi_right*2];
        updateProgress("Added Stops to Element "+ i);
    end
    ele_num{end+1} = ele_num_temp;
end
%% Filling cell list with elements for a single arm
for k=1:num_arms
    arm_temp = [];
    for j=1:num_sections
        arm_temp = [arm_temp repelem(SG_elements{k}(j),ele_num{k}(j))];
        arm_temp = [arm_temp  SG_conns{k}(j+1)];
    end    
    arms{end+1} = [SG_conns{k}(1) arm_temp];
end
%% Adding base to arms in a cell list
base = SGmanipulatorbase([CPLs_holes{1}{1};NaN NaN;CPL_in{1}],CPL_out{1}{1},optic_radius,positions{1}(1,:),sensor_channel,optic,single,base_length,seal);
SGs = [{base} arms{1} arms{2}];
num = size(SGs,2);
updateProgress("Created Base");
%% Generating full manipulator with framechain
framechain = SGTframeChain2(num,sum(ele_num{1})+size(ele_num{1},2)+1);
phis2 = 0;
phis = 0;
if isempty(angles)
    for i=1:num_sections
        phis = [phis zeros(1,ele_num{1}(i)+1)];
        phis2 = [phis2 zeros(1,ele_num{2}(i)+1)];
    end
else
    for i=1:num_sections
        if angles(1,i)>=0
            phis = [phis repmat(SG_elements{1}{i}.phi(2)*angles(1,i),1,ele_num{1}(i)+1)];
        else
            phis = [phis repmat(SG_elements{1}{i}.phi(1)*angles(1,i),1,ele_num{1}(i)+1)];
        end
        if angles(2,i)>=0
            phis2 = [phis2 repmat(SG_elements{2}{i}.phi(2)*angles(2,i),1,ele_num{2}(i)+1)];
        else
            phis2 = [phis2 repmat(SG_elements{2}{i}.phi(1)*angles(2,i),1,ele_num{2}(i)+1)];
        end
    end
    end
phis = deg2rad(phis);
phis2 = deg2rad(phis2);
SGc = SGTchain(SGs,[0 phis phis2],'',framechain);
updateProgress("Created SGTchain");
SG = SGcat(SGc);
SGplot(SG);

    function updateProgress(message)
        s_n = s_n+1;
        disp((floor(s_n/(3+size(angle_p,1)*2)*100)) +"% "+message);
    end

end