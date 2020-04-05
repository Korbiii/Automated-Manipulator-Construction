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
single=0; sensor_channel=0; side_stabi = 0; base_length = 7; optic = 0;optic_radius = 3; seal = 0; torsion = 0; bottom_up = 0; hole_r = 0.7; angles = [];
c_inputs = {}; skip_next = 0;
for f=1:size(varargin,2)
    if skip_next skip_next = 0; continue; end
      switch varargin{f}
          case 'single'
              c_inputs{end+1} = 'single';
              single = 1;
              crimp = 0;
          case 'sensor_channel'
              sensor_channel = 1;
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
              skip_next = 1;  
          case 'length'
              base_length = varargin{f+1};
             skip_next = 1;  
          case 'seal'
              seal = 1;
          case 'torsion'
              torsion = 1;
          case 'bottom_up'
              bottom_up = 1;    
          case 'angles'
              angles = varargin{f+1};
              skip_next = 1;  
          case 'hole_radius'
              hole_r = varargin{f+1};
              skip_next = 1;      
      end   
end   
tool_r = tool_d/2;
%% Setting up variables
CPLs = {};
if ~iscell(CPL_out)
    CPL_out = {CPL_out};
end
while size(CPL_out,2)<size(angle_p,1)
    CPL_out{end+1} = CPL_out{end};
end
CPL_out = CPL_out';
CPL_in = PLcircle(tool_r,tool_r*20);
for i=1:size(CPL_out,1)
    if sensor_channel          
        CPL_in = CPLbool('-',CPL_in,PLtrans(PLcircle(tool_r),[0 tool_r+1.75]));
        CPL_in = [CPL_in;NaN NaN;PLtrans(PLcircle(1.4),[0 tool_r])];
    end
    CPLs{end+1} = CPLbool('-',CPL_out{i},CPL_in);
end                         
%% Initializing arrays and variables
arm = {};
SG_elements = {};
CPL_com ={};
offsets = [];
ranges = [];
s_n = 0;
%% Finding Positions of holes and creating CPLs
[CPLs_holes,positions] = PLholeFinder(CPL_out,tool_r,angle_p(:,[1,4]),length_p(:,3:5),hole_r,single,torsion,bottom_up); 
updateProgress("Rope channels created");
%%  Creating elements and connectors
SG_bottom = SGelements(CPLbool('-',CPLs{1},CPLs_holes{1}),angle_p(1,[1,4]),length_p(1,3),length_p(1,3),length_p(1,4),length_p(1,5),'bottom_element');
SG_conns = {SG_bottom};
for i=1:size(angle_p,1)
    CPL_com{end+1} = CPLbool('-',CPLs{i},CPLs_holes{i});    
    if i == size(angle_p,1)   %% Top of arms
        if single == 1
            SG_conn_temp = SGconnector(CPLs(size(angle_p,1)),CPLs_holes(end),positions(end,:),[angle_p(end,[1,4]);angle_p(end,[1,4])],hole_r,tool_r,length_p(i,3),'',length_p(i,4),length_p(i,5),'',{'end_cap' 'single'}); 
        else
            SG_conn_temp = SGconnector(CPLs(size(angle_p,1)),CPLs_holes(end),positions(end,:),[angle_p(end,[1,4]);angle_p(end,[1,4])],hole_r,tool_r,length_p(i,3),'',length_p(i,4),length_p(i,5),'',{'end_cap'}); 
        end  
         SG_conn_temp.hole_positions = positions(end,:);
    else
        SG_conn_temp = SGconnector(CPLs(i:i+1),CPLs_holes(i:i+1),positions(i:i+1,:),angle_p(i:i+1,[1,4]),hole_r,tool_r,length_p(i,3),length_p(i+1,3),length_p(i,4),length_p(i,5),length_p(i+1,5),c_inputs); 
        SG_conn_temp.hole_positions = positions(i:i+1,:);
    end    
    
    [SG_ele_temp, offset] = SGelements(CPL_com{i},angle_p(i,[1,4]),length_p(i,3),length_p(i,2),length_p(i,4),length_p(i,5)); 
    if i == size(angle_p,1)   
        SG_ele_temp.hole_positions = positions(end,:);
    else
        SG_ele_temp.hole_positions = positions(i:i+1,:);
    end
    if ~isempty(SG_conn_temp) SG_conns{end+1} = SG_conn_temp;   SG_conn_temp = []; end    
    SG_elements{end+1} = SG_ele_temp; 
    offsets = [offsets offset];
    updateProgress("Element "+ i + " and Connector " + i + " created!");
end
%% Calculating number of elements && adding stops
ele_num =[];
length_p = [0 0 0 0 0;length_p;0 0 0 0 0];
angle_p = [0 0 0 0;angle_p;0 0 0 0];
for i=2:size(SG_elements,2)+1
    ele_num = [ele_num floor(length_p(i,1)/(length_p(i,2)+(2*length_p(i,5))))];    
    max_dim = max(sizeVL(CPL_out{i-1}))+1;
    middle_axis = PLtransR(PLtrans([-max_dim 0;max_dim 0],[0 offsets(i-1)]),rot(-deg2rad(angle_p(i,1))));
    
    phi_left = max(1,(angle_p(i,2)/ele_num(i-1)-1)/2);
    phi_right = max(1,(angle_p(i,3)/ele_num(i-1)-1)/2);
    
    SG_el = SGstops(SG_elements{i-1},angle_p(i,1),offsets(i-1),phi_left,phi_right,length_p(i,[3,5]));
    SG_elements{i-1} = SG_el;
    [SG_con] = SGstops(SG_conns(i-1:i),angle_p(i,1),offsets(i-1),phi_left,phi_right,length_p(i-1:i+1,[3,5]));
    SG_conns{i-1} = SG_con{1};
    SG_conns{i} = SG_con{2};
    
    dis_axis_pos = distPointLine(middle_axis,positions(i-1,:));
    height_l = 2*(tand(phi_right)*dis_axis_pos);
    height_r = 2*(tand(phi_left)*dis_axis_pos);
    
    ranges = [ranges;max(0,height_l*ele_num(i-1)),max(0,height_r*ele_num(i-1))];
    SG_elements{i-1}.phi =  [phi_left*2,phi_right*2];
    SG_conns{i-1}.phi_t = [phi_left*2,phi_right*2];
    SG_conns{i}.phi_b = [phi_left*2,phi_right*2];
    updateProgress("Added Stops to Element "+ i);
end
%% Filling cell list with elements for a single arm
for j=1:size(SG_elements,2)
    arm = [arm repelem(SG_elements(j),ele_num(j))];
    arm = [arm  SG_conns(j+1)];
end
arm = [SG_conns(1) arm];
%% Adding base to arms in a cell list
base = SGmanipulatorbase([CPLs_holes{1};NaN NaN;CPL_in],CPL_out{1},optic_radius,positions(1,:),sensor_channel,optic,single,base_length,seal);
SGs = [{base} arm arm];
num = size(SGs,2);
updateProgress("Created Base");
%% Generating full manipulator with framechain
framechain = SGTframeChain2(num);
phis2 = 0;
phis = 0;
if isempty(angles)
    for i=1:size(ele_num,1)
        phis = [phis zeros(1,ele_num(i)+1)];
        phis2 = [phis2 zeros(1,ele_num(i)+1)];
    end
else
    for i=1:size(ele_num,2)
        if angles(1,i)>=0
            phis = [phis repmat(SG_elements{i}.phi(2)*angles(1,i),1,ele_num(i)+1)];
        else            
            phis = [phis repmat(SG_elements{i}.phi(1)*angles(1,i),1,ele_num(i)+1)];
        end
        if angles(2,i)>=0
            phis2 = [phis2 repmat(SG_elements{i}.phi(2)*angles(2,i),1,ele_num(i)+1)];
        else            
            phis2 = [phis2 repmat(SG_elements{i}.phi(1)*angles(2,i),1,ele_num(i)+1)];
        end
        
    end
end
phis = deg2rad(phis);
phis2 = deg2rad(phis2);
SGc = SGTchain(SGs,[0 phis 0 phis2],'',framechain);
updateProgress("Created SGTchain");
SG = SGcat(SGc);
SGplot(SG);

    function updateProgress(message)
        s_n = s_n+1;
        disp((floor(s_n/(3+size(angle_p,1)*2)*100)) +"% "+message);
    end

end