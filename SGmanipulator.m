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
function [SG,SGc] = SGmanipulator(CPL_out,tool_r,angle_p,length_p,varargin) 
single=0; sensor_channel=0; side_stabi = 0; 
c_inputs = {};
for f=1:size(varargin,2)
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
      end   
end

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
hole_r = 0.9;                          % Radius of rope/pushrodtubes
%% Initializing arrays and variables
arm = {};
SG_elements = {};
CPL_com ={};
offsets = [];
progress = 0;
total_progress = (2*size(angle_p,1))+2;
%% Finding Positions of holes and creating CPLs
[CPLs_holes,positions] = PLholeFinder(CPL_out,tool_r,angle_p(:,[1,4]),hole_r,single); updateProgress;
%%  Creating elements and connectors
SG_bottom = SGelements(CPLbool('-',CPLs{1},CPLs_holes{1}),angle_p(1,:),'','','bottom_element'); updateProgress;
SG_conns = {SG_bottom};
for i=1:size(angle_p,1)
    CPL_com{end+1} = CPLbool('-',CPLs{i},CPLs_holes{i});
    
    if i == size(angle_p,1)   %% Top of arms
        if single == 1
            SG_conn_temp = SGconnector(CPLs(size(angle_p,1)),CPLs_holes(end),positions(end,:),[angle_p(end,:);angle_p(end,:)],hole_r,tool_r,length_p(i,3),'',{'end_cap' 'single'}); 
        else
            SG_conn_temp = SGconnector(CPLs(size(angle_p,1)),CPLs_holes(end),positions(end,:),[angle_p(end,:);angle_p(end,:)],hole_r,tool_r,length_p(i,3),'',{'end_cap'}); 
        end  
         SG_conn_temp.hole_positions = positions(end,:);
    else
        SG_conn_temp = SGconnector(CPLs(i:i+1),CPLs_holes(i:i+1),positions(i:i+1,:),angle_p(i:i+1,:),hole_r,tool_r,length_p(i,3),length_p(i+1,3),c_inputs); 
        SG_conn_temp.hole_positions = positions(i:i+1,:);
    end    
    
    [SG_ele_temp, offset] = SGelements(CPL_com{i},angle_p(i,:),length_p(i,3),length_p(i,2)); 
    if i == size(angle_p,1)   
        SG_ele_temp.hole_positions = positions(end,:);
    else
        SG_ele_temp.hole_positions = positions(i:i+1,:);
    end
    if ~isempty(SG_conn_temp) SG_conns{end+1} = SG_conn_temp;   SG_conn_temp = []; end    
    SG_elements{end+1} = SG_ele_temp; 
    offsets = [offsets offset];
    updateProgress;
end
%% Calculating number of elements && adding stops
ele_num =[];
for i=1:size(SG_elements,2)
    ele_num = [ele_num floor(length_p(i,1)/(length_p(i,2)+1))];
    
    max_dim = max(sizeVL(CPL_out{i}))+1;
    middle_axis = PLtransR(PLtrans([-max_dim 0;max_dim 0],[0 offsets(i)]),rot(deg2rad(angle_p(i,1))));
    e_dir = (middle_axis/norm(middle_axis))*rot(pi/2);
    e_dir = (e_dir(1,:)-e_dir(2,:))/norm(e_dir(1,:)-e_dir(2,:));
    left_plane = [flip(middle_axis);PLtrans(middle_axis,e_dir*max_dim)]; % Plane for finding points in positive area
    right_plane = [flip(middle_axis);PLtrans(middle_axis,e_dir*-max_dim)];    
    
    CPL_left = CPLbool('-',CPL_out{i},left_plane);
    CPL_right = CPLbool('-',CPL_out{i},right_plane);
    dis_left = 0;
    dis_right = 0;
    for k = 1:size(CPL_left,1)
        dis_temp = distPointLine(middle_axis,CPL_left(k,:));
        if dis_temp > dis_left dis_left = dis_temp; end
    end
     for k = 1:size(CPL_right,1)
        dis_temp = distPointLine(middle_axis,CPL_right(k,:));
        if dis_temp > dis_right dis_right = dis_temp; end
     end
    
     phi_left = 2*atand(0.5/dis_left);
     phi_right = 2*atand(0.5/dis_right);
     
     phi_left_p = angle_p(i,2)/ele_num(i);
     phi_right_p = angle_p(i,3)/ele_num(i);     
     
     SG_stops = SGtrans(SGelementstops(CPL_com{i},angle_p(i,1),phi_left_p,phi_right_p,1.2,offsets(i)),[0 0 length_p(i,2)/2]);
     SG_elements{i} = SGcat(SG_elements{i},SG_stops);     
     SG_elements{i} = SGcat(SG_elements{i},SGmirror(SG_stops,'xy'));
     SG_conns{i} = SGcat(SG_conns{i},SGtransrelSG(SG_stops,SG_conns{i},'ontop',-0.5-length_p(i,3)/2));
     SG_conns{i+1} = SGcat(SG_conns{i+1},SGtransrelSG(SGmirror(SG_stops,'xy'),SG_conns{i+1},'under',-0.5-length_p(i,3)/2));
    
end
%% Filling cell list with elements for a single arm
for j=1:size(SG_elements,2)
    arm = [arm repelem(SG_elements(j),ele_num(j))];
    arm = [arm  SG_conns(j+1)];
end
arm = [SG_conns(1) arm];
%% Adding base to arms in a cell list
base = SGmanipulatorbase([CPLs_holes{1};NaN NaN;CPL_in],CPL_out{1},3,positions(1,:),1,sensor_channel);
SGs = [{base} arm arm];
num = size(SGs,2);
%% Generating full manipulator with framechain
framechain = SGTframeChain2(num);
phis = [repmat(-0.095,1,6) zeros(1,27)];
SGc = SGTchain(SGs,[0 0 phis phis],'',framechain);
SG = SGcat(SGc);
SGplot(SG);
    function updateProgress
        progress = progress+1;
        disp(floor((progress/total_progress)*100)+"%");
    end

end