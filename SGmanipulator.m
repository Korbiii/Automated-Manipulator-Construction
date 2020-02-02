%%   [SG]=SGmanipulator(CPL,tool_r,M_paras)
%	=== INPUT PARAMETERS ===
%	CPL_out:        CPL of outer contour of elements
%   tool_r:         tool hole at [0 0] with radius tool_r
%	M_paras:        3xn Vector of DoFs [direction_angle total_angle offset]
%   flags:          'single';'sensor_channel','side_stabi','first_single'
%	=== OUTPUT RESULTS ======
%	SG:         SG of Manipulator
%   SGc:        SGTchain of Manipulator
function [SG] = SGmanipulator(CPL_out,tool_r,M_paras,varargin) 
single=0; sensor_channel=0; side_stabi = 0; 
for f=1:size(varargin,2)
      switch varargin{f}
          case 'single'
              single = 1;
              crimp = 0;
          case 'sensor_channel'
              sensor_channel = 1;
          case 'side_stabi'
              side_stabi = 1;
          case 'first_single'
              single = 2;
              crimp = 1;
      end   
end
%% Setting up variables
CPLs = {};
if ~iscell(CPL_out)
    CPL_out = {CPL_out};
end
while size(CPL_out,2)<size(M_paras,1)
    CPL_out{end+1} = CPL_out{end};
end
CPL_out = CPL_out';

for i=1:size(CPL_out,1)
    if sensor_channel
        CPL_in = PLcircle(tool_r);
        CPL_in = CPLbool('-',CPL_in,PLtrans(PLcircle(tool_r),[0 tool_r+1.75]));
        CPL_in = [CPL_in;NaN NaN;PLtrans(PLcircle(1.4),[0 tool_r])];
    else
        CPL_in = PLcircle(tool_r);
    end
    CPLs{end+1} = CPLbool('-',CPL_out{i},CPL_in);
end
hole_r = 0.9;                          % Radius of rope/pushrodtubes
%% Initializing arrays and variables
arm = {};
SG_elements = [];
SG_conns =[];
progress = 0;
total_progress = (2*size(M_paras,1))+2;
%% Finding Positions of holes and creating CPLs
[CPLs_holes,positions] = PLholeFinder(CPL_out,tool_r,M_paras(:,[1,4]),hole_r,single); updateProgress;
%%  Creating elements and connectors
SG_bottom = SGelements(CPLbool('-',CPLs{1},CPLs_holes{1}),M_paras(1,1),M_paras(1,4),'','','bottom_element'); updateProgress;
if ~single
SG_top = SGconnector(CPLs(size(M_paras,1)),CPLs_holes(end),positions(1,:),[M_paras(end,[1,4]);M_paras(end,[1,4])],hole_r,'','','end_cap','y'); updateProgress;
else
  SG_top = SGconnector(CPLs(size(M_paras,1)),CPLs_holes(end),positions(1,:),[M_paras(end,[1,4]);M_paras(end,[1,4])],hole_r,'','','end_cap','y','single'); updateProgress;  
end
for i=1:size(M_paras,1)
    CPL_curr = CPLbool('-',CPLs{i},CPLs_holes{i});
    if i==1
        SG_elements = [SG_elements SGelements(CPL_curr,M_paras(i,1),M_paras(i,4),1.2,4)];  updateProgress;
        if single
            SG_conns = [SG_conns SGconnector(CPLs(i:i+1),CPLs_holes(i:i+1),flip(positions(end-i:end-i+1,:)),M_paras(i:i+1,[1,4]),hole_r,1.2,1.0,'single','y')]; updateProgress;
        else
            SG_conns = [SG_conns SGconnector(CPLs(i:i+1),CPLs_holes(i:i+1),flip(positions(end-i:end-i+1,:)),M_paras(i:i+1,[1,4]),hole_r,1.2,1.0,'y')]; updateProgress;
        end
    elseif i==2
        SG_elements = [SG_elements SGelements(CPL_curr,M_paras(i,1),M_paras(i,4),1.0,4)];  updateProgress;
        if single == 2
            SG_conns = [SG_conns SGconnector(CPLs(i:i+1),CPLs_holes(i:i+1),flip(positions(end-i:end-i+1,:)),M_paras(i:i+1,[1,4]),hole_r,1.0,0.8,'single','y','crimp')]; updateProgress;
        elseif single == 1
            SG_conns = [SG_conns SGconnector(CPLs(i:i+1),CPLs_holes(i:i+1),flip(positions(end-i:end-i+1,:)),M_paras(i:i+1,[1,4]),hole_r,1.0,0.8,'single','y')]; updateProgress;
        else
            SG_conns = [SG_conns SGconnector(CPLs(i:i+1),CPLs_holes(i:i+1),flip(positions(end-i:end-i+1,:)),M_paras(i:i+1,[1,4]),hole_r,1.0,0.8,'y')]; updateProgress;
        end
    else
        SG_elements = [SG_elements SGelements(CPL_curr,M_paras(i,1),M_paras(i,4),0.8)];  updateProgress;
    end
end
%% Filling cell list with elements for a single arm
arm = [arm repelem({SG_elements(1)},floor(M_paras(1,2)/15))];
arm = [arm {SG_conns(1)}];
if side_stabi == 1
    arm = [arm repmat({SG_elements(2);SG_elements(3)},floor(M_paras(2,2)/30),1)'];
else
    arm = [arm repelem({SG_elements(2)},floor(M_paras(2,2)/15))];
end
arm = [arm {SG_conns(2)}];
if side_stabi == 1
    arm = [arm repelem({SG_elements(4)},floor(M_paras(3,2)/15))];
else
    arm = [arm repelem({SG_elements(3)},floor(M_paras(3,2)/15))];
end
arm = [{SG_bottom} arm {SG_top}];
%% Generating single arm with SGTchain()
phis = [0 repmat(-0.1,1,6) zeros(1,15)];
arm = SGTchain(arm,phis);
arm = SGcat(arm);
%% Adding base to arms in a cell list
base = SGmanipulatorbase([CPLs_holes{1};NaN NaN;CPL_in],CPL_out{1},3,positions(end,:),1,sensor_channel);
SG = [{base} {arm} {arm}];
%% Generating full manipulator with framechain
framechain = SGTframeChain(1:2,[1 'F1' 3 'B']);
SG = SGcat(SGcat(SGTchain(SG,[0 0 0],0,framechain)));
SGplot(SG);
    function updateProgress
        progress = progress+1;
        disp(floor((progress/total_progress)*100)+"%");
    end

end