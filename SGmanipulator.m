%%   [SG]=SGmanipulator(CPL,tool_r,M_paras)
%	=== INPUT PARAMETERS ===
%	CPL:            CPL of outer contour of elements
%   tool_r:         tool hole at [0 0] with radius tool_r
%	M_paras:        3xn Vector of DoFs [direction_angle total_angle offset]
%   push_rod:       1 = push_rod; 0 = double rope pull
%   sensor_channel: 1 = sensor channel, default: no sensor channel
%   side_stabi:     1 = sidestabi; default = no Sidestabi
%	=== OUTPUT RESULTS ======
%	SG:         SG of Manipulator
function [SG] = SGmanipulator(CPL_out,tool_r,M_paras,varargin)
push_rod=0;             if nargin>=4 && ~isempty(varargin{1}); push_rod=varargin{1}; end
sensor_channel=0;       if nargin>=5 && ~isempty(varargin{2}); sensor_channel=varargin{2}; end
side_stabi = 0;         if nargin>=6 && ~isempty(varargin{3}); side_stabi = varargin{3}; end

%% Setting up variables
if sensor_channel
    CPL_in = PLcircle(tool_r);
    CPL_in = CPLbool('-',CPL_in,PLtrans(PLcircle(tool_r),[0 tool_r+1.75]));
    CPL_in = [CPL_in;NaN NaN;PLtrans(PLcircle(1.4),[0 tool_r])];
else
    CPL_in = PLcircle(tool_r);
end
CPL = CPLbool('-',CPL_out,CPL_in);
hole_r = 0.9;                          % Radius of rope/pushrodtubes
%% Initializing arrays and variables
arm = {};
SG_elements = [];
SG_conns =[];
progress = 0;
total_progress = (2*size(M_paras,1))+2;
%% Finding Positions of holes and creating CPLs
[CPLs,positions] = PLholeFinder(CPL_out,CPL_in,M_paras(:,[1,3]),hole_r,push_rod); updateProgress;
%%  Creating elements and connectors
SG_bottom = SGelements(CPLbool('-',CPL,CPLs{1}),M_paras(1,1),M_paras(1,3),1); updateProgress;
SG_top = SGconnector(CPLbool('-',CPL,CPLs{end}),CPL,CPL_out,positions(1,:),[M_paras(end,[1,3]);M_paras(end,[1,3])],hole_r,1); updateProgress;
for i=1:size(M_paras,1)
    CPL_curr = CPLbool('-',CPL,CPLs{i});
    if i==1
        SG_elements = [SG_elements SGelements(CPL_curr,M_paras(i,1),M_paras(i,3),'',0,1.4-(0.2*i),4)];  updateProgress;
        if side_stabi == 1 
            SG_conns = [SG_conns SGconnector(CPL_curr,CPLbool('-',CPL,CPLs{i+1}),CPL_out,flip(positions(end-i:end-i+1,:)),M_paras(i:i+1,[1,3]),hole_r,'',1.4-(0.2*i),1.2-(0.2*i),1,2)]; updateProgress;
        else
            SG_conns = [SG_conns SGconnector(CPL_curr,CPLbool('-',CPL,CPLs{i+1}),CPL_out,flip(positions(end-i:end-i+1,:)),M_paras(i:i+1,[1,3]),hole_r,'',1.4-(0.2*i),1.2-(0.2*i),1,0)]; updateProgress;
        end
    elseif i==2
        SG_temp = SGelements(CPL_curr,M_paras(i,1),M_paras(i,3),'',side_stabi,1.4-(0.2*i),4);        
        SG_elements = [SG_elements SG_temp];  updateProgress;
        if side_stabi == 1
            SG_conns = [SG_conns SGconnector(CPL_curr,CPLbool('-',CPL,CPLs{i+1}),CPL_out,flip(positions(end-i:end-i+1,:)),M_paras(i:i+1,[1,3]),hole_r,'',1.4-(0.2*i),1.2-(0.2*i),0,1)]; updateProgress;
        else
            SG_conns = [SG_conns SGconnector(CPL_curr,CPLbool('-',CPL,CPLs{i+1}),CPL_out,flip(positions(end-i:end-i+1,:)),M_paras(i:i+1,[1,3]),hole_r,'',1.4-(0.2*i),1.2-(0.2*i),0,0)]; updateProgress;
        end
    else
        SG_elements = [SG_elements SGelements(CPL_curr,M_paras(i,1),M_paras(i,3),'',0,1.4-(0.2*i))];  updateProgress;
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
base = SGmanipulatorbase([CPLs{1};NaN NaN;CPL_in],CPL_out,3,positions(end,:),1,sensor_channel);
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