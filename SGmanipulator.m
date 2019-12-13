%%   [SG]=SGmanipulator(CPL,tool_r,M_paras)
%	=== INPUT PARAMETERS ===
%	CPL:        CPL of outer contour of elements
%   tool_r:     tool hole at [0 0] with radius tool_r
%	M_paras:   	3xn Vector of DoFs [direction_angle total_angle offset]
%   push_rod:   1 = push_rod; 0 = double rope pull
%	=== OUTPUT RESULTS ======
%	SG:         SG of Manipulator
function [SG] = SGmanipulator(CPL_out,tool_r,M_paras,push_rod)
%% Setting up variables
CPL_in = PLcircle(tool_r);
CPL = CPLbool('-',CPL_out,CPL_in);
hole_r = 0.9;                          % Radius of rope/pushrodtubes
%% Initializing arrays and variables
arm = {};
SG_elements = [];
writeSTLs = {};
SG_conns =[];
progress = 0;
total_progress = (2*size(M_paras,1))+2;
%% Finding Positions of holes and creating CPLs
[CPLs,positions] = PLholeFinder(CPL_out,CPL_in,M_paras(:,[1,3]),hole_r,push_rod); updateProgress;
%%  Creating elements and connectors
SG_bottom = SGelements(CPLbool('-',CPL,CPLs{1}),M_paras(1,1),M_paras(1,3),1); updateProgress;
SG_top = SGcolor(SGconnector(CPLbool('-',CPL,CPLs{end}),CPL_out,positions(1,:),[M_paras(end,[1,3]);M_paras(end,[1,3])],hole_r,1)); updateProgress;
for i=1:size(M_paras,1)
    CPL_curr = CPLbool('-',CPL,CPLs{i});
    if i==1
        SG_elements = [SG_elements SGelements(CPL_curr,M_paras(i,1),M_paras(i,3),'',0,1.4-(0.2*i),4)];  updateProgress;
        SG_conns = [SG_conns SGcolor(SGconnector(CPL_curr,CPL_out,flip(positions(end-i:end-i+1,:)),M_paras(i:i+1,[1,3]),hole_r,'',1.4-(0.2*i),1.2-(0.2*i),1,2))]; updateProgress;
    elseif i==2
        SG_temp = SGelements(CPL_curr,M_paras(i,1),M_paras(i,3),'',1,1.4-(0.2*i),4);
        SG_elements = [SG_elements SG_temp];  updateProgress;
        SG_conns = [SG_conns SGcolor(SGconnector(CPL_curr,CPL_out,flip(positions(end-i:end-i+1,:)),M_paras(i:i+1,[1,3]),hole_r,'',1.4-(0.2*i),1.2-(0.2*i),0,1))]; updateProgress;
    else
        SG_elements = [SG_elements SGelements(CPL_curr,M_paras(i,1),M_paras(i,3),'',0,1.4-(0.2*i))];  updateProgress;
    end
end
%% Filling cell list with elements for a single arm
% arm2 = [repmat({SG_elements(2);SG_elements(3)},floor(M_paras(2,2)/30),1)'];
% arm2 = SGTchain(arm2);
% SGwriteSTL(SGcat(arm2));
% SGwriteSTL(SG_elements(1),"1");
% SGwriteSTL(SG_elements(2),"2");
% SGwriteSTL(SG_elements(3),"3");
% SGwriteSTL(SG_conns(2),"4");
% SGwriteSTL(SG_conns(1),"5");
arm = [arm repelem({SG_elements(1)},floor(M_paras(1,2)/15))];
arm = [arm {SG_conns(1)}];
arm = [arm repmat({SG_elements(2);SG_elements(3)},floor(M_paras(2,2)/30),1)'];
% arm = [arm repelem({SG_elements(2)},floor(M_paras(2,2)/15))];
arm = [arm {SG_conns(2)}];
arm = [arm repelem({SG_elements(4)},floor(M_paras(3,2)/15))];
arm = [{SG_bottom} arm {SGcolor(SG_top)}];
%% Generating single arm with SGTchain()
phis = [0 repmat(-0.1,1,6) zeros(1,15)];
arm = SGTchain(arm,phis);
arm = SGcat(arm);
%% Adding base to arms in a cell list
base = SGmanipulatorbase([CPLs{1};NaN NaN;CPL_in],CPL_out,3,positions(end,:),1);
SG = [{base} {arm} {arm}];
%% Generating full manipulator with framechain
framechain = SGTframeChain(1:2,[1 'F1' 3 'B']);
SG = SGcolor(SGcat(SGcat(SGTchain(SG,[0 0 0],0,framechain))));
SGplot(SG);
% SGwriteMultipleSTL(writeSTLs);
    function updateProgress
        progress = progress+1;
        disp(floor((progress/total_progress)*100)+"%");
    end

end