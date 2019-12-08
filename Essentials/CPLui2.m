%%   [S_coords]=CPLui2([snap_value,CPL_ref,SG_ref,phi_SG])
%	=== INPUT PARAMETERS ===
%	snap_value:	Value that clicked points snap to; 0 for no snapping
%	CPL_ref:	Reference CPL to build around
%	SG_ref:		Referemce SG to build around
%	phi_SG:     Angle to rotate SG
%   x_ref:      Referenceline in x-Direction
%   y_ref:      Referenceline in y_Direction
%   snap_deg    Degree to what lines snap to; default = 45°;-1 = free
%	=== OUTPUT RESULTS ======
%	S_coords:	String of coordinate
%   CPL:        CPL of contour
function [S_coords,CPL] = CPLui2(varargin)
snap_value=1;       if nargin>=1 && ~isempty(varargin{1}); snap_value=varargin{1}; end
CPL_ref='';         if nargin>=2 && ~isempty(varargin{2}); CPL_ref=varargin{2}; end
SG_ref='';          if nargin>=3 && ~isempty(varargin{3}); SG_ref=varargin{3}; end
phi_SG=[0 0 0];     if nargin>=4 && ~isempty(varargin{4}); phi_SG=varargin{4}; end
x_ref = {3 'r'};    if nargin>=5 && ~isempty(varargin{5}); x_ref=varargin{5}; end
y_ref = {};         if nargin>=5 && ~isempty(varargin{5}); y_ref=varargin{5}; end
% snap_angle = 45;     if nargin>=5 && ~isempty(varargin{6}); snap_angle=varargin{6}; end
CPL = [];
if ~isempty(SG_ref)
    SG_ref = SGtransR(SG_ref,rot(phi_SG(1),phi_SG(2),phi_SG(3)));
    SG_ref = SGtrans0(SG_ref);
    limits = SGsize(SG_ref)*1.5; %limit
    SG_ref.alpha = 0.25;
else
    limits = [-30 30 -10 10];
end
f = figure('Name','UI');
refresh
S_coords = "";
while true
    [p,in] = input();
    if in == 3 break; end;
    if in == 2 && (~isempty(x_ref) || ~isempty(y_ref))
       [~,pos] = min(abs(cell2mat(x_ref(:,1))-p(1)));
       S_coords = strsplit(strip(S_coords,'right',';'),';');
       S_coords = strcat(join(S_coords(1:end-1),';'),";");
       S_coords = strcat(S_coords,x_ref{pos,2},"+",num2str(CPL(end,1)-x_ref{pos,1})," ",num2str(CPL(end,2)),";");
    elseif in == 1
       S_coords = strcat(S_coords,num2str(p(1))," ",num2str(p(2)),";");
       CPL = [CPL;p(1) p(2)];
    elseif in == 114
        CPL(end,:) =[];
         S_coords = strsplit(strip(S_coords,'right',';'),';');
         S_coords = strcat(join(S_coords(1:end-1),';'),";");
    end
    refresh
end
S_coords = strcat("[",strip(S_coords,'right',';'),"]");

while true
    [p,in] = input();
    if in == 3 break; end;
    for m=1:size(CPL,1)
        if isequal(p,CPL(m,:)) break; end
    end
    CPL = circshift(CPL,(size(CPL,1)-m-1));
    points = CPL(end-2:end,:);
    points(:,3) =0;
    points = VLradialEdges(points);
    points(:,3) = [];
    CPL = [CPL(1:end-3,:);points];
    refresh
end

close(f);
    function refresh()
        clf;
        SGplot(SG_ref);
        CPLplot(CPL);
        CPLplot(CPL_ref,'y');
        if ~isempty(x_ref)
            for k=1:size(x_ref,1)
                PLplot([x_ref{k,1} -5000;x_ref{k,1} 5000],'g');
            end
        end
        if ~isempty(y_ref)
            for k=1:size(y_ref,1)
                PLplot([-5000 y_ref{k,1};5000 y_ref{k,1}],'g');
            end
        end
        view(2);grid on;
        xlim([limits(1) limits(2)]);
        ylim([limits(3) limits(4)]);
    end
    function [p,in] = input()
        p = [];

        [x,y,in]=ginput(1);
        if in == 3; return; end 
        snap_inv= 1/snap_value;
%         if ~isempty(CPL) 
%             b = [x;y]-CPL(end,:)';
%             clo_angle = acosd(dot([0;-1],b)/((norm([0;-1])*norm(b))));
%             clo_angle = round(clo_angle/snap_angle)*snap_angle;
%             x = round(snap_inv*x)/snap_inv;
%             if(clo_angle<90)
%                 y = CPL(end,2)-(x/tand(clo_angle));
%             elseif clo_angle>90
%                 y = CPL(end,2)+x*tand(clo_angle-90);
%             end
%         else
            x = round(snap_inv*x)/snap_inv;
            y = round(snap_inv*y)/snap_inv;
%         end             
        p = [x y];
    end
end