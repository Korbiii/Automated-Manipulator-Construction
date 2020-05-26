%%  [CPLs,positions] = hole_finder(CPL_out,CPL_in,h_axis,hole_r,push_rod)
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
        
        
        if isempty(hole_position) error("CPL f�r Sektion " + i + " zu klein"); end
        
        %% Adding Points
        if single == 1 || (single == 2 && first_section)
            hole_positions = hole_position;
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
        CPL_hole_positions_temp = [CPL_hole_positions_temp;hole_positions(1,:)];
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
