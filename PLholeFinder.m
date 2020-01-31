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
function [CPLs,positions] = PLholeFinder(CPL_out,tool_r,h_axis,hole_r,single)
%% Initializing
CPL_no_go_area = [];
CPLs = {};
CPL_holes = [];
positions = [];
h_axis = flip(h_axis);
CPL_out = flip(CPL_out);
CPL_in = PLcircle(tool_r);
%% Generating CPL of area where no holes can go based on axis constraints
CPL_axis_constraint = [tool_r+1.5 0.6;tool_r+1.5 -0.6;-tool_r-1.5 -0.6;-tool_r-1.5 0.6];
for i=1:size(h_axis,1)
    CPL_no_go_area = CPLbool('+',CPL_no_go_area,PLtransR(CPL_axis_constraint,rot(deg2rad(h_axis(i,1)))));
end
max_dim = 2*max(sizeVL(CPL_out{size(h_axis,1)}));

%%Looping over each section
for i=1:size(h_axis,1)
    %% calculating general values
    CPL_opti_area = [];
    curr_axis = PLtransR([-100 0;+100 0],rot(deg2rad(h_axis(i,1))));
    curr_mid_point = [0 0];
    e_dir = curr_axis/norm(curr_axis); 
    %% Generating CPL half for hinge optimization
    if h_axis(i,2) ~= 0
        CPL_opti_area = [max_dim 0;-max_dim 0;-max_dim max_dim;max_dim max_dim];
        CPL_opti_area = PLtransC(CPL_opti_area,[0 0],rot(deg2rad(h_axis(i,1))));
        if h_axis(i,2) == 1
            CPL_opti_area = PLtransC(CPL_opti_area,[0 0],pi);
        end
    end
    %% Generating CPL of points where holes currently can go
    CPL_o_in = CPLgrow(CPL_out{i},-0.7-hole_r);
    [~,pos]=separateNaN(CPL_in);
    if size(pos,1) > 2
        CPL_1 = CPLgrow(CPL_in(1:pos(2)-1,:),-0.5-hole_r);        
        CPL_2 = CPLgrow(CPL_in(pos(2)+1:end,:),+0.5+hole_r);
        CPL_i_out = CPLbool('+',CPL_1,CPL_2);
    else
        CPL_i_out = CPLgrow(CPL_in,+0.5+hole_r);
    end
    CPL_limit = CPLbool('-',CPL_o_in,CPL_i_out);
    CPL_limit = CPLbool('-',CPL_limit,CPL_no_go_area);
    CPL_limit = CPLbool('-',CPL_limit,CPL_opti_area);
    if ~isempty(CPL_holes)
        CPL_limit = CPLbool('-',CPL_limit,CPLgrow(CPL_holes,+0.5+hole_r));
    end
    %% Searching point with furthest distant to axis that still can be mirrored to the other side
    dis = 0;
    dis_pos=0;
    for j=1:size(CPL_limit,1)
        new_dis = distPointLine(curr_axis,CPL_limit(j,:));
        if dis<new_dis
            if single == 1 || (single == 2 && i == size(h_axis,1))
                dis = new_dis;
                dis_pos = j;
            else
                is_inside = insideCPS(CPL_limit,PLtransC(CPL_limit(j,:),curr_mid_point,pi));
                if is_inside >= 0
                    dis = new_dis;
                    dis_pos = j;
                end
            end
        end
    end
    if single == 1 || (single == 2 && i == size(h_axis,1))
        hole_positions = CPL_limit(dis_pos,:);
    else
        hole_positions = [CPL_limit(dis_pos,:);PLtransC(CPL_limit(dis_pos,:),curr_mid_point,pi)];
    end       
    
    if isempty(CPL_holes)
        if single == 1 || (single == 2 && i == size(h_axis,1))
            CPL_holes = PLtrans(PLcircle(hole_r),hole_positions(1,:));
        else
            CPL_holes = [PLtrans(PLcircle(hole_r),hole_positions(1,:));NaN NaN;PLtrans(PLcircle(hole_r),hole_positions(2,:))];
        end
    else
        if single == 1 || (single == 2 && i == size(h_axis,1))
           CPL_holes = [CPL_holes;NaN NaN;PLtrans(PLcircle(hole_r),hole_positions(1,:))];
        else
            CPL_holes = [CPL_holes;NaN NaN;PLtrans(PLcircle(hole_r),hole_positions(1,:));NaN NaN;PLtrans(PLcircle(hole_r),hole_positions(2,:))];
        end
    end
    CPLs{end+1} = CPL_holes;
    positions = [positions;hole_positions(1,:)];
end
CPLs = flip(CPLs);
end
