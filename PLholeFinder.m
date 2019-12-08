%%  [CPLs,positions] = hole_finder(CPL_out,CPL_in,h_axis,hole_r,push_rod)
%	=== INPUT PARAMETERS ===
%	CPL_out:    CPL of outer contour of elements
%	CPL_in:     CPL of tool hole
%	h_axis:   	2xn Vector of DoF axis
%   hole_r:     Radius for actuator rope
%   push_rod:   1 = push rod;0 = double pull rope
%	=== OUTPUT RESULTS ======
%	CPLs:         SG of Manipulator
%	positions:    2xn vector of positions of holes
function [CPLs,positions] = PLholeFinder(CPL_out,CPL_in,h_axis,hole_r,push_rod)
%% Initializing
CPL_no_go_area = [];
CPLs = {};
CPL_holes = [];
positions = [];
h_axis = flip(h_axis);

for i=1:size(h_axis,1)
    %% calculating general values
    curr_axis = PLtransR(PLtrans([-100 0;+100 0],[0 h_axis(i,2)]),rot(deg2rad(h_axis(i,1))));
    curr_mid_point = PLtrans([0 h_axis(i,2)],rot(deg2rad(h_axis(i,1))));
    e_dir = curr_axis/norm(curr_axis);
    %% Generating CPL of area where points cant go because of axis constraints
    cross_points = PLcrossCPLLine2(curr_axis,CPL_in);
    cross_points = sortrows(cross_points);
    if ~isempty(cross_points)
        cross_points = cross_points + 2*e_dir;
        cross_points = [cross_points(1,:)+e_dir(1,:)*rot(pi/2);cross_points(1,:)+e_dir(2,:)*rot(pi/2);cross_points(2,:)+e_dir(2,:)*rot(pi/2);cross_points(2,:)+e_dir(1,:)*rot(pi/2)];
    else
        %von mitte aus
    end
    if isempty(CPL_no_go_area)
        CPL_no_go_area = cross_points;
    else
        CPL_no_go_area = CPLbool('OR',CPL_no_go_area,cross_points);
    end
    %% Generating CPL of points where holes currently can go
    CPL_o_in = CPLgrow(CPL_out,-0.7-hole_r);
    CPL_i_out = CPLgrow(CPL_in,+0.5+hole_r);
    CPL_limit = CPLbool('-',CPL_o_in,CPL_i_out);
    CPL_limit = CPLbool('-',CPL_limit,CPL_no_go_area);
    if ~isempty(CPL_holes)
        CPL_limit = CPLbool('-',CPL_limit,CPLgrow(CPL_holes,+0.5+hole_r));
    end
    %% Searching point with furthest distant to axis that still can be mirrored on the other side
    dis = 0;
    dis_pos=0;
    for j=1:size(CPL_limit,1)
        new_dis = distPointLine(curr_axis,CPL_limit(j,:));
        if dis<new_dis
            if push_rod == 0 && i~=size(h_axis,1)
                is_inside = insideCPS(CPL_limit,PLtransC(CPL_limit(j,:),curr_mid_point,pi));
                if is_inside >= 0
                    dis = new_dis;
                    dis_pos = j;
                end
            else
                dis = new_dis;
                dis_pos = j;
            end
        end
    end
    if push_rod == 0 && i~=size(h_axis,1)
        hole_positions = [CPL_limit(dis_pos,:);PLtransC(CPL_limit(dis_pos,:),curr_mid_point,pi)];
    else
        hole_positions = CPL_limit(dis_pos,:);
    end
    if isempty(CPL_holes)
        if push_rod == 0 && i~=size(h_axis,1)
            CPL_holes = [PLtrans(PLcircle(hole_r),hole_positions(1,:));NaN NaN;PLtrans(PLcircle(hole_r),hole_positions(2,:))];
        else
            CPL_holes = PLtrans(PLcircle(hole_r),hole_positions(1,:));
        end
    else
        if push_rod == 0 && i~=size(h_axis,1)
            CPL_holes = [CPL_holes;NaN NaN;PLtrans(PLcircle(hole_r),hole_positions(1,:));NaN NaN;PLtrans(PLcircle(hole_r),hole_positions(2,:))];
        else
           CPL_holes = [CPL_holes;NaN NaN;PLtrans(PLcircle(hole_r),hole_positions(1,:))];
        end
    end
    CPLs{end+1} = CPL_holes;
    positions = [positions;hole_positions(1,:)];
end
CPLs = flip(CPLs);
end
