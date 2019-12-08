%%  [PL] = PLcircarc2(startp,endp,radius)
%	=== INPUT PARAMETERS ===

%	=== OUTPUT RESULTS ======
function [PL] = PLcircarc2(startp,endp,radius)
clf;
min_radius = pdist2(startp,endp)/2;
if radius < min_radius,radius = min_radius; end
e_dir = [endp-startp]*rot(pi/2);
e_dir = e_dir/norm(e_dir);

PL_circle = PLcircle(radius,60);

dis = pdist2(startp,endp);
dis_mid = sqrt((radius^2)-(dis/2)^2);
mid_x = (startp(1)+endp(1))/2;
mid_y = (startp(2)+endp(2))/2;
mid_point = [mid_x mid_y];
circ_mid_point = mid_point - dis_mid*e_dir;

PL_circle = PLtrans(PL_circle,circ_mid_point);
PL_box = [startp(1) startp(2);startp(1) endp(2);endp(1) endp(2)];

PL = CPLbool('x',PL_circle,PL_box);
while round(PL(end,:)*100)/100 ~= round(startp*100)/100
    PL = circshift(PL,1);
end


% PLplot(PL_circle);
PLplot(startp,'p');
PLplot(endp,'p');
end