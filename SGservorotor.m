%%  [SG] = SGservorotor(r,SG_connector,screw_h)
%	=== INPUT PARAMETERS ===
%	r:              Radius of Rotor 
%	SG_connector:   SG of Servoconnector
%   screw_h:        3x1 Vector [pitch_radius hole_number hole_radius]
%	=== OUTPUT RESULTS ======
%   SG:            	SG of rotordisk with servomount
function [SG] = SGservorotor(r,SG_connector,screw_h)
CPL_screw_circle= [];
if ~isempty(screw_h) 
    CPL_screw_circle = PLholePattern(screw_h(1),screw_h(2),screw_h(3)); 
    screw_head_s = screw_h(1)+screw_h(3)*2.5; %Platz für Schraubenkopf Mitte
else
    screw_head_s = 10;
end

r_mid = (r-screw_head_s)/2; % dicke Ropelayer 
r_mid_pos = screw_head_s+r_mid; % Positions mittelkreis
% base_r = min(50,max(27,r+2));
base_r = r+2;

if ~isempty(CPL_screw_circle)
    SG_base_layer = SGofCPLz([PLcircle(1.75);NaN NaN;PLcircle(base_r);NaN NaN;CPL_screw_circle],1.5);
else
    SG_base_layer = SGofCPLz([PLcircle(base_r)],1.5);
end
if ~isempty(SG_connector)
SG_base_layer = SGcat(SGunder(SG_connector,SG_base_layer),SG_base_layer);
end

f = 0.2;
phi_diff = 0.2;
startp = (2.5+phi_diff)*pi;
phi_closest = (-0.5+phi_diff)*pi;
endp = pi;

PL_rope_layer = [PLcircseg(r,'',startp,endp);flip(PLcircseg(screw_head_s,'',startp,endp))];

PL_rope_ramp_cutout = [-r_mid_pos+0.5 0;-r_mid_pos+1 r_mid;-r_mid_pos-1 r_mid;-r_mid_pos-0.5 0];
PL_rope_ramp_cutout = CPLbool('+',PL_rope_ramp_cutout,PLtrans(PLsquare(5,4),[-r_mid_pos -8]));
PL_half_circles = PLtrans(PLcircseg(r_mid,'',pi,0),[-screw_head_s-(r_mid) 0]);
PL_half_circles = CPLbool('-',PL_half_circles,PLtransR(PL_rope_ramp_cutout,rot(0)));
PL_half_circles = [PL_half_circles;NaN NaN;PLtransC(VLswapY(PL_half_circles),[0 0],phi_closest)];
 
PL_rope_layer = CPLbool('+',PL_rope_layer,PL_half_circles);
SG_rope_layer = SGofCPLz(PL_rope_layer,2);
SG = SGcat(SG_base_layer,SGontop(SG_rope_layer,SG_base_layer));

PL_rope_ramp = [0 0;r_mid 0;r_mid 3.5];
SG_rope_ramp = SGtrans(SGofCPLz(PL_rope_ramp,2),TofR(rotx(90)*roty(-90)));
SG_rope_ramp = SGtransrelSG(SG_rope_ramp,SG,'ontop',-2,'centerx',-r_mid_pos,'centery',r_mid/2);
SG_rope_ramp = SGcat(SGtransR(SGmirror(SG_rope_ramp,'xz'),rot(0,0,phi_closest)),SG_rope_ramp);

PL_top_layer = [PLcircle(screw_head_s);NaN NaN;PLcircle(r+2);NaN NaN;PL_rope_ramp_cutout;NaN NaN;PLtransC(VLswapY(PL_rope_ramp_cutout),[0 0],phi_closest)];
SG_top_layer = SGofCPLz(PL_top_layer,1.5);
SG = SGcat(SG,SGontop(SG_top_layer,SG),SG_rope_ramp);

PL_rope_guide = [r+2 0;r+7 0;r+7 5;r+2 5;r+2 3.5;r+5 3.5;r+5 1.5;r+2 1.5];
PL_rope_guide = PLroundcorners(PL_rope_guide,[2,3,6,7],1);
SG_rope_guide = SGcat(SGofCPLrota(PL_rope_guide,0.75,false,1.5),SGofCPLrota(PL_rope_guide,0.75,false,3.2));
SG_rope_guide = SGtransrelSG(SG_rope_guide,SG,'ontop',-5);

% PL_front_wall = [PLsquare(12,4);NaN NaN;PLcircle(0.4)];
% SG_front_wall = SGofCPLy(PL_front_wall,3);
% PL_back_wall = CPLbool('-',PLsquare(12,6),PLsquare(2,10));
% PL_back_wall = CPLbool('-',PL_back_wall,PLtrans(PLsquare(8,4),[0 1]));
% SG_back_wall = SGofCPLz(PL_back_wall,4);
% 
% SG_front_walls = SGtransrelSG(SG_front_wall,SG,'ontop','transx',-r_mid_pos,'transy',-3);
% SG_back_wall = SGtransrelSG(SG_back_wall,SG_front_walls,'aligntop','infront','alignright');
PL_walls = CPLbool('-',PLsquare(11,9),PLtrans(PLsquare(9,4),[0 -1]));
PL_walls = CPLbool('-',PL_walls,PLtrans(PLsquare(2,5),[0 -2]));
PL_walls = PLroundcorners(PL_walls,[1,2,3,4],[1,3,3,1]);
SG_walls = SGtrans(SGofCPLz(PL_walls,5),[0 -11/2 0]);
SG_walls = SGtransrelSG(SG_walls,SG,'ontop','transx',-r_mid_pos,'transy',-1.5);
SG_hole = SGtransrelSG(SGofCPLy(PLcircle(0.35),10),SG_walls,'aligntop',-3,'centerx','alignfront');
SG_walls = SGboolh('-',SG_walls,SG_hole);

SG_walls_2 = SGtransR(SGmirror(SG_walls,'xz'),rot(0,0,phi_closest));
SG_walls = SGcat(SG_walls_2,SG_walls);

PL_stamp = CPLbool('+',PLsquare(8.7,5),PLtrans(PLsquare(4.7,1.25),[0 -3.125]));
PL_stamp = PLroundcorners2(PL_stamp,[1,2,4,5,6,7],1);
SG_stamp = SGtrans(SGofCPLz(PL_stamp,4),[0 10 0]);
SG_stamps = SGcircularpattern(SG_stamp,4,20);
SG_stamps = SGtransrelSG(SG_stamps,SG,'ontop',-3);

SG = SGcat(SG,SG_rope_guide,SG_walls,SG_stamps);
end
