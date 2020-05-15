%%  [SG] = SGservorotor(r,SG_connector,screw_h)
%	=== INPUT PARAMETERS ===
%	r:              Radius of Rotor 
%	SG_connector:   SG of Servoconnector
%   screw_h:        3x1 Vector [pitch_radius hole_number hole_radius]
%	=== OUTPUT RESULTS ======
%   SG:            	SG of rotordisk with servomount
function [SG] = SGservorotor(rotor_radius,SG_connector,screw_h,varargin)
name = 0; if nargin>=4 && ~isempty(varargin{1}); name=varargin{1}; end
click = 0; if nargin>=5 && ~isempty(varargin{2}); click=varargin{2}; end
CPL_screw_circle= [];
if ~isempty(screw_h) 
    CPL_screw_circle = PLholePattern(screw_h(1),screw_h(2),screw_h(3)); 
    screw_head_s = screw_h(1)+screw_h(3)*2.5; %Platz für Schraubenkopf Mitte
else
    screw_head_s = 10;
end

PL_click = PLtransR(PLsquare(click+0.2),rot(pi/4));

SG_text = {SGoftext("SM40BL",[10 4 1]),SGoftext("SM85CL",[10 4 1]),SGoftext("SM120BL",[10 4 1])};


r_mid = (rotor_radius-screw_head_s)/2; % dicke Ropelayer 
r_mid_pos = screw_head_s+r_mid; % Positions mittelkreis
% base_r = min(50,max(27,r+2));
base_r = rotor_radius+2;
phi_diff = 0.2;
startp = (2.5+phi_diff)*pi;
phi_closest = (-0.5+phi_diff)*pi;
endp = pi;

if ~isempty(CPL_screw_circle)
    PL_base_layer = [PLcircle(1.75);NaN NaN;PLcircle(base_r);NaN NaN;CPL_screw_circle];    
    PL_base_layer = CPLbool('-',PL_base_layer,PLtrans(PLsquare(5,4),[-r_mid_pos -8]));
    PL_base_layer = CPLbool('-',PL_base_layer,PLtransC(VLswapY(PLtrans(PLsquare(5,4),[-r_mid_pos -8])),[0 0],phi_closest))
else
    PL_base_layer = PLcircle(base_r);
    PL_base_layer = CPLbool('-',PL_base_layer,PLtrans(PLsquare(5,4),[-r_mid_pos -8]));
    PL_base_layer = CPLbool('-',PL_base_layer,PLtransC(VLswapY(PLtrans(PLsquare(5,4),[-r_mid_pos -8])),[0 0],phi_closest));    
end
 PL_base_layer = CPLbool('-',PL_base_layer,PL_click);
SG_base_layer = SGofCPLz(PL_base_layer,1.5);
if ~isempty(SG_connector)
SG_base_layer = SGcat(SGunder(SG_connector,SG_base_layer),SG_base_layer);
end



PL_rope_layer = [PLcircseg(rotor_radius,'',startp,endp);flip(PLcircseg(screw_head_s,'',startp,endp))];


PL_rope_ramp_cutout = [-r_mid_pos+0.5 0;-r_mid_pos+1 r_mid;-r_mid_pos-1 r_mid;-r_mid_pos-0.5 0];
PL_rope_ramp_cutout = CPLbool('+',PL_rope_ramp_cutout,PLtrans(PLsquare(5,4),[-r_mid_pos -8]));
PL_half_circles = PLtrans(PLcircseg(r_mid,'',pi,0),[-screw_head_s-(r_mid) 0]);
PL_half_circles = CPLbool('-',PL_half_circles,PLtransR(PL_rope_ramp_cutout,rot(0)));
PL_half_circles = [PL_half_circles;NaN NaN;PLtransC(VLswapY(PL_half_circles),[0 0],phi_closest)];
 
PL_rope_layer = CPLbool('+',PL_rope_layer,PL_half_circles);
PL_rope_layer = CPLbool('+',PL_rope_layer,PLcircle(r_mid_pos-0.5));
PL_rope_layer = CPLbool('-',PL_rope_layer,CPL_screw_circle);
PL_rope_layer = CPLbool('-',PL_rope_layer,PLcircle(screw_h(3)));
PL_rope_layer = CPLbool('-',PL_rope_layer,PLtrans(PLsquare(5,4),[-r_mid_pos -8]));
PL_rope_layer = CPLbool('-',PL_rope_layer,PLtransC(VLswapY(PLtrans(PLsquare(5,4),[-r_mid_pos -8])),[0 0],phi_closest));

PL_rope_layer = CPLbool('-',PL_rope_layer,PL_click);
SG_rope_layer = SGofCPLz(PL_rope_layer,2);
SG = SGcat(SG_base_layer,SGontop(SG_rope_layer,SG_base_layer));

PL_rope_ramp = [0 0;r_mid 0;r_mid 3.5];
SG_rope_ramp = SGtrans(SGofCPLz(PL_rope_ramp,2),TofR(rotx(90)*roty(-90)));
SG_rope_ramp = SGtransrelSG(SG_rope_ramp,SG,'ontop',-2,'centerx',-r_mid_pos,'centery',r_mid/2);
SG_rope_ramp = SGcat(SGtransR(SGmirror(SG_rope_ramp,'xz'),rot(0,0,phi_closest)),SG_rope_ramp);

PL_top_layer = [PLcircle(screw_head_s);NaN NaN;PLcircle(rotor_radius+2);NaN NaN;PL_rope_ramp_cutout;NaN NaN;PLtransC(VLswapY(PL_rope_ramp_cutout),[0 0],phi_closest)];

PL_top_layer = CPLbool('+',PL_top_layer,PLcircle(screw_head_s));
PL_top_layer = CPLbool('-',PL_top_layer,PLholePattern(screw_h(1),screw_h(2),screw_h(3)+1));
PL_top_layer = CPLbool('-',PL_top_layer,PLcircle(screw_h(3)+1));
PL_top_layer = CPLbool('-',PL_top_layer,PL_click);
SG_top_layer = SGofCPLz(PL_top_layer,1.5);
SG = SGcat(SG,SGontop(SG_top_layer,SG),SG_rope_ramp);

PL_rope_guide = [rotor_radius+2 0;rotor_radius+7 0;rotor_radius+7 5;rotor_radius+2 5;rotor_radius+2 3.5;rotor_radius+5 3.5;rotor_radius+5 1.5;rotor_radius+2 1.5];
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
PL_walls = CPLbool('-',PLsquare(11,15),PLsquare(9,4));
PL_walls = CPLbool('-',PL_walls,PLtrans(PLsquare(2,10),[0 -6]));
PL_walls = PLroundcorners(PL_walls,[1,2,3,4],[1,3,3,1]);
SG_walls = SGtrans(SGofCPLz(PL_walls,5),[0 -6.5 0]);
SG_walls = SGtransrelSG(SG_walls,SG,'ontop','transx',-r_mid_pos,'transy',-1.5);
SG_hole = SGtransrelSG(SGofCPLy(PLcircle(0.35),30),SG_walls,'aligntop',-4,'centerx','alignfront');
SG_walls = SGboolh('-',SG_walls,SG_hole);

SG_walls_2 = SGtransR(SGmirror(SG_walls,'xz'),rot(0,0,phi_closest));
SG_walls = SGcat(SG_walls_2,SG_walls);

if name ~=0
    SG_tex = SGtransrelSG(SG_text{name},SG,'ontop','centerx','transy',-rotor_radius);
end
SG_tex_2 = SGoftext("R = " + rotor_radius+"MM",[10 4 1]);
 SG_tex_2= SGtransrelSG(SG_tex_2,SG,'ontop','centerx','transy',-rotor_radius,'rotz',pi/2);
SG_tex = SGcat(SG_tex,SG_tex_2);

PL_stamp = CPLbool('+',PLsquare(8.7,5),PLtrans(PLsquare(4.7,1.25),[0 -3.125]));
PL_stamp = PLroundcorners2(PL_stamp,[1,2,4,5,6,7],1);
SG_stamp = SGtrans(SGofCPLy(PL_stamp,4),[0 10 0]);
% SG_stamps = SGcircularpattern(SG_stamp,4,20);
% SG_stamps = SGtransrelSG(SG_stamps,SG,'ontop',-3);

SG = SGcat(SG,SG_rope_guide,SG_walls,SG_tex);
SG = SGtransR(SG,rotz(120));
end
