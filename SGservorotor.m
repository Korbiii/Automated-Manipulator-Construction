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

PL_crimp_holder_bot = PLcircle(5);
PL_crimp_holder_top = PLcircle(6);
PL_crimp_cutout = [-0.5 5.5;-0.5 2;-6.5 2;-6.5 1;-2 1;-2 -6;2 -6;2 1;8 1;8 2;0.5 2;0.5 5.5;0.5 5.5];
PL_crimp_holder_bot = CPLbool('-',PL_crimp_holder_bot,PL_crimp_cutout);
SG_crimp_holder_bot = SGofCPLz(PL_crimp_holder_bot,2.5);
PL_crimp_holder_top = CPLbool('-',PL_crimp_holder_top,[0.5 6;-0.5 6;-0.6 -6;0.5 -6]);
SG_crimp_holder_top = SGofCPLz(PL_crimp_holder_top,2.5);
SG_crimp_holder = SGcat(SGontop(SG_crimp_holder_top,SG_crimp_holder_bot),SG_crimp_holder_bot);
SG_crimp_holder = SGtransrelSG(SG_crimp_holder,SG,'ontop');
SG_crimp_holder = SGtrans(SG_crimp_holder,[-r_mid_pos -3 0]);
SG_crimp_holder_2 = SGmirror(SG_crimp_holder,'xz');
SG_crimp_holder_2= SGtransR(SG_crimp_holder_2,rot(0,0,phi_closest));


% PL_crimp_holder = [0.5 0;4 0;4 8;1.5 8;1.5 4;0.5 4];
% PL_crimp_holder = [PL_crimp_holder;NaN NaN;VLswapX(PL_crimp_holder)];
% SG_crimp_holder = SGofCPLz(PL_crimp_holder,2.5);
% PL_crimp_holder_top = [0.5 0;4 0;4 8;1.5 8;1.5 6;0.5 6];
% PL_crimp_holder_top = [PL_crimp_holder_top;NaN NaN;VLswapX(PL_crimp_holder_top)];
% SG_crimp_holder_top = SGofCPLz(PL_crimp_holder_top,2);
% SG_crimp_holder = SGcat(SGontop(SG_crimp_holder_top,SG_crimp_holder),SG_crimp_holder);
% SG_crimp_holder = SGtransrelSG(SG_crimp_holder,SG,'ontop');
% SG_crimp_holder = SGmirror(SG_crimp_holder,'xz');
% SG_crimp_holder = SGtrans(SG_crimp_holder,[-r_mid_pos 0 0]);
% SG_crimp_holder_2 = SGmirror(SG_crimp_holder,'xz');
% SG_crimp_holder_2= SGtransR(SG_crimp_holder_2,rot(0,0,phi_closest));
SG = SGcat(SG,SG_crimp_holder,SG_crimp_holder_2);

SG.FC = repmat([255 255 0],size(SG.VL,1),1);
end
