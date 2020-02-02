function [ VLprojection ] = PLtoVLprojection(PL, PLcontour)
%PLtoVLprojection Projection of a 2D-PL (point list) towards a vertical 2D-contour. The result is a
%   3D-VL (vertex list) list of the projected PL (point list). 
%   PL: point list (2D; x-y-plane)
%   PLcontour: Contour to project on as PL. The contour will be transfered
%   into vertical plane (x-z-plane) for the projection.
%   
%   Example: 
%   PL=PLcircseg(3,100,0,2*pi);
%   PLcontour = PLcircseg(4,1000,0,pi);
%   PLcontour = [PLcontour(:,1) PLcontour(:,2)+1];
%   VLprojection = PLtoVLprojection(PL, PLcontour);
%   PLplot(PL); plot3(PLcontour(:,1),zeros(size(PLcontour,1),1), PLcontour(:,2),'b');VLplot(VLprojection);
% 



% %%%%%%EXAMPLE PARAMETERS
% phi=0;
% PL=PLcircseg(3,100,0,2*pi);
% % contourfunction = @(phi)sin(phi); 
% VLcontour = VLaddz(PLcircseg(10,50,0,pi),0);
% VLcontour =[VLcontour(:,1) VLcontour(:,3)+2 VLcontour(:,2)]; % VL has to be vertically to PL contour
% PLcontour = PLcircseg(4,1000,0,pi);
% PLcontour = [PLcontour(:,1) PLcontour(:,2)+1];
% %%%%%%

% PL=PLcircseg(3,100,0,2*pi);% 
% xPL= linspace(0,12,500);
% yPL= 5 - xPL*tand(20);
% xPLminus= -flip(xPL);
% yPLminus= flip(yPL);
% xPL = [xPLminus,xPL];
% yPL = [yPLminus,yPL];
% PLcontour = [xPL' yPL'];
% PLcontour = [PLcontour(:,1) PLcontour(:,2)+1];
% 
% direction = 1;
%%%%%%%%


%convert PL contour to VLcontour (contour in x-z-plane and y=0)
VLcontour = [PLcontour(:,1) zeros(size(PLcontour,1),1) PLcontour(:,2) ]; %create  VLcontour to plot it in vertical plane (x-z-plane)

%PL of contour, that shall be projected onto PLcontour, create VLprojection
VLprojection = [PL(:,1) PL(:,2) zeros(size(PL,1),1)]; %create VL

%find closest value in x for protection
for i = 1:size(VLprojection,1)
[~,idx]=min(abs(VLprojection(i,1)-VLcontour(:,1)));
Minidx(i)=idx; % gives index of the fittet fct x1 corresponding to the point x(i) 
% idx
VLprojection(i,3) = VLcontour(idx,3); %procetion by defining a z-value for the 2D-PL

end;

% FIND FUNCTION FOR CONTOUR
% % %find the edges of the given contour
% % xmin = min(PLcontour(:,1));
% % xmax = max(PLcontour(:,1));
% % 
% % if xmax >=0 xspace=(xmax-xmin); %find lenght 
% % elseif xmin<0 && xmax < 0 xspace=(abs(xmax)-abs(xmin)); 
% % end;
% % %fit curve
% % VLcontour = [PLcontour(:,1) zeros(size(PLcontour,1),1) PLcontour(:,2) ]; %create  VLcontour to plot it in vertical plane (x-z-plane)
% % x = VLcontour(:,1);
% % y = VLcontour(:,2);
% % z = VLcontour(:,3);
% % 
% % p = polyfit(x,z,20); % find a function (polynom grad 20 that defines the contour
% % x1 = linspace(xmin,xmax,xspace*100); % resolution 0.01mm
% % y1 = linspace(0,0,xspace*100); % define y-column with y=0
% % z1 = polyval(p,x1);
% % for i = 1:size(VLprojection,1)
% % [~,idx]=min(abs(VLprojection(i,1)-x1));
% % Minidx(i)=idx; % gives index of the fittet fct x1 corresponding to the point x(i) 
% % % idx
% % VLprojection(i,3) = z1(idx); %procetion by defining a z-value for the 2D-PL
% % 
% % end;


% hold on
% VLplot(VLcontour)
% plot3(x1,y1,z1,'b'); 
% 
% PLplot(PL,'r');
% VLplot(VLprojection,'g');
% plot3(PLcontour(:,1),zeros(size(PLcontour,1),1), PLcontour(:,2),'b')
% view(0,0) ;
% axis equal 
end

