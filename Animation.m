clf;
addpath('Essentials');

fps =36;

SG_background = SGreadSTL("C:\Users\Korbi\Desktop\Aufbau.stl");
SG_background = SGtransR(SG_background,rot(-pi/4,0,pi/4));
SG_background = SGtrans(SG_background,[120 50 -500]);
SGplot(SG_background);
writerObj = VideoWriter('test.avi');
writerObj.FrameRate = 24;
writerObj.Quality = 100;
open(writerObj);
phi_zero(:) = phi;
phi_zero(:) = 0;
viewSG(SGc,phi_zero,fc);
writeVideo(writerObj, getframe(gcf));
clf;

max_phi = phi(68)*0.7;
max_phi_2 = phi(79)*0.75;
stepsize = max_phi/fps;
stepsize_2_c = max_phi_2/fps;
for i=0:fps
    phi_zero(68:79) = phi_zero(68:79)+stepsize;    
    if i>0.1*fps
        phi_zero(79:2:end) = phi_zero(79:2:end)+stepsize_2_c;
    end
    
    SGplot(SG_background);
    viewSG(SGc,phi_zero,fc);
     writeVideo(writerObj, getframe(gcf));
     clf;
end
dir = 1;
for i=0:fps
    if i>0.25*fps dir = -1; end    
    if i>0.75*fps dir = 1; end
   phi_zero(80:2:end) = phi_zero(80:2:end)+(stepsize_2_c*2*dir);
    
    SGplot(SG_background);
    viewSG(SGc,phi_zero,fc);
     writeVideo(writerObj, getframe(gcf));
     clf;
end
% max_phi = phi(79);
% stepsize = max_phi/fps;
% for i=0:fps/2
%     phi_zero(79:end) = phi_zero(79:end)+stepsize;
%     viewSG(SGc,phi_zero,fc);
%      writeVideo(writerObj, getframe(gcf));
%      clf;
% end

max_phi = phi(2)*0.7;
stepsize = max_phi/fps;
max_phi_2 =  phi(15)*0.85;
stepsize_2 = max_phi_2/fps;
max_phi_3 =  phi(8);
stepsize_3 = max_phi_3/fps;
% dir = -1;
for i=0:fps
    phi_zero(2:7) = phi_zero(2:7)+stepsize;    
    phi_zero(35:40) = phi_zero(35:40)+stepsize;
%     if i > 0.5*fps dir = 2; end
%      phi_zero(80:2:end) = phi_zero(80:2:end)+(stepsize_2_c*dir);
    
    if i>0.3*fps
        phi_zero(15:33) = phi_zero(15:33)+stepsize_2;    
        phi_zero(48:66) = phi_zero(48:66)+stepsize_2;
    end
    

    
    SGplot(SG_background);
    viewSG(SGc,phi_zero,fc);
     writeVideo(writerObj, getframe(gcf));
     clf;
end
dir = 1;
for i=0:2*fps
    if i>0.5*fps dir = -1.3; end
    if i>1.5*fps dir = 1.3; end
    phi_zero(8:14) = phi_zero(8:14)+stepsize_3*dir;
    phi_zero(41:47) =phi_zero(41:47)+stepsize_3*dir;
       
    
    SGplot(SG_background);
    viewSG(SGc,phi_zero,fc);
     writeVideo(writerObj, getframe(gcf));
     clf;
end
dir = -1;
for i=0:fps
    if i>0.5*fps dir = 1; end
   phi_zero(15:33) = phi_zero(15:33)+stepsize_2*2*dir;         
    
    SGplot(SG_background);
    viewSG(SGc,phi_zero,fc);
     writeVideo(writerObj, getframe(gcf));
     clf;
end
dir = 1;
for i=0:fps
    if i>0.5*fps dir = -1; end
    phi_zero(15:33) = phi_zero(15:33)+stepsize_2*dir;    
    phi_zero(48:66) = phi_zero(48:66)+stepsize_2*dir;
    
    phi_zero(8:14) = phi_zero(8:14)+stepsize_3;
    phi_zero(41:47) =phi_zero(41:47)+stepsize_3;
       
    
    SGplot(SG_background);
    viewSG(SGc,phi_zero,fc);
     writeVideo(writerObj, getframe(gcf));
     clf;
end

% 

% 
% max_phi = phi(15);
% stepsize = max_phi/fps;
% for i=0:floor(fps*0.7)
%     phi_zero(15:33) = phi_zero(15:33)+stepsize;     
%     phi_zero(48:66) = phi_zero(48:66)+stepsize;
%     viewSG(SGc,phi_zero,fc);
%      writeVideo(writerObj, getframe(gcf));
%      clf;
% end



close(writerObj);



% for i=0:fps
%    phi(2:end) = phi(2:end)-(stepsize);
%   viewSG(SG,phi);
%     F = getframe(gcf);
%     writeVideo(writerObj, F);
%    clf;
%     
%     
% end
% close(writerObj);

function viewSG(SGc,phi,fc)
   SG = SGTchain(SGc,[0 phi],'',fc);
   SG = SGtransR(SG,rot(pi/4,pi/4,0));
  SGplot(SG);
   VLFLplotlight(1);
   grid off;
   
   axis([-450 330 -250 300 -800 800]);
%    axis([-50 330 -250 20 -800 800]);
% axis([-50 200 -400 0 -100 100]);
set(gcf,'color','w');
set(gca,'visible','off');
% fig.Positions = [0 0 4096 2160];
% figure('units','pixels','position',[0 0 4096 0])
% axis([-400 500 -400 400 -100 100]);
end


