clf;
addpath('Essentials');



gap = 100;
start = 1000;
ende = size(total1,1)-gap;
% ende = 6000-gap;
total = ende-start;
c = 1;
for i=start:gap:ende
    colors = {[1 0 0],[1 0.5 0],[1 1 0],[0 1 0]};
    if i>0.25*total && i<0.5*total
        c= 2;
    elseif  i>0.5*total && i<0.75*total
        c= 3;
    elseif  i>0.75*total
        c=4;
    end
    
    
    plot3(total1(i:i+gap,1),total1(i:i+gap,2),total1(i:i+gap,3),'Color',colors{c});
hold on;
   
end