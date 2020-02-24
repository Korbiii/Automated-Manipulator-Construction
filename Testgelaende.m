clf;clear;
addpath('Essentials');


CPLplot(PLtrans(PLkidney(7,17,pi/6.5),[-12 0]),'g');

ova = PLtrans0(PLtransR(PLcircleoval(2.2,'',5),rot(pi/2)));
ova = (ova);
CPLplot(ova);








axis equal;