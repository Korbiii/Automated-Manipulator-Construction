clear; clf;
addpath('Essentials');

CPLplot(PLtrans(PLkidney(7,16,pi/5.5),[-11.5 0]));
CPLplot(PLcircle(3.25));

CPLplot(PLtrans(PLkidney(7,17,pi/6.5),[-12 0]),'g');

SGplot(SGbox(3));