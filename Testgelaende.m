clf;clear;
addpath('Essentials');

h_r = 1;

CPL1 = CPLconvexhull([PLcircle(h_r);NaN NaN;PLtrans(PLsquare(h_r*2),[0 -2])]);
CPL2 = CPLbool('-',PLcircle(h_r),PLtrans(PLcircle(h_r+0.1),[0 -0.4]));
% CPLplot(CPL1);
% CPLplot(CPL2,'g');
SG = SGof2CPLsz(CPL2,CPL1);
SGplot(SG);