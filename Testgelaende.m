clf;clear;
addpath('Essentials');

PLcontour = [linspace(-5,5,500);linspace(2,4,250) linspace(4,2,250)]';

CPL = PLcircle(5,50);
CPL = CPLbool('-',CPL,PLsquare(8,10));
CPL = CPLaddauxpoints2D(CPL,0.5);

SG = SGofCPLz(CPL,0.1);

n=size(SG.VL,1);

% PLup=[SG.VL(1:n/2,1) SG.VL(1:n/2,2)];

PLup=[SG.VL(n/2+1:end,1) SG.VL(n/2+1:end,2)];

VLprojection = PLtoVLprojection(PLup, PLcontour);

SG.VL = [SG.VL(1:n/2,1) SG.VL(1:n/2,2) SG.VL(1:n/2,3);VLprojection];

SGplot(SG);