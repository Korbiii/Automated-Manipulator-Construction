clear; clf;
addpath('Essentials');
SGs ={};
SG1 = SGbox(5);

SGs{end+1} = SG1;

SG2 = SGtrans(SGbox(6),[10 10 10]);

SGs{end+1} = SG2;

SG = SGcat(SGs);

SGplot(SG);