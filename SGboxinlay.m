function [SG] = SGboxinlay()


SG_base_mid = SGbox([30 54.5 5]);

SG_base_front = SGbox([80 20 10]);

SG_base_front = SGtransrelSG(SG_base_front,SG_base_mid,'behind','alignbottom');

SG_base_back = SGtransrelSG(SG_base_front,SG_base_mid,'infront');

SG_base_mid_holder = SGbox([42 20 5]);
SG_base_mid_holder = SGtransrelSG(SG_base_mid_holder,SG_base_back,'behind','aligntop');



SG = SGcat(SG_base_mid,SG_base_front,SG_base_back,SG_base_mid_holder);



end