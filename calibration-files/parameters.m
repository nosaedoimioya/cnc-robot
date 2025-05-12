function [wn,zeta] = parameters(J1)
load('base_damping_stiffness_params_15-Nov-2022.mat') 
load('shoulder_damping_stiffness_params_15-Nov-2022.mat')

wn     = [];
zeta   = [];

k       = k0_base+k1_base*J1;
c       = (c0_base+c1_base*sqrt(J1)+c2_base*J1)*(1+c_tau_base*J1);
wn(1)   = sqrt(k/J1);
zeta(1) = c/(2*sqrt(k*J1)); % 2*J1*wn = cc -> Critical damping


k       = k0_shoulder+k1_shoulder*J1;
c       = (c0_shoulder+c1_shoulder*sqrt(J1)+c2_shoulder*J1)*(1+c_tau_shoulder*J1);
wn(2)   = sqrt(k/J1);
zeta(2) = c/(2*sqrt(k*J1)); % 2*sqrt(k*J1) = cc -> Critical damping

end