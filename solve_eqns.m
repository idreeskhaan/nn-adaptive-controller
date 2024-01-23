

%% Solve for zeta wn
clear;
clc;

syms zeta wn;

eqn1= wn^4/((wn^2-9)^2 + (6*zeta*wn)^2) == 0.501;
eqn2= 3*pi/5 == atan2((6*pi*zeta*wn), (wn^2 - 9*pi^2));


anss= vpasolve([eqn1,eqn2]);

anss.wn
anss.zeta


%% Bode Plot
clear;
clc;

wn= 6.347; 
zeta= 1.25;

num= wn^2;
den= [1 2*zeta*wn wn^2];
sys= tf(num,den);

W= 0.001:0.001:1000;
bode(sys, W);
