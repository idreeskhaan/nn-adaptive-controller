

%% Define Robot dynamic Params
clc;
close all;
clear;

%Load the Initial NN params
load("out.mat");

M= 5; %robot Base Mass
L1= 0.2; m1= 1; %Link 1 mass
L2= 0.2; m2= 0.5; %Link 2 mass
L3= 0.3; m3= 0.25; %Link 3 mass

B= 0.001; %damping coeff b/w links
b1= 0.001;
b2= 0.001;
b3= 0.001;


%% Reference Traj
dt=0.01;
T= 4; %sim time
t= (0:dt:T)'; %time vector 
dth= (2*pi/(length(t)-1))';
theta= 0:dth:2*pi;

%radius and center of the circle
r= 0.1;
ox= 0.1;
oz= 0.1;

%pos
py= zeros(size(t));
px= (ox + r*cos(theta))';
pz= (oz + r*sin(theta))';

%vel
v= 0.1; 
vx= -v*sin(theta);
vz= v*cos(theta);
vy= zeros(size(theta));

%accel
a= v^2/r;
ax= -a*cos(theta);
az= -a*sin(theta);
ay= zeros(size(theta));

xd= [px,py,pz];
xd_dot= [vx,vy,vz];
xd_2dot= [ax,ay,az];

figure();
plot(px,pz); grid on; xlabel("px [m]"); ylabel("pz [m]"); title("Desired Position of End Effector");
figure();
plot(vx,vz); grid on; xlabel("vx [ms^-^1]"); ylabel("vz [ms^-^1]"); title("Desired Velocity of End Effector");
figure();
plot(ax,az); grid on; xlabel("ax [ms^-^2]"); ylabel("az [ms^-^2]"); title("Desired Acceleration of End Effector");


%% Perform Ikine on Traj
for i=1:length(py)

ikine_eval(px(i),py(i),pz(i));

end

display("ikine Check successful. Trajectory is Valid!");


%% Control Law Params
lamda= 1.2;
Kv= [1.5, 0, 0;
    0, 0.1, 0;
    0, 0, 1.5];
w= zeros(3,1);



%% Initilize Neural Network Params

%Alphas
A1.time= [t];
A1.signals.values= [rand(9,5,length(t))];
A1.signals.dimensions= [9,5];

A2.time= [t];
A2.signals.values= [rand(5,4,length(t))];
A2.signals.dimensions= [5,4];

%Betas
B1.time= [t];
B1.signals.values= [rand(9,5,length(t))];
B1.signals.dimensions= [9,5];

B2.time= [t];
B2.signals.values= [rand(5,8,length(t))];
B2.signals.dimensions= [5,8];


%Gammas
C1.time= [t];
C1.signals.values= [rand(3,5,length(t))];
C1.signals.dimensions= [3,5];

C2.time= [t];
C2.signals.values= [rand(5,8,length(t))];
C2.signals.dimensions= [5,8];



%% Plot Actual and Desired Traj

sim dynamic_model_v4.slx;

actual_traj= ans.traj_actual;
px_actual= actual_traj(:,1,:);px_actual= px_actual(:);
pz_actual= actual_traj(:,3,:);pz_actual= pz_actual(:);

figure();
plot(px_actual,pz_actual);
hold on;
plot(px,pz);
grid on;
legend("Actual", "Desired");
xlabel("Px in [m]");
ylabel("Pz in [m]");
title("Desired vs Actual trajectory");


% Error plots
px_desired= ans.traj_desired(:,1);
pz_desired= ans.traj_desired(:,2);
ex= px_desired-px_actual;
ez= pz_desired-pz_actual;
tout= ans.tout;


figure();
plot(tout,ex);
hold on;
plot(tout,ez);
grid on;
xlabel("time [sec]");
ylabel("Error in [m]");
title("Tracking errors");
legend(["ex", "ez"]);


%Joint space plots
d= ans.q(:,1);
q1= ans.q(:,2);
q2= ans.q(:,3);
q3= ans.q(:,4);

figure();
plot(tout,d, tout, q1, tout, q2, tout, q3);
grid on;
xlabel("time [sec]");
ylabel("Joint Space Positions");
title("Joint Space Positions");
legend(["d", "q1", "q2", "q3"]);



%extract joint torques
tout= ans.tout;
tau= ans.tau;
tau= tau(:,:);

figure();
plot(tout, tau);
grid on;
xlabel("time");
ylabel("Joint Torques");
title("Joint Torques");
legend("F", "tau1", "tau2", "tau3");








