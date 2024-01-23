


clc;
close all;

%% Reference Traj

dt=0.01;
T= 2; %sim time
t= (0:dt:T)'; %time vector 
dth= (2*pi/(length(t)-1))';
theta= 0:dth:2*pi;

%radius and center of the circle
r= 0.1;
ox= 0.6;
oz= 0.4;

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
a= v^2/r; %normal accel (const. speed motion)
ax= -a*cos(theta);
az= -a*sin(theta);
ay= zeros(size(theta));

xd= [px,py,pz];
xd_dot= [vx,vy,vz];
xd_2dot= [ax,ay,az];

%% Robot Modelling

%import the robot
robot= loadrobot("universalUR5", DataFormat="column");
show(robot);


%% Controller 

%controller settings
lamda= 0.1;
Kv= 1*(ones(3,3));
w= zeros(3,1);



tau= M*xr_2dot + C*xr_dot + H - Kv*s + w; %control law


%forward Kinematics
q= [0,0,0,0,0,0]';
q_dot= [0,0,0,0,0,0]';

T0EE = getTransform(robot,q,'base_link','tool0')


%Jacob
J= geometricJacobian(robot,q,'tool0')


%Get Tau from controller
tau= [0, 0, 0, 0, 0, 0]';
fext= zeros(6,10); %end effector loading


%Forward Dynamics
q_2dot= forwardDynamics(robot,q,q_dot,tau,fext);








%% RTB Model

%robot= loadrobot("universalUR5");

% Load robot
%
% robot = loadrobot("universalUR5", "DataFormat", "column", "Gravity", [0 0 -9.81]);
% 
% % Set up simulation, starting from home position at zero velocity
% tspan = 0:0.01:1;
% initialState = [homeConfiguration(robot); zeros(6,1)];
% 
% % Define a reference state with a target position, zero
% % velocity, and zero acceleration
% targetState = [pi/4; pi/3; pi/2; -pi/3; pi/6; -pi/4; zeros(6,1); zeros(6,1)];

% Model the behavior with as a system under computed torque
% control with error dynamics defined by a moderately fast step
% response with 5% overshoot


% motionModel = jointSpaceMotionModel("RigidBodyTree", robot);
% updateErrorDynamicsFromStep(motionModel, .3, .05);
% 
% % Simulate the behavior over 1 second using ode45
% [t,robotState] = ode45(@(t,state)derivative(motionModel, state, targetState), tspan, initialState);
% 
% % Plot response
% figure
% plot(t, robotState(:, 1:motionModel.NumJoints));
% hold all;
% plot(t, targetState(1:motionModel.NumJoints)*ones(1,length(t)), "--");
% title("Joint Position (Solid) vs Reference (Dashed)");
% xlabel("Time (s)")
% ylabel("Position (rad)");


% %cross product
% 
% 
% syms c2 s2 d_dot q1_dot q2_dot L2;
% 
% A= [s2*q1_dot;c2*q1_dot;q2_dot];
% B= [L2/2;0;0];
% 
% C= cross(A,B)
% 
% 
% syms s23 c23 q3_dot L3;
% P= [s23*q1_dot;c23*q1_dot;q2_dot+q3_dot];
% Q= [L3;0;0];
% R= cross(P,Q)
% 
% 
% 
% 
% %Transf
% syms q1 q2 q3 L1 L2 L3 d;
% 
% c1= cos(q1); s1= sin(q1);
% c2= cos(q2); s2= sin(q2);
% c3= cos(q3); s3= sin(q3);
% 
% T01= [1 0 0 0;0 1 0 0;0 0 1 d;0 0 0 1];
% T12= [c1 -s1 0 0;s1 c1 0 0;0 0 1 L1;0 0 0 1];
% T23= [c2 -s2 0 0;0 0 -1 0;s2 c2 0 0;0 0 0 1];
% T34= [c3 -s3 0 L2;s3 c3 0 0;0 0 1 0;0 0 0 1];
% T45= [1 0 0 L3;0 1 0 0;0 0 1 0;0 0 0 1];
% 
% 
% T02= T01*T12;
% T03= T02*T23;
% T04= T03*T34;
% T05= T04*T45
% 
% 
% 
% 
% syms a b c L3;
% 
% cross([a;b;c], [L3;0;0])
