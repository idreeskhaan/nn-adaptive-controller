clc




s= tf('s');
G= (3*s+1)/((0.05*s+1)*(20*s+1));


bandwidth(G)

%% 
syms w;
eqn= 0.708^2 == (1+9*w^2)/((-w^2+1)^2+(20.05*w)^2)
vpa(solve(eqn,w), 4)

%% 
syms a;
eqn2= 0.708/a== 1/(sqrt(a^2+900))
vpa(solve(eqn2,a), 4)

%% Verify qn3

s= tf('s');
a= 30.08;
G= 1/(s+a);
bandwidth(G)


