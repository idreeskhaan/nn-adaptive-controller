
function out=ikine_eval(px,py,pz)

%input is x,y,z
%output is d,q1,q2,q3
%px= 0.2
%py= 0.2;
%pz= 0.4;

%intitialize output variables
q1=0;
q2=0;
q3=0;
d=0;
y= [d,q1,q2,q3];


%Define Link lengths
L1= 0.2; 
L2= 0.2; 
L3= 0.3;
dmin= 0;
dmax= 0.254; %stroke length: 10in


%solution to q1
q1= atan2(py,px);
c1= cos(q1);

%solution to q3
pz_bar= pz-L1;
c3= ((px/c1)^2+ (pz_bar)^2 - (L2^2+L3^2))/(2*L2*L3);

%if c3 is within range
if (abs(c3)<1)

    %display("within Range");
    d=0;
    s3= sqrt(1-c3^2);
    q3= atan2(s3,c3);
end

%if c3 is not within range
if (abs(c3>1))
    
    %display("NOT within Range");
    d=0;
    while (abs(c3>1))
       d= d + 0.01;
       if d>=dmax
           break
       end
       pz_bar= pz-L1-d;
       c3= ((px/c1)^2+ (pz_bar)^2 - (L2^2+L3^2))/(2*L2*L3);

    end

    s3= sqrt(1-c3^2);
    q3= atan2(s3,c3);
end

c3= cos(q3); s3= sin(q3);

%solution to q2
A= [L2+L3*c3, -L3*s3;
    L3*s3, L2+L3*c3];
A1= [px/c1, -L3*s3;
    pz-(L1+d), L2+L3*c3];
A2= [L2+L3*c3, px/c1;
    L3*s3, pz-(L1+d)];

c2= det(A1)/det(A);
s2= det(A2)/det(A);
q2= atan2(s2,c2);


y = [d,q1,q2,q3];

out=y;
