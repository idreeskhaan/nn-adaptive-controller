function y = M_cap(u)


q= [u(1);u(2);u(3);u(4)];

A1= rand(9,5); %Alpha1
A2= rand(5,4); %Alpha2

k=1; %kappa
g_M= 1; %gamma M

%sigmoid function
function out= sigm(x)
    out= 1./ (1 + exp(x));
end

function out= sigm_dot(x)
    out= exp(-x)./(exp(-x) + 1).^2;
end


%A1_dot= -k*g_M*abs(s)*A1 - g_M*(sigm(A2'*q) + sigm_dot(A2'*q)*A2'*q)*xr_2dot*s';



%compute M
layer1= sigm(A2*q); %1./ (1 + exp(A2*q));
M= A1*layer1;

%convert to mtx
M_mtx= [M(1), M(2), M(3);
    M(4), M(5), M(6);
    M(7), M(8), M(9)];


y = sigm(A2*q);

end
