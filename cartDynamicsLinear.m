function [A,B] = cartDynamicsLinear(m,M,L,g,b)
% Returns A and B matrices of linearized cart dynamics around reference
% point theta = pi for pendulum (position of cart is free variable)
A = [0 1 0 0; 0 -b/M (m*g)/M 0; 0 0 0 1; 0 -b/(M*L) -((m+M)*g)/(M*L) 0];
B = [0; 1/M; 0; 1/(M*L)];
end

