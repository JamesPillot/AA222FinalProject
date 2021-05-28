function ds = cartDynamics(s,m,M,L,g,b,u)
% Computes cart dynamics for simulation, code for cart Dynamics (free of my
% own typos) provided by Steve Brunton J. Nathan Kutz in "Data Driven
% Science & Engineering, Machine Learning, Dynamical Systems, and Control"
Sx = sin(s(3));
Cx = cos(s(3));
D = m*L*L*(M+m*(1-Cx^2));

xdot = s(2);
xddot = (1/D)*(-m^2*L^2*g*Cx*Sx + m*L^2*(m*L*s(4)^2*Sx - b*s(2))) + m*L*L*(1/D)*u;
thetadot = s(4);
thetaddot = (1/D)*((m+M)*m*g*L*Sx - m*L*Cx*(m*L*s(4)^2*Sx - b*s(2))) - m*L*Cx*(1/D)*u;

ds = [xdot; xddot; thetadot; thetaddot];
end