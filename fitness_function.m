function [J] = fitness_function(theta_ss_error,x_ss_error, s_log, u_log,...
    p1,p2,p3,p4, u_max)
% Takes in necessary parameters and weights (p) and returns fitness
% function evaluation
J = p1*abs(theta_ss_error)+ p2*abs(x_ss_error) + p3*max(abs(s_log(1,:)))...
    + p4*(norm(u_log));
if(max(abs(u_log)) > u_max)
    J = inf;
end
end

