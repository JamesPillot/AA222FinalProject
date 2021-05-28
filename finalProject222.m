%% Pendulum, Cart, Simulation Parameters
clear all;
close all;
% System
m = 1; % mass of pendulum (kg)
M = 5; % mass of cart (kg)
L = 2; % length of pendulum (meters)
g = -9.81; % m/s^2
b = 1; % cart damping

% Simulation
simtime = 0;
runtime = 30; % in seconds
delta_time = .01; % in seconds, 10ms intervals
index = 1; % for storing data

% Log data
N = length(0:delta_time:runtime);
s_log = zeros(4,N);
u_log = zeros(1,N);

% Control limit
u_max = 150;

% Fitness Function Weights
p1 = 1; % theta steady state error
p2 = 1; % x steady state error
p3 = 25; % non-minimum phase reponse
p4 = 1; % actuator effort


%% LQR - base with no GA
% Linearize system
[A, B] = cartDynamicsLinear(m,M,L,g,b);
% LQR for Optimal Gain K
Q = eye(4);
R = 1;
K = lqr(A,B,Q,R);

s0 = [2; 0; pi+.7; 0]; % initial condition
ref = [4; 0; pi; 0]; % reference position

index = 1;
% tic;
for simtime = 0:delta_time:runtime
    % Run Controller
    u = -K*(s0 - ref);
    u_log(index) = u;
    % Simulate system with input
    tspan = simtime:delta_time:simtime+delta_time;
    [t, s] = ode45(@(t,s) cartDynamics(s,m,M,L,g,b,u),tspan,s0);
    % Log simulated data
    curr_s = s(length(s),:)'; % grab state at last ode45 time interval
    s_log(:,index) = curr_s; 
    s0 = curr_s;
    index = index + 1;

end
% toc;

% Plotting
t = 0:delta_time:runtime;
onesvec = ones(1, length(t));
close all;
figure;
% Plot cart's x position
subplot(2, 1, 1);
hold on;
[dip, dip_time] = min(s_log(1,:));
plot(t, (s_log(1,:)),'Color','r');
plot(t, ref(1)*onesvec,'--','Color', 'b');
plot(t(dip_time), dip, 'd');
xlabel('Time (seconds)', 'Interpreter', 'latex', 'Fontsize', 14);
ylabel('Cart Position (meters)',  'Interpreter', 'latex', 'Fontsize', 10);
title('Cart Position over Time', 'Interpreter', 'latex', 'Fontsize', 12);
legend('$x$',  '$x_{goal}$', 'Dip = ' + string(dip), 'Interpreter', 'latex', 'Fontsize', 14, 'location', 'southeast');
hold off;

subplot(2, 1, 2);
hold on;
plot(t, (s_log(3,:)),'Color','black');
plot(t, ref(3)*onesvec, '--','Color','b');
xlabel('Time (seconds)', 'Interpreter', 'latex', 'Fontsize', 14);
ylabel('Pendulum Position (radians)',  'Interpreter', 'latex', 'Fontsize', 10);
title('Pendulum Position over Time', 'Interpreter', 'latex', 'Fontsize', 12);
legend('$\theta$','$\theta_{goal}$', 'Interpreter', 'latex', 'Fontsize', 14); 
hold off;

% Plot cart's control effort
figure;
hold on;
plot(t,u_log, 'b');
[max_val, max_time] = max(abs(u_log));
plot(max_time-1, -max_val, 'd');
xlabel('Time (seconds)', 'Interpreter', 'latex', 'Fontsize', 14);
ylabel('Control Input (Newtons)',  'Interpreter', 'latex', 'Fontsize', 14);
title('Control Input over Time', 'Interpreter', 'latex', 'Fontsize', 16);
legend('$u$','$u_{max} = $ ' + string(-max_val) + ' N', 'Interpreter', 'latex', 'Fontsize', 14, 'location', 'southeast'); 
hold off;

% Baseline performance
theta_ss_error = ref(3) - s_log(3,length(s_log));
x_ss_error = ref(1) - s_log(1,length(s_log));

J_LQR = fitness_function(theta_ss_error, x_ss_error, s_log, u_log, p1, p2, p3, p4, u_max);

%% LQR + GA
% Set seed for debugging
rng(1);
% Linearize system
[A, B] = cartDynamicsLinear(m,M,L,g,b);

% Iterations of GA
max_iters = 5;
% Initial Population
population_size = 40; 
% Highest and lowest possible values of individual gene
upper_bound = 10^2;
lower_bound = .01;
% Create initial chromosomes of (q11, q22, q33, q44, r) with uniform
% distribution
population = zeros(5,population_size);
for i = 1:1:population_size
    chromosome = lower_bound + (upper_bound - lower_bound).* rand(5,1);
    population(:,i) = chromosome;
end
% Fitness function evaluation, weights, and storage
Jvec = zeros(population_size,1);
% Best k chromosome limit for parent selection
parent_cutoff = 5; 
parents = zeros(5, 2, population_size);
children = zeros(5, population_size);
sigma = 3; % for gaussian mutation

for i = 1:1:max_iters
    i
    Jvec = zeros(population_size,1);
    for j = 1:1:population_size
        % LQR for Optimal Gain K
        Q = diag([population(1:4,j)]);
        R = population(5,j);
        K = lqr(A,B,Q,R);

        s0 = [2; 0; pi+.7; 0]; % initial condition
        ref = [4; 0; pi; 0]; % reference position

        index = 1;
        s_log = zeros(4,N);
        u_log = zeros(1,N);
        for simtime = 0:delta_time:runtime
            % Run Controller
            u = -K*(s0 - ref);
            u_log(index) = u;
            % Simulate system with input
            tspan = simtime:delta_time:simtime+delta_time;
            [t, s] = ode45(@(t,s) cartDynamics(s,m,M,L,g,b,u),tspan,s0);
            % Log simulated data
            curr_s = s(length(s),:)'; % grab state at last ode45 time interval
            s_log(:,index) = curr_s; 
            s0 = curr_s;
            index = index + 1;
        end
        % Evaluate fitness of chromosome and store
        theta_ss_error = ref(3) - s_log(3,length(s_log));
        x_ss_error = ref(1) - s_log(1,length(s_log));
        Jvec(j) = fitness_function(theta_ss_error, x_ss_error, s_log, u_log, p1, p2, p3, p4, u_max);
    end 
    if i == max_iters
        break
    end
    % Truncation
    parents = selection_truncation(Jvec, population, population_size, parent_cutoff, parents);
    % Crossover
    children = singlePointCrossover(parents, children, population_size);
    % Mutation
    population = gaussianMutation(children, sigma, population_size, lower_bound);
end % end of iteration
%% Plot result of best chromosome
[minval indice] = min(Jvec);
best = population(:,indice);

% Linearize system
[A, B] = cartDynamicsLinear(m,M,L,g,b);
% LQR for Optimal Gain K
Q = diag(best(1:4));
R = best(5);
K = lqr(A,B,Q,R);

s0 = [2; 0; pi+1.1; 0]; % initial condition
ref = [4; 0; pi; 0]; % reference position

index = 1;

s_log = zeros(4,N);
u_log = zeros(1,N);
for simtime = 0:delta_time:runtime
    % Run Controller
    u = -K*(s0 - ref);
    u_log(index) = u;
    % Simulate system with input
    tspan = simtime:delta_time:simtime+delta_time;
    [t, s] = ode45(@(t,s) cartDynamics(s,m,M,L,g,b,u),tspan,s0);
    % Log simulated data
    curr_s = s(length(s),:)'; % grab state at last ode45 time interval
    s_log(:,index) = curr_s; 
    s0 = curr_s;
    index = index + 1;
end

theta_ss_error = ref(3) - s_log(3,length(s_log));
x_ss_error = ref(1) - s_log(1,length(s_log));
Jnew = fitness_function(theta_ss_error,x_ss_error, s_log, u_log, p1,p2,p3,p4, u_max);

% Plotting
t = 0:delta_time:runtime;
onesvec = ones(1, length(t));
close all;
figure;
% Plot cart's x position
subplot(2, 1, 1);
hold on;
[dip, dip_time] = min(s_log(1,:));
plot(t, (s_log(1,:)),'Color','r');
plot(t, ref(1)*onesvec,'--','Color', 'b');
plot(t(dip_time), dip, 'd');
xlabel('Time (seconds)', 'Interpreter', 'latex', 'Fontsize', 14);
ylabel('Cart Position (meters)',  'Interpreter', 'latex', 'Fontsize', 10);
title('Cart Position over Time', 'Interpreter', 'latex', 'Fontsize', 12);
legend('$x$',  '$x_{goal}$', 'Dip = ' + string(dip), 'Interpreter', 'latex', 'Fontsize', 14, 'location', 'southeast');
hold off;

subplot(2, 1, 2);
hold on;
plot(t, (s_log(3,:)),'Color','black');
plot(t, ref(3)*onesvec, '--','Color','b');
xlabel('Time (seconds)', 'Interpreter', 'latex', 'Fontsize', 14);
ylabel('Pendulum Position (radians)',  'Interpreter', 'latex', 'Fontsize', 10);
title('Pendulum Position over Time', 'Interpreter', 'latex', 'Fontsize', 12);
legend('$\theta$','$\theta_{goal}$', 'Interpreter', 'latex', 'Fontsize', 14); 
hold off;

% Plot cart's control effort
figure;
hold on;
plot(t,u_log, 'b');
[max_val, max_time] = max(abs(u_log));
plot(max_time-1, -max_val, 'd');
xlabel('Time (seconds)', 'Interpreter', 'latex', 'Fontsize', 14);
ylabel('Control Input (Newtons)',  'Interpreter', 'latex', 'Fontsize', 14);
title('Control Input over Time', 'Interpreter', 'latex', 'Fontsize', 16);
legend('$u$','$u_{max} = $ ' + string(-max_val) + ' N', 'Interpreter', 'latex', 'Fontsize', 14, 'location', 'southeast'); 
hold off;
%% Full state feedback
% Linearize system
[A, B] = cartDynamicsLinear(m,M,L,g,b);
% LQR for Optimal Gain K
Q = eye(4);
R = 1;
poles_loc = [-1.1 -1.2 -1.5 -1];

K = place(A,B,poles_loc);

s0 = [2; 0; pi+1.1; 0]; % initial condition
ref = [4; 0; pi; 0]; % reference position

index = 1;
% tic;
for simtime = 0:delta_time:runtime
    % Run Controller
    u = -K*(s0 - ref);
    if(abs(u) > u_max)
        u = sign(u)*u_max;
    end
    u_log(index) = u;
    % Simulate system with input
    tspan = simtime:delta_time:simtime+delta_time;
    [t, s] = ode45(@(t,s) cartDynamics(s,m,M,L,g,b,u),tspan,s0);
    % Log simulated data
    curr_s = s(length(s),:)'; % grab state at last ode45 time interval
    s_log(:,index) = curr_s; 
    s0 = curr_s;
    index = index + 1;

end
% toc;
% Plotting
t = 0:delta_time:runtime;
onesvec = ones(1, length(t));
close all;
figure;
% Plot cart's x position
subplot(2, 1, 1);
hold on;
[dip, dip_time] = min(s_log(1,:));
plot(t, (s_log(1,:)),'Color','r');
plot(t, ref(1)*onesvec,'--','Color', 'b');
plot(t(dip_time), dip, 'd');
xlabel('Time (seconds)', 'Interpreter', 'latex', 'Fontsize', 14);
ylabel('Cart Position (meters)',  'Interpreter', 'latex', 'Fontsize', 10);
title('Cart Position over Time', 'Interpreter', 'latex', 'Fontsize', 12);
legend('$x$',  '$x_{goal}$', 'Dip = ' + string(dip), 'Interpreter', 'latex', 'Fontsize', 14, 'location', 'southeast');
hold off;

subplot(2, 1, 2);
hold on;
plot(t, (s_log(3,:)),'Color','black');
plot(t, ref(3)*onesvec, '--','Color','b');
xlabel('Time (seconds)', 'Interpreter', 'latex', 'Fontsize', 14);
ylabel('Pendulum Position (radians)',  'Interpreter', 'latex', 'Fontsize', 10);
title('Pendulum Position over Time', 'Interpreter', 'latex', 'Fontsize', 12);
legend('$\theta$','$\theta_{goal}$', 'Interpreter', 'latex', 'Fontsize', 14); 
hold off;

% Plot cart's control effort
figure;
hold on;
plot(t,u_log, 'b');
[max_val, max_time] = max(abs(u_log));
plot(max_time-1, -max_val, 'd');
xlabel('Time (seconds)', 'Interpreter', 'latex', 'Fontsize', 14);
ylabel('Control Input (Newtons)',  'Interpreter', 'latex', 'Fontsize', 14);
title('Control Input over Time', 'Interpreter', 'latex', 'Fontsize', 16);
legend('$u$','$u_{max} = $ ' + string(-max_val) + ' N', 'Interpreter', 'latex', 'Fontsize', 14, 'location', 'southeast'); 
hold off;

% Baseline performance
theta_ss_error = ref(3) - s_log(3,length(s_log));
x_ss_error = ref(1) - s_log(1,length(s_log));

J_FullState = fitness_function(theta_ss_error, x_ss_error, s_log, u_log, p1, p2, p3, p4, u_max);
