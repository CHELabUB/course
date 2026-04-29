clear; clc; close all;
%% Load car and track
load('F1CarData.mat')
Car = CarParameter;
load('CircuitOfAmerica.mat','Track')
%% Start point
s_start = 0;
pos0 = Track.center(s_start);
%% Load baseline path
load('RyanCostelloPath.mat','Path')
%% Controller gains
Kp_speed = 497;
Ki_speed = 0;
R_control = @(t,x,y,psi,sigma,gamma,z_v,s0,sf,sr,n0,nf,nr) ...
    Speed_control(t,x,y,psi,sigma,gamma,z_v, ...
    s0,sf,sr,n0,nf,nr,Path,Car,Kp_speed,Ki_speed);
Kp_steering = 3;
Ki_steering = 0;
K_lateral = 1;
gamma_dot_control = @(t,x,y,psi,sigma,gamma,z_gamma,s0,sf,sr,n0,nf,nr) ...
    Steering_control(t,x,y,psi,sigma,gamma,z_gamma, ...
    s0,sf,sr,n0,nf,nr,Path,Car,K_lateral,Kp_steering,Ki_steering);
%% Dynamics
car_dynamics = @(t,x,y,psi,sigma,gamma,z_v,z_gamma,s) ...
    car_RWD_with_control(t,x,y,psi,sigma,gamma, ...
    z_v,z_gamma,gamma_dot_control,R_control,Track,Car,s);
%% Simulation settings
Time = 350;
sim_step = 0.05;
Animation = 0;
Constraints_check_type = 0;
usize = 12;
%% Initial condition
v0 = sqrt(Car.R_max/Car.k);
gamma0 = 0;
% states: x, y, psi, sigma, gamma, z_v, z_gamma
x0 = [pos0(1);
      pos0(2);
      Track.ftheta(s_start);
      v0;
      gamma0;
      0;
      0];
sys = @(t,x,para) car_dynamics(t,x(1),x(2),x(3),x(4),x(5),x(6),x(7),para);
%% Run simulation
[t,y,u,TotalTime,Num_of_violation] = CarSimRealTime(sys,[0 Time],x0, ...
    s_start,sim_step,usize,Track,Car,Animation,Constraints_check_type);
%% Print result
fprintf('\nTotal Time = %.3f sec\n',TotalTime);
fprintf('Violations: off-track = %d, front force = %d, rear force = %d\n', ...
    Num_of_violation(1),Num_of_violation(2),Num_of_violation(3));
%% Save result
TeamName = "Optimal Ackermann";
save("Team5.mat",'t','y','u',"Num_of_violation","TotalTime","TeamName");
%% Plot results
plot_results;

