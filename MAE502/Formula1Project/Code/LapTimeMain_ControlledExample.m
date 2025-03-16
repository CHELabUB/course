%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAE 502 Vehicle Control Systems Project 
% Formula 1 car racing: subfunctions
%   Copyright (C)2025 Chaozhe He. All Rights Reserved.
%   Author: Prof. Chaozhe He
%           Department of Mechanical and Aerospace Engineering
%           SUNY University at Buffalo
%           March 2025
% Any issues/bug reports,
% please email to chaozheh@buffalo.edu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main script with a controller that drives the car to finish the race
clear;clc;close all;
%% Car information
load('F1CarData.mat')
Car=CarParameter;
%% Track Information
load('CircuitOfAmerica.mat', "Track");
addpath("Utilities");
%% Start point set
s_start=0; % start from the race start and end at the same location after one lap

%%%% If you want to start and end somewhere else in the middle  %%%%%%%%%%%
%%%% (for testing purpose) uncommon and edit the following %%%%%%%%%%%%%%%%
% !!!!!note!!!!! that start_index = 11 !!!! is the start point (s = 0)
% s = 0 corresponds to 11th point in the track, and 587 is the same as 11th point
% start_index=11; % 1 to 597
% end_index=587; % 1 to 597
% % start at a certain location 
% s_start=Track.arc_s(start_index);
% Track.bstl=Track.bl(:,start_index);
% Track.bstr=Track.br(:,start_index);
% Track.bfl=Track.bl(:,end_index);
% Track.bfr=Track.br(:,end_index);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
figure(100)
hold on;axis equal;box on
plot3(Track.bl(1,:),Track.bl(2,:),Track.bl(3,:),'k','Markersize',5);
plot3(Track.br(1,:),Track.br(2,:),Track.br(3,:),'k','Markersize',5);
plot3(Track.cline(1,:),Track.cline(2,:),Track.cline(3,:),'k--','Markersize',5);
for i=1:length(Track.bl(1,:))
    if mod(i-1,20)==0
        plot3([Track.bl(1,i) Track.br(1,i)],[Track.bl(2,i) Track.br(2,i)],[Track.bl(3,i) Track.br(3,i)],'b','LineWidth',1)
    end

end


%%%%%%%%% initial condition %%%%%%%%%%%%%%%
pos0=Track.center(s_start);
v0=sqrt(Car.R_max/Car.k); % initial longitudinal speed
gamma0=0; % initial steering angle
x0=[pos0(1:2);Track.ftheta(s_start);v0; gamma0];


%% Load path
load("sample_path.mat", "Path");
figure(100);
X_path = Path.X(Track.arc_s);
V_ref_path = Path.v(Track.arc_s);
plot3(X_path(1, :), X_path(2,:), V_ref_path, 'g', "LineWidth", 2);
view(3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% determine feedback system
% 1. speed control
Kp_speed=1000; Ki_speed=0;
R_control = @(t, x, y, psi, sigma, gamma, z_v, s0, sf, sr, n0, nf, nr) ...
                        Speed_control(t, x, y, psi, sigma, gamma, z_v, ...
                                        s0, sf, sr, n0, nf, nr, ...
                                        Path, Car, Kp_speed, Ki_speed);
% 2. steering control
Kp_steering= 3; Ki_steering=0;
K_lateral = 1; % lateral feedback gain
gamma_dot_control = @(t, x, y, psi, sigma, gamma, z_gamma, s0, sf, sr, n0, nf, nr) ...
                        Steering_control(t, x, y, psi, sigma, gamma,...
                                           z_gamma,...
                                           s0,sf,sr,n0,nf,nr,...
                                           Path, Car,...
                                           K_lateral, Kp_steering, Ki_steering);
%% Dynamics
car_dynamics = @(t, x, y, psi, sigma, gamma, z_v, z_gamma, s) ...
                        car_RWD_with_control(t, x, y, psi, sigma, gamma, ...
                                            z_v, z_gamma, ...
                                            gamma_dot_control, R_control, ...
                                            Track, Car, s);
%%%  By default, the car RWD will give out dX and 10 values for monitoring purpose
%%% They are [R;gamma_dot; tgamma; dtgamma; s0;sf;sr;n0;nf;nr]; at time t
% R driving force. 
% gamma_dot time derivative of steering angle
% tan(gamma), d(tan(gamma))/dt,
% s0 reference to the centerline of the center of mass,
% sf reference to the centerline of the front axle 
% sr reference to the centerline of the rear axle
% n0 distance to the centerline from the center of mass,
% nf distance to the centerline from the front axle 
% nr distance to the centerline from the rear axle
% IF you define more value to be monitored in car_RWD, you need to change
% this number accordingly
% for this example I added u(11) desired v, u(12) desired gamma
usize=12;


%%
% Animation=1; % Animation on
% Better for fast and complete simulation
Animation=0; % Animation off
% = 0 only issue warning and let the simulation continue, facilitate tunning
% = 1 will stop the simulation when any of the constraints is violated (competition way)
Constraints_check_type = 0; 


%%
Time=400;
sim_step=0.05; % do not change this as I will using this for the competition

v0=sqrt(Car.R_max/Car.k); % initial longitudinal speed
gamma0=0; % initial steering angle
x0=[pos0(1:2);Track.ftheta(s_start);v0; gamma0; 0; 0]; % add z_v and z_gamma to the initial condition


sys=@(t, x, para) car_dynamics(t, x(1), x(2), x(3), x(4), x(5), x(6), x(7), para);
%% Run

[t,y,u, total_time, Num_of_violation] = CarSimRealTime(sys, [0 Time], x0, ...
                                           s_start, sim_step, usize, ...
                                           Track,Car,Animation, ...
                                           Constraints_check_type); 
if any(Num_of_violation ~= 0)
    fprintf("Num_of_violation: off track %d, front force %d, rear force %d", ...
             Num_of_violation(1), Num_of_violation(2), Num_of_violation(3));
else
    fprintf("Finished without violation.\n");
end

save("Sample_path_run.mat", 't','y','u', "Num_of_violation");
%% Lateral Forces
[Ffl_ana,Frl_ana]=Force_rwd(y(:,4),u(:,1),u(:,2),u(:,3),Car.m,Car.m0,Car.b,Car.w);
gamma_dot_limit = t * 0;
for i = 1:length(t)
    gamma_dot_limit(i) = gamma_dot_limit_by_Force(y(i, 4), y(i, 5), Car);
end
plot_results;
