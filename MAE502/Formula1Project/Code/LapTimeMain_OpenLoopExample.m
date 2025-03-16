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
% Main script to submit your results
clear;clc;close all;
%% Car information
load('F1CarData.mat')
Car=CarParameter;
%% Track Information
load('CircuitOfAmerica.mat');
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
figure(1)
hold on;axis equal;box on
plot3(Track.bl(1,:),Track.bl(2,:),Track.bl(3,:),'k','Markersize',5);
plot3(Track.br(1,:),Track.br(2,:),Track.br(3,:),'k','Markersize',5);
plot3(Track.cline(1,:),Track.cline(2,:),Track.cline(3,:),'k--','Markersize',5);
for i=1:length(Track.bl(1,:))
    if mod(i-1,20)==0
        plot3([Track.bl(1,i) Track.br(1,i)],...
              [Track.bl(2,i) Track.br(2,i)],...
              [Track.bl(3,i) Track.br(3,i)],'b','LineWidth',1)
    end

end

%%%%%%%%% initial condition %%%%%%%%%%%%%%%
% initial positions should be always at the center.
pos0=Track.center(s_start);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TODOs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% TODO: Present your control design here
%% Open loop control

file=['ExampleSubmission',filesep,'SampleInput.mat'];
filesave=['ExampleSubmission',filesep,'SampleOutput.mat'];

Inputs=load(file);

if isfield(Inputs,'v0')
    v0 = Inputs.v0;
else
    v0=sqrt(Car.R_max/Car.k); % default initial longitudinal speed
end

% Driving force as a function of time
Rcontrol=Inputs.Rcontrol;  
% Steering angle as a function of time
gamma=Inputs.gamma;
% if you instead define gamma as input, then the numerical differentiation to get gamma_dot
diff_num=@(f,dt,t) (f(t+dt)-f(t-dt))/(2*dt);
gamma_dot=@(t) diff_num(gamma,1e-6,t);

Inputs.gamma_dot=gamma_dot;
Inputs.gamma = gamma;

gamma0=gamma(0);

% make sure you set a time long enough
Time=350;

% Animation=1; % Animation on
Animation=1; % Animation off

%%
usize=10;
% By default, the car RWD will give out dX and 10 values for monitoring purpose
% They are [R;gamma_dot; tgamma; dtgamma; s0;sf;sr;n0;nf;nr]; at time t
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%End of TODOS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Check for the input constraints
x0=[pos0(1:2);Track.ftheta(s_start);v0; gamma0];
sim_step=0.05; % do not change this as I will using this for the competition

[Rcontrol_real,gamma_real,gamma_dot_real]=InputChecker(Rcontrol,gamma,gamma_dot,Car,Time,sim_step);

Inputs.Rcontrol_real=Rcontrol_real;
Inputs.gamma_real=gamma_real;
Inputs.gamma_dot_real=gamma_dot_real;

car_dynamics=@(t,x,y,psi,sigma, gamma, s) car_RWD(t,x,y,psi,sigma,gamma, ...
                                                 gamma_dot_real(t),Rcontrol_real(t),Track,Car,s);                    
sys=@(t,x,para) car_dynamics(t,x(1),x(2),x(3),x(4), x(5),para);

%% Run

[t,y,u, TotalTime, Num_of_violation]=CarSimRealTime(sys,[0 Time],x0,s_start,sim_step,usize,Track,Car,Animation);

%% Lateral Forces
[Ffl_ana,Frl_ana]=Force_rwd(y(:,4),u(:,1),u(:,2),u(:,3),Car.m,Car.m0,Car.b,Car.w);   

if any(Num_of_violation ~= 0)
    fprintf("Num_of_violation: off track %d, front force %d, rear force %d", ...
             Num_of_violation(1), Num_of_violation(2), Num_of_violation(3));
else
    fprintf("Finished without violation.\n");
end
% reconstruct
save(filesave,'Inputs','t','y','u', "Num_of_violation");

plot_results;
