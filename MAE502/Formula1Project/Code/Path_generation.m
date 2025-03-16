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
% Example script to generate a path/trajectory for the car to follow
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;clc;close all;
%%
addpath("Utilities")
%% Car information
load('F1CarData.mat')
Car=CarParameter;
%% Track Information
load('CircuitOfAmerica.mat');
% the track has 1 to 597 points, contains a loop
s_start=0; % start from the race start and end at the same location after one lap
% corresponds to 11th point in the track, and 587 is the same as 11th point
start_index = 11; 
end_index = 587; 
total_indexes = length(Track.arc_s);
is_loop = true;
save_name = "sample_path";
speed_scale = 0.5;

% Manually slow down for some sharp turns
% Define sections with [s_start, s_end] and v_ref_raw
manual_sections = [
    % [s_start, s_end], v_ref_raw
    [200, 500], 10;
    [700, 850], 45;
    [1400, 1800], 25;
    [2100, 2400], 10;
    [2800, 3200], 62;
    [3200, 4200], 15;
    [4350, 4450], 25;
    [4700, 4850], 10;
    [4850, 5000], 30;
    [5000, 5150], 10;
];



% load('McityTrackFitted2Dext.mat');
% % the track has 1 to 597 points, contains a loop
% s_start=0; % start from the race start and end at the same location after one lap
% % corresponds to 11th point in the track, and 587 is the same as 11th point
% start_index = 1; 
% end_index = 477; 
% total_indexes = length(Track.arc_s);
% is_loop = false;
% save_name = "sample_path_Mcity";
% speed_scale = 0.3;

figure(1);clf;
hold on;axis equal;box on
plot3(Track.bl(1,:),Track.bl(2,:),Track.bl(3,:),'k','Markersize',5);
plot3(Track.br(1,:),Track.br(2,:),Track.br(3,:),'k','Markersize',5);
plot3(Track.cline(1,:),Track.cline(2,:),Track.cline(3,:),'k--','Markersize',5);
for i=1:length(Track.bl(1,:))
    if mod(i-1,20)==0
plot3([Track.bl(1,i) Track.br(1,i)],[Track.bl(2,i) Track.br(2,i)],[Track.bl(3,i) Track.br(3,i)],'b','LineWidth',1)
    end

end

%% estimate curvature of the track
s_sam = Track.arc_s;
theta=Track.theta;
if is_loop
    curvature_loop_raw = diff(theta(start_index:end_index)) ./ diff(s_sam(start_index:end_index));
    curvature_loop_smooth = sgolayfilt(curvature_loop_raw, 3, 21); % smooth the curvature


    close_the_loop = @(x) [x(:, (end - start_index + 1):(end)),... 
                        x,...
                        x(1:(total_indexes - end_index))];


    curvature_raw = close_the_loop(curvature_loop_raw);
    curvature = close_the_loop(curvature_loop_smooth);
else
    curvature_raw  = diff(theta) ./ diff(s_sam);
    curvature_smooth = sgolayfilt(curvature_raw, 3, 11); % smooth the curvature
    curvature_raw = [curvature_raw, curvature_raw(end)];
    curvature = [curvature_smooth, curvature_smooth(end)];
end

%% Generate speed trajectory
% goal: the sharper the turn, the slower the speed
% captured by the curvature of the path.
sigma_max = sqrt(Car.R_max / Car.k);

Force_rwd_cornering_front = @(gamma,Ffl_max, Frl_max, m, ~, d, L) sqrt(Ffl_max / (m*d/L^2*tan(gamma)/cos(gamma)));
Force_rwd_cornering_rear = @(gamma,Ffl_max, Frl_max, m, ~, d, L) sqrt(Frl_max / (m/L*(1-d/L)*tan(gamma)));

v_ref_raw = zeros(1, total_indexes);
for i = 1:total_indexes
    gamma = atan(Car.w * abs(curvature(i)));
    v_ref_raw(i) = min(Force_rwd_cornering_front(gamma, Car.Ffl_max, Car.Frl_max, Car.m, Car.m0, Car.b, Car.w),...
                   Force_rwd_cornering_rear(gamma, Car.Ffl_max, Car.Frl_max, Car.m, Car.m0, Car.b, Car.w));
end

% slow down uniformally
v_ref_raw = v_ref_raw * speed_scale;
v_ref_raw(v_ref_raw > sigma_max) = sigma_max;


% Apply the speed limits to the sections
v_ref = v_ref_raw;
for i = 1:size(manual_sections, 1)
    s_start = manual_sections(i, 1);
    s_end = manual_sections(i, 2);
    v_ref_section = manual_sections(i, 3);
    
    % Find the indices corresponding to the s_start and s_end
    start_idx = find(s_sam >= s_start, 1);
    end_idx = find(s_sam <= s_end, 1, 'last');
    
    % Apply the speed limit to the section
    v_ref(start_idx:end_idx) = min([v_ref_section, v_ref_raw(start_idx:end_idx)]);
end

figure(2);clf;
subplot(3,1,1);hold on;box on; grid on;
plot(s_sam, curvature, 'b', 'LineWidth', 1);
plot(s_sam, curvature_raw, 'r--', 'LineWidth', 1);
ylabel("$1/\rho$ [1/s]", "Rotation", 0);
legend("smoothed", "raw");
subplot(3,1,2);
plot(s_sam, theta, 'b', 'LineWidth', 1);
ylabel("$\theta$ [rad]", "Rotation", 0);
subplot(3,1,3);hold on;grid on; box on;
plot(s_sam, v_ref, 'b', 'LineWidth', 1);
plot(s_sam, v_ref_raw, 'r', 'LineWidth', 1);
plot(s_sam, v_ref_raw * 0 + sigma_max, 'k--', 'LineWidth', 1);
ylabel('v [m/s]', "Rotation", 0);
xlabel('s [m]');
axleHandles = findobj(gcf, 'Type', 'axes');
linkaxes(axleHandles, 'x');
%% generate a path using the track coordinates
% example: generate a sinusoidal path along the track center

amp = 0; % there is a problem with the track fitting. need to Refine this.
freq = 20; % for whole loop
w = 2*pi / (s_sam(end_index)-s_sam(start_index))*freq;
X = Track.cline;
n_path = amp * sin(w * s_sam);
X_path=[s_sam; s_sam; s_sam] * 0;
for i=1:length(s_sam)
    % 2D so only theta is non-zero
    R = angle2R(theta(i), 0 , 0);
    X_path(:,i) = R * [0; n_path(i); 0] + X(:,i);
end

% use numerical expression ot get tangent vector of t
deltaXpath = X_path(:,2:end) - X_path(:,1:end-1);
deltaX=sqrt(deltaXpath(1,:).^2 + deltaXpath(2,:).^2 + deltaXpath(3,:).^2);
t_path=[deltaXpath(1,:)./deltaX;
       deltaXpath(2,:)./deltaX;
       deltaXpath(3,:)./deltaX;];
t_path=[t_path,t_path(:,end)];   

% distance to the center line
Path.n=@(s) interp1(s_sam,n_path,s);

Path.X=@(s) [interp1(s_sam,X_path(1,:),s)
             interp1(s_sam,X_path(2,:),s)
             interp1(s_sam,X_path(3,:),s)];
Path.t=@(s) [interp1(s_sam,t_path(1,:),s)
             interp1(s_sam,t_path(2,:),s)
             interp1(s_sam,t_path(3,:),s)];   
Path.v=@(s) interp1(s_sam, v_ref, s);

figure(1);
plot3(X_path(1,:),X_path(2,:),X_path(3,:),'r','LineWidth',1);
mmm = 50;
for i=1:length(X_path(1,:))
    if mod(i-1,20)==0
        plot3([X_path(1,i) X_path(1,i) + t_path(1,i) * mmm], ...
              [X_path(2,i) X_path(2,i) + t_path(2,i) * mmm], ...
              [X_path(3,i) X_path(3,i) + t_path(3,i) * mmm], 'c','LineWidth',1)
    end
end

plot3(X_path(1,:),X_path(2,:),v_ref,'g','LineWidth',2);
view(3);

%% save
save(save_name, "Path");
