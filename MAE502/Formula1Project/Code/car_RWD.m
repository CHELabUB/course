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
% Rear wheel drive 2D bicycle model.

% The states are 
%      x coordinate
%      y coordinate
%      psi yaw angle
%      sigma longitudinal speed
%      gamma steering angle
% The inputs are 
%      R driving force at rear wheel   
%      gamma_dot time derivative of steering angle
% Car is the structure variable for Car parameters
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

% function Cartesian2Track is used to convert Cartesian coordinate x y psi to
% the description relative to track ribbon.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [dX, u] = car_RWD(t, x, y, psi, sigma, gamma,...
                           gamma_dot, R,...
                           Track, Car, s0_prior)
    % return the s and n coordinate along the track of COM (0), front wheel (f) and rear wheel (r)
    [s0,sf,sr,n0,nf,nr]=Cartesian2Track(x,y,psi,s0_prior,Track,Car);
    % dynamics
    tgamma = tan(gamma);
    dx = (cos(psi)-Car.b/Car.w * sin(psi) * tgamma) * sigma;
    dy = (sin(psi)+Car.b/Car.w * cos(psi) * tgamma) * sigma;
    dpsi = sigma * tgamma / Car.w;
    dgamma = gamma_dot;
    dtgamma = 1 / (cos(gamma))^2 * gamma_dot;
    dv = (R - Car.k * sigma^2 - Car.m0 * tgamma * dtgamma * sigma) / (Car.m + Car.m0 * tgamma^2);
    %% output derivative and monitoring states
    % default states is of 5, add more to the end if needed
    dX = [dx;dy;dpsi;dv;dgamma];
    % output the control and update of parameters
    % keep R and gamma_dot as control input in case they may be given by feedback laws
    u = [R; gamma_dot; tgamma; dtgamma; s0; sf; sr; n0; nf; nr];
end
