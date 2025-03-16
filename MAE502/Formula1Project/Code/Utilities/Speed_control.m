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
% Example of a speed controller
function [R, v_desired, saturated] = Speed_control(t, x, y, psi, v, gamma, z_v, ...
                                        s0, sf, sr, n0, nf, nr, ...
                                        path, Car, Kp_gain, Ki_gain)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% get error
    v_desired = path.v(sf); % desired speed

    R_ff = Car.k * v_desired^2; % feed forward term

    R = R_ff + Kp_gain * (v_desired - v) + Ki_gain * z_v;

    % cap the R and allow anti-windup
    % R = max(Car.R_min, min(Car.R_max, R));
    if R > Car.R_max
        R = Car.R_max;
        saturated = 1;
    else
        if R < Car.R_min
            R = Car.R_min;
            saturated = 1;
        else
            saturated = 0;
        end
    end
end
