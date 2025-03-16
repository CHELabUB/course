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
% Approximate gamma_dot limit by the force limit
function gamma_dot_limit = gamma_dot_limit_by_Force(sigma, gamma, Car)
    % approximation only, dropped the force
    L = Car.w;
    d = Car.b;
    m = Car.m;
    m0 = Car.m0;
    Frl_max = Car.Frl_max;
    Ffl_max = Car.Ffl_max;
    gamma = abs(gamma);
    % m0*L-m*d < 0 for this vehicle 
    due_to_Frl = @(Frl) (Frl - m/L*(1-d/L)*tan(gamma).*sigma.^2) ...
        * L * (m*(cos(gamma)).^2+m0*(sin(gamma)).^2) ...
        / (m*abs(m0*L-m*d)*sigma);

    due_to_Ffl = @(Ffl) (Ffl - m*d/L^2*tan(gamma)./cos(gamma).*sigma.^2) ...
        * (m*(cos(gamma)).^2+m0*(sin(gamma)).^2) * cos(gamma) ...
        / (m*m0*sigma);

    boundary = [due_to_Frl(Frl_max), due_to_Ffl(Ffl_max)];
    gamma_dot_limit = max(0, min(boundary));
end
