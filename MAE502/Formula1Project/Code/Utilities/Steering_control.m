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
%% Template for steering control
function [gamma_dot, gamma_desired, saturated] = Steering_control(...
                            t, x, y, psi, v, gamma, z_gamma, ...
                            s0, sf, sr, n0, nf, nr, ...
                            path, Car, K_lateral, Kp_gain, Ki_gain)
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % based on stanley model
   %%
   ntol=0.1;   % dead zone for the lateral error
   alphatol=0.02; % dead zone for the heading error
   %% get error
   n_ref = path.n(s0); % positive if too the left of the center
   n = n0 - n_ref; % positive if too the left of the path
   % dead zone: almost on the path no need to adjust
   if abs(n) < ntol
      n = 0;
   end

   tfront=[cos(psi);sin(psi)];  % the vector along the vehicle body
   t_sfront=path.t(sf);%   the tangent vector along the path
   t_sfront=t_sfront(1:2);  % the vector on the path (planary one)

   alpha=acos(t_sfront'*tfront);  % both are unit vector
   % determine the angle
   value=det([tfront';t_sfront']); % from the heading direction to the reference direction
   if value < -alphatol  % t_front to the left of the reference direction should go back
      alpha = -alpha;   
   end
   % dead zone: almost parallel no need to adjust
   if abs(value)<=alphatol  
      alpha=0; 
   end

   % by default n is positive when towards the left of the path, ...
   % which need a NEGATIVE gamma to over come
   % by default alpha is positive when towards the right of the reference
   % tangent vector at the front, should have a POSITIVE angle to over come. 
   gamma_desired=alpha+atan(K_lateral*(-n)/v);  
   gamma_desired=max(Car.gamma_min,min(Car.gamma_max,gamma_desired));

   % Use PI control to follow desired steering angle gamma_desired
   gamma_error = gamma_desired-gamma;
   gamma_dot = Kp_gain*gamma_error + Ki_gain*z_gamma;

   gamma_dot_max = Car.gamma_dot_max;
   gamma_dot_min = Car.gamma_dot_min;

   % % Also want to cap with force constraints.
   % % the following does not work yet.
   % gamma_dot_limit = gamma_dot_limit_by_Force(v, gamma, Car);
   % gamma_dot_max = min(Car.gamma_dot_max, gamma_dot_limit);
   % gamma_dot_min = max(Car.gamma_dot_min, -gamma_dot_limit);

   % cap the gamma_dot and allow anti-windup
   % gamma_dot=max(gamma_dot_min, min(gamma_dot, gamma_dot_max));
   if gamma_dot > gamma_dot_max
      gamma_dot = gamma_dot_max;
      saturated = 1;
   else
      if gamma_dot < gamma_dot_min
         gamma_dot = gamma_dot_min;
         saturated = 1;
      else
         saturated = 0;
      end
   end
end
