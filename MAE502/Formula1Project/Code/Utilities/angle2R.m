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
% Convert the angles theta, mu and phi to a rotation matrix R
% mainly used for track (ribbon) coordinate and global coordinate conversion
% theta, mu and phi should be of size 1*N
function R=angle2R(theta,mu,phi)
   t=[cos(theta).*cos(mu)
      sin(theta).*cos(mu) 
      -sin(mu)];
   n=[cos(theta).*sin(mu).*sin(phi)-sin(theta).*cos(phi);
      sin(theta).*sin(mu).*sin(phi)+cos(theta).*cos(phi);
      cos(mu).*sin(phi)];
   m=[cos(theta).*sin(mu).*cos(phi)+sin(theta).*sin(phi);
      sin(theta).*sin(mu).*cos(phi)-cos(theta).*sin(phi);
      cos(mu).*cos(phi)];
   R=[t,n,m];
end
