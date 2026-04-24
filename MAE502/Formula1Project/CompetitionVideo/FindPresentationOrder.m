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
% 
% clear
% clc
% close all
%% Number and Names of teams 

% Teamname={...
% 'Max-JetSpeed        ';
% 'Kreuzer             ';
% 'Viper Racing        ';
% 'Force India         ';
% '404 Not Fast        ';
% 'Rodriguez           ';
% 'Gurram              ';
% 'Albino-Isa          ';
% 'Example Open loop   ';
% 'Example Closed loop ';
% }
Teamname={...
'Example Open        ';
'Example Stanley     ';
}
N=length(Teamname);

%%

ratio=1000;
num=mod(round(rand(1)*ratio),N);
if num==0
    num=N; 
end
order(1)=num;
while length(order)<N
    num=mod(round(rand(1)*ratio),N);
       if num==0
          num=N; 
       end    
    new=num~=order;
    if prod(new)==1

       order=[order;num]; 
    end
end
%
List=cell(N,2);
for i=1:N
   List{i,1}=i;
   List{i,2}=Teamname{order(i)};
end
%% Output
fprintf('Let us show the videos in this order.\n');
List
