%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAE 502 Vehicle Control Systems Project 
% Formula 1 car racing: subfunctions
%   Copyright (C)2025 Chaozhe He. All Rights Reserved.
%   Author: Prof. Chaozhe He
%           Department of Mechanical and Aerospace Engineering
%           SUNY University at Buffalo
%           March 2026
% Any issues/bug reports,
% please email to chaozheh@buffalo.edu
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;clc;close all;

%% Car information
load('F1CarData.mat')
Car=CarParameter;
color=distinguishable_colors(12);
%% Track Information
load('CircuitOfAmerica.mat');
%% TODO Need to update this with new results.
% 8 team this year.
Teamname={...
'2026 Champion        '; %  
'2025 Champion        '; %  
'Example Open loop    '; %  
'Example Closed loop  '; % 
};
%% If random

% FindPresentationOrder
%% if Predefine
% order=[1:length(Teamname)]';
% textlength=zeros(length(Teamname),1);
% TeamnameReal=cell(length(Teamname),1);
% for i=1:length(Teamname)
%     TeamnameReal{i}=Teamname{order(i)};    
%     textlength(i)=length(TeamnameReal{i});
% end
% textlength_max=max(textlength);
% pause;
%% Team run
%% for test
Team_can=[1,2,3,4];
% Team_can = 1:length(Teamname);
Traj=cell(length(Team_can),3);
Cars=cell(length(Team_can),1);
Names=cell(length(Team_can),1);
for i=1:length(Team_can)
    TeamNum=Team_can(i);
    file= fullfile("Sample", "Team" + num2str(TeamNum));
    results=load(file);
    Traj{i,1}=results.t;
    Traj{i,2}=results.y;
    Traj{i,3}=results.u;
    Traj{i,4}=results.Num_of_violation;
    Cars{i}=Car;
    % Names{i}=[results.Inputs.Teamname,char(32)*ones(1,(textlength_max-textlength(i))*2)];
    Names{i}=Teamname{TeamNum};
    fprintf("Team %d, %s, %s \n", i, Names{i}, results.TeamName);
end

%% Generate videos

color(4,:)=[]; % don't want black
MatchRun(length(Cars),Traj,Cars,Names,Track,color)

%% Summary Plot 
% plot the trajectory and control enforce

Height=0.85;
Width=0.7;
FontSize=12;
showzoom=1;
figure(101);
set(gcf,'units','normalized');
pos_default = get(gcf,'pos');
pos1=pos_default;
pos1(1)=pos1(1)+pos1(3)/2-Width/2;
pos1(2)=pos1(2)-(Height-pos1(4));
pos1(3)=Width;
pos1(4)=Height;
figure(101);
set(gcf,'pos',pos1)
subplot(5,2,1);hold on;box on;
subplot(5,2,3);hold on;box on;
subplot(5,2,5);hold on;box on;
subplot(5,2,7);hold on;box on;
subplot(5,2,9);hold on;box on;
subplot(5,2,2);hold on;box on;
subplot(5,2,4);hold on;box on;
subplot(5,2,6);hold on;box on;
subplot(5,2,8);hold on;box on;
subplot(5,2,10);hold on;box on;

pos2=pos_default;
pos2(1)=pos2(1)+pos2(3)/2-Width/2;
pos2(2)=pos2(2)-(Height-pos2(4));
pos2(3)=Width/3*2;
pos2(4)=Height/3*2;
figure(102)
set(gcf,'units','normalized');
hold on;box on;axis equal;
set(gcf,'pos',pos2)
hold on;box on;axis equal;
plot3(Track.bl(1,:),Track.bl(2,:),Track.bl(3,:),'k-','LineWidth',1);
plot3(Track.br(1,:),Track.br(2,:),Track.br(3,:),'k-','LineWidth',1);
plot3(Track.cline(1,:),Track.cline(2,:),Track.cline(3,:),'k--','LineWidth',1);

% legend_name = cell(1, length(Cars));
legend_handle = zeros(1, length(Cars));
for i=1:length(Cars)
%% Lateral Forces
t=Traj{i,1};
y=Traj{i,2};
u=Traj{i,3};
[Ffl_ana,Frl_ana]=Force_rwd(y(:,4), u(:,1) - Car.k * y(:,4).^2, y(:,5), u(:,2),...
                            Car.m,Car.m0,Car.b,Car.w);  
R=u(:,1);
ss=u(:,5);
nn=u(:,8);
gamma=y(:,5);
gamma_dot=u(:,2);
%% Show the trajectories

figure(101)
subplot(5,2,1)
plot(t,y(:,1),'color',color(i,:),'LineWidth',2);
subplot(5,2,3)
plot(t,y(:,2),'color',color(i,:),'LineWidth',2);
subplot(5,2,5)
plot(t,y(:,3),'color',color(i,:),'LineWidth',2);
subplot(5,2,7)
plot(t,y(:,4),'color',color(i,:),'LineWidth',2);

subplot(5,2,2)
plot(t,R,'color',color(i,:),'LineWidth',2);
subplot(5,2,4)
plot(t,ss,'color',color(i,:),'LineWidth',2);
subplot(5,2,6)
plot(t,nn,'color',color(i,:),'LineWidth',2);
subplot(5,2,8)
plot(t,gamma,'color',color(i,:),'LineWidth',2);
subplot(5,2,9);
plot(t,Ffl_ana,'color',color(i,:),'LineWidth',2);
subplot(5,2,10);
plot(t,Frl_ana,'color',color(i,:),'LineWidth',2);

%% Show route together with track
figure(102)
legend_handle(i) = plot(y(:,1),y(:,2),'LineWidth',2,'color',color(i,:));
end
L=[];
figure(101);
set(gcf,'pos',pos1)
subplot(5,2,1)
L=[L,ylabel('$x$[m]')];
subplot(5,2,3)
L=[L,ylabel('$y$[m]')];
subplot(5,2,5)
L=[L,ylabel('$\psi$[rad]')];
subplot(5,2,7)
L=[L,ylabel('$v$[m/s]')];
subplot(5,2,2)
L=[L,ylabel('$R$[N]')];
xL=xlim;
plot(xL,xL*0+Car.R_max,'k--','LineWidth',1.5);
plot(xL,xL*0+Car.R_min,'k--','LineWidth',1.5);
ylim([Car.R_min-200,Car.R_max+200])
subplot(5,2,4)
L=[L,ylabel('$s$[m]')];
subplot(5,2,6);hold on
L=[L,ylabel('$n$[m]')];
subplot(5,2,8)
L=[L,ylabel('$\gamma$[rad]')];
plot(xL,xL*0+Car.gamma_max,'k--','LineWidth',1.5);
plot(xL,xL*0+Car.gamma_min,'k--','LineWidth',1.5);
ylim([Car.gamma_min-0.03,Car.gamma_max+0.03])
subplot(5,2,9);
L=[L,ylabel('$F_{\rm F}$[N]')];L=[L,xlabel('$t$[sec]')];
plot(xL,xL*0+Car.Ffl_max,'k--','LineWidth',1.5);
plot(xL,xL*0-Car.Ffl_max,'k--','LineWidth',1.5);
ylim([-Car.Ffl_max-200,Car.Ffl_max+200])
subplot(5,2,10);
L=[L,ylabel('$F_{\rm R}$[N]')];
L=[L,xlabel('$t$[sec]')];
plot(xL,xL*0+Car.Frl_max,'k--','LineWidth',1.5);
plot(xL,xL*0-Car.Frl_max,'k--','LineWidth',1.5);
ylim([-Car.Frl_max-200,Car.Frl_max+200])
linkaxes(findall(gcf, 'Type', 'axes'), 'x');
figure(102)
L=[L,xlabel('$x$[m]')];
L=[L,ylabel('$y$[m]')];
L = [L, legend(legend_handle, Names)];
set(L,'Interpreter','latex');
