% MatchRun: plot a match run
% N is car number.
% 
function MatchRun(N,Trajectory,Cars,Name,Track,color)
%% setup figures size

FontSize=14;
showzoom=1;
set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', FontSize*showzoom)
set(0,'DefaultTextFontname', 'Times New Roman')
set(0,'DefaultTextFontSize', FontSize*showzoom)

% Window size relative size
% Height=0.85;
% Width=0.7;
% set(0,'defaultlinelinewidth',2*showzoom)
% figure(101);
% set(gcf,'units','normalized');
% pos_default = get(gcf,'pos');
% pos1=pos_default;
% pos1(1)=pos1(1)+pos1(3)/2-Width/2;
% pos1(2)=pos1(2)-(Height-pos1(4));
% pos1(3)=Width;
% pos1(4)=Height;
% close gcf
%%

Fig=figure(1);
% set(gcf,'units','normalized','pos',pos1);
setFigure(Fig, [16, 10])

%% Plot the Track
centerline=Track.cline;
bl=Track.bl;
br=Track.br;
bst=[Track.bstl,Track.bstr];
bf=[Track.bfl,Track.bfr];
bl_c=bl;
br_c=br;
figure(1)
subplot(3,2,[1 3]);
% zoom view
axis equal
hold on
plot(centerline(1,:),centerline(2,:),'k--');
plot(bl_c(1,:),bl_c(2,:),'k','Markersize',5);
plot(br_c(1,:),br_c(2,:),'k','Markersize',5);
plot(bf(1,:),bf(2,:),'m','LineWidth',10);
plot(bst(1,:),bst(2,:),'g','LineWidth',5);
box on;xlabel('x[m]');ylabel('y[m]');
subplot(3,2,[2 4]);
% global view
axis equal
hold on
plot(centerline(1,:),centerline(2,:),'k--');
plot(bl_c(1,:),bl_c(2,:),'k','Markersize',5);
plot(br_c(1,:),br_c(2,:),'k','Markersize',5);
box on;xlabel('x[m]');ylabel('y[m]');
plot(bf(1,:),bf(2,:),'m','LineWidth',10);
plot(bst(1,:),bst(2,:),'g','LineWidth',5);

%% Setup simulation table
% combine the states

Finish_index=zeros(1,N);
Finish_time=zeros(1,N);
Lap_time = zeros(1,N);
Final_distance = zeros(1,N);
for i=1:N
    Num_of_violation = Trajectory{i,4};
    Finish_index(i)=length(Trajectory{i,1});
    Finish_time(i)=Trajectory{i,1}(end);
    Lap_time(i) = Finish_time(i);
    Final_distance(i) = Track.arc_s(end); % for those finished, the distance is the length of the track.
    if any(Num_of_violation>0)
        Lap_time(i) = +Inf;
        Final_distance(i) = Trajectory{i,3}(end,5);
    end

end
% for those terminated, rank according to distance travelled.

[~,FinalOrder] = sortrows([Lap_time(:), -Final_distance(:)], [1,2]);
[~,FinishTimeOrder]=sort(Finish_time);
[~,FinalRank]=sort(FinalOrder);
Finish_time_total=max(Finish_time);
Time=[0:0.05:floor(Finish_time_total/0.05)*0.05,Finish_time_total];Time=Time(:);
X=zeros(length(Time),N);
Y=zeros(length(Time),N);
Psi=zeros(length(Time),N);
Sigma=zeros(length(Time),N);
Gamma=zeros(length(Time),N);
Gammadot=zeros(length(Time),N);
R=zeros(length(Time),N);
S=zeros(length(Time),N);
% Order=zeros(length(Time),N);
vmax=80;
for i=1:N
  % state: x, y, psi, sigma, gamma 
  X(1:Finish_index(i)-1,i)=Trajectory{i,2}(1:Finish_index(i)-1,1);
    X(Finish_index(i):end,i)=Trajectory{i,2}(Finish_index(i),1);
  Y(1:Finish_index(i)-1,i)=Trajectory{i,2}(1:Finish_index(i)-1,2);
    Y(Finish_index(i):end,i)=Trajectory{i,2}(Finish_index(i),2);
  Psi(1:Finish_index(i)-1,i)=Trajectory{i,2}(1:Finish_index(i)-1,3);
    Psi(Finish_index(i):end,i)=Trajectory{i,2}(Finish_index(i),3);
  Sigma(1:Finish_index(i)-1,i)=Trajectory{i,2}(1:Finish_index(i)-1,4);
    Sigma(Finish_index(i):end,i)=Trajectory{i,2}(Finish_index(i),4);
  Gamma(1:Finish_index(i)-1,i)=Trajectory{i,2}(1:Finish_index(i)-1,5);
    Gamma(Finish_index(i),i)=Trajectory{i,2}(Finish_index(i),5);
  % input: R, gamma dot 
  R(1:Finish_index(i)-1,i)=Trajectory{i,3}(1:Finish_index(i)-1,1);
  R(Finish_index(i),i)=Trajectory{i,3}(Finish_index(i),1);
  Gammadot(1:Finish_index(i)-1,i)=Trajectory{i,3}(1:Finish_index(i)-1,2);
  Gammadot(Finish_index(i),i)=Trajectory{i,3}(Finish_index(i),2);
  S(1:Finish_index(i)-1,i)=Trajectory{i,3}(1:Finish_index(i)-1,5);
  S(Finish_index(i):end,i)=Trajectory{i,3}(Finish_index(i),5);
  if Lap_time(i) < Inf
      % s (keep rolling even if they stop)
      % this would allow the final ranking to sustain
      S(Finish_index(i):end,i)=(Time(Finish_index(i):end)-Trajectory{i,1}(Finish_index(i)))*vmax ...
          +Trajectory{i,3}(Finish_index(i),5);
  end
end
s_final=Track.arc_s(587);
% determine the order
[~,Order]=sort(-S,2);
[~,Rank]=sort(Order,2);
[~,Leader]=max(S,[],2);
S_rest=S;
% [~,FinalRank]=sort(FinalOrder);
for i=1:N-1
   index=find(Time>Finish_time(FinishTimeOrder(i)),1);
   S_rest(:,FinishTimeOrder(i))=0;
   [~,Leader_rest]=max(S_rest,[],2);
   Leader(index:end)=Leader_rest(index:end);
end

iter_final=length(Time);


%% Start video
% Time_step=Trajectory{1,1}(2);
videoname='Match';
writerObj = VideoWriter(videoname,'MPEG-4'); % Name it.
writerObj.FrameRate = 20; % How many frames per second.
open(writerObj); 

L=zeros(N,5);
Lg=zeros(1,N+1);
focus_window_size = 15;
focus_window_size_zoom = 20;
%% Initial Frame paused   
iter=1;
subplot(3,5,12:14);box on;hold on;
ylim([-1,N+1]);
Lg(N+1)=text(0.35,N,sprintf('Time:%.2f\n',Time(iter)));
   for i=1:N

         x= X(iter,i);
         y= Y(iter,i); 
         psi=Psi(iter,i);
         gamma=Gamma(iter,i);

         L(i,:)=DrawCar(Cars{i},...
                    x,y,psi,gamma,...
                    color(i,:));

          subplot(3,5,12:14);
         Lg(i)=text(0.05,N-Rank(iter,i),[Name{i},'  Distance:',sprintf('%.2f',S(iter,i))],'color',color(i,:));
   
   end
   
focus = [X(iter,Leader(iter)), Y(iter,Leader(iter))];
yL(1)=focus(2)-focus_window_size;yL(2)=focus(2)+focus_window_size;
xL(1)=focus(1)-focus_window_size;xL(2)=focus(1)+focus_window_size;   

subplot(3,2,[1 3]);
xlim([xL(1) xL(2)]);
ylim([yL(1) yL(2)]);  
subplot(3,2,[2 4]);
yL(1)=focus(2)-focus_window_size_zoom;yL(2)=focus(2)+focus_window_size_zoom;
xL(1)=focus(1)-focus_window_size_zoom;xL(2)=focus(1)+focus_window_size_zoom;   
window = [xL(1), xL(2), xL(2), xL(1), xL(1);
         yL(1), yL(1), yL(2), yL(2), yL(1)];
Lwindow = plot(window(1,:),window(2,:),'k','LineWidth',1);

frame = getframe(Fig); 
writeVideo(writerObj, frame);
   


get_distance = @(p1, p2) norm(p1 - p2, 2);
mid_point = @(p1, p2, lambda) lambda * p2 + (1 - lambda) * p1;
stopped_veh = [];
running_veh = 1:N;
% for debugging purpose, we can skip the frames to speed up the process
% skip = 20;
% focus_move_step = 200;
% % for actual run, set skip = 1
skip = 1;
focus_move_step = 4;
for iter=2:skip:iter_final
 
   for i=1:N
         if any(stopped_veh(:) == i)
            continue;
         end
         x= X(iter,i);
         y= Y(iter,i); 
         psi=Psi(iter,i);
         gamma=Gamma(iter,i);

         delete(L(i,:));        
         L(i,:)=DrawCar(Cars{i},...
                    x,y,psi,gamma,...
                    color(i,:));
   end

   % focus on the leader but will move
   
   if iter > 1 && Leader(iter) ~= Leader(iter-1)
      % if the leader changes, then we need to update the view
      old_focus = [X(iter,Leader(iter - 1)), Y(iter,Leader(iter - 1))];
   else
      old_focus = focus;
   end

   new_focus = [X(iter,Leader(iter)), Y(iter,Leader(iter))];
   if get_distance(old_focus, new_focus) > focus_move_step
      % if the distance is larger than the step, then we need to move the focus
      lambda = focus_move_step / get_distance(old_focus, new_focus);
      focus = mid_point(old_focus, new_focus, lambda);
   else
      % if the distance is smaller than the step, then we can just use the new focus
      focus = new_focus;
   end

   yL(1)=focus(2)-focus_window_size;yL(2)=focus(2)+focus_window_size;
   xL(1)=focus(1)-focus_window_size;xL(2)=focus(1)+focus_window_size;   
   
   subplot(3,2,[1 3]);
   xlim([xL(1) xL(2)]);
   ylim([yL(1) yL(2)]);

   subplot(3,2,[2 4]);
   yL(1)=focus(2)-focus_window_size_zoom;yL(2)=focus(2)+focus_window_size_zoom;
   xL(1)=focus(1)-focus_window_size_zoom;xL(2)=focus(1)+focus_window_size_zoom; 
   window = [xL(1), xL(2), xL(2), xL(1), xL(1);
             yL(1), yL(1), yL(2), yL(2), yL(1)];
   delete(Lwindow);  
   Lwindow = plot(window(1,:),window(2,:),'k','LineWidth',1);
   subplot(3,5,12:14);
   delete(Lg([running_veh, N+1]));
   Lg(N+1)=text(0.35,N,sprintf('Time:  %.2f [s]\n',Time(iter)));
   for i=1:N
      if all(running_veh(:) ~= i)
         continue;
      end
      if Time(iter)<Finish_time(i) 
         Lg(i)=text(0.05,N-Rank(iter,i),[num2str(Rank(iter, i)), ' ', Name{i},'  Distance:  ',sprintf('%.2f',S(iter,i)),' [m]  Speed:  ',sprintf('%.2f',Sigma(iter,i)),' [m/s]'],'color',color(i,:));
      else
         % add vehicle to stop vehicle list.
         if ~any(stopped_veh == i)
            stopped_veh = [stopped_veh, i];
         end
         if FinalRank(i) == Rank(iter,i)
             % in case its rank is not settled down yet.
             running_veh = running_veh(running_veh ~= i);  
         end
         if Lap_time(i) == +Inf
            idx = find(Trajectory{i, 4} >0, 1, 'first');
            final_status = [num2str(Rank(iter,i)), ' ', Name{i},'  Terminated Time:  ',sprintf('%.3f',Finish_time(i)),' [s] Violation ', num2str(idx), '  Final Distance:  ',sprintf('%.2f',S(iter,i)),' [m]'];
         else
            final_status = [num2str(Rank(iter,i)), ' ', Name{i},'  Finished Time:  ',sprintf('%.3f',Finish_time(i)),' [s]  Average Speed: ',sprintf('%.2f',s_final/Finish_time(i)),' [m/s]'];
         end
         Lg(i)=text(0.02,N-Rank(iter,i), final_status,'color',color(i,:));
      end
   end
   frame = getframe(Fig); 
   writeVideo(writerObj, frame);
end
pause(3);
close(writerObj); % Saves the movie.
end



function setFigure(fig, figSizeInches)
    % figure_size = [width, height] in inches. If empty, keep the original size.
    % Use inches for figure units, optionally resize while preserving top-left corner.
    oldUnits = get(fig, 'Units');
    oldPos = get(fig, 'Position');
    set(fig, 'Units', 'inches');
    posInches = hgconvertunits(fig, oldPos, oldUnits, 'inches', groot);

    if isempty(figSizeInches)
        set(fig, 'Position', posInches);
        return;
    end

    if ~isnumeric(figSizeInches) || numel(figSizeInches) ~= 2
        error('figSizeInches must be a numeric [width, height] vector.');
    end

    newPos = posInches;
    topY = posInches(2) + posInches(4);
    newPos(3) = figSizeInches(1);
    newPos(4) = figSizeInches(2);
    newPos(2) = topY - newPos(4);
    set(fig, 'Position', newPos);
end
