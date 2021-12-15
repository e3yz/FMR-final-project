

clc;
clear all;
close all

load('map1.mat');
gif = figure;
filename='demo';
wall = drawMap(map);
xlabel('x position')
ylabel('y position')
title('FMR project demo')

pose = [2.5 7.5 0];
r = 0.4;

maxV = 1;
wheel2Center = 0.2;
[r_body,r_dir] = drawRobot(pose,r);

goal = pose(1:2);
g_fig = plot(goal(1),goal(2),'r*');
legend([wall,g_fig],{'wall','goal'});

% frame = getframe(gif);
% im = frame2im(frame);
% [imind,cm] = rgb2ind(im,256);
% imwrite(imind,cm,filename,'gif','DelayTime',0.1, 'Loopcount',inf);


%% LTL propositions:
% alarm, danger, r0 r1 r2 r3 r4 r5

fileName = 'fmr_single_bot.txt'; % filename in JSON extension
fid = fopen(fileName); % Opening the file
raw = fread(fid,inf); % Reading the contents
str = char(raw'); % Transformation
fclose(fid); % Closing the file
jsondata = jsondecode(str); % Using the jsondecode function to parse JSON from string
automaton = jsondata.nodes;

state_labels = fieldnames(automaton);

% initialize
current_label = state_labels{1};
prev_label = current_label;

% ENV input
alarm = false;
danger = false;

% preset waypoints
waypoints = [2.5 7.5;       % corresponding to region r0 to r5
    7.5 7.5;
    12.5 7.5;
    2.5 2.5;
    7.5 2.5;
    12.5 2.5];

max_step = 1000;  % max steps
step = 1;


% input prompt
prompt = 'Env signal(a for alarm, d for danger, ad for both, n for both off): ';

while(step < max_step)
    %% read change in ENV signal
    %     str = input(prompt,'s');
    t = timer('ExecutionMode', 'singleShot', 'StartDelay', 0.1, 'TimerFcn', @pressEnter);
    start(t);
    str = input(prompt, 's');
    stop(t);
    delete(t);
    if length(str) <=2 && ~isempty(str)
        if str == 'a'
            alarm = true;
            danger = false;
            subtitle('alarm on');
        elseif str == 'd'
            alarm = false;
            danger = true;
            subtitle('danger on')
        elseif str == 'ad'
            alarm = true;
            danger = true;
            subtitle('alarm & danger on')
        elseif str == 'n'
            alarm = false;
            danger = false;
            subtitle('both off')
        end
    end
    %% executing state machine
    if alarm || danger  % if any of the ENV sig is active
        
        % validate the current state,if wrong, roll back to previous state
        region = find(automaton.(current_label).state(3:8),1)-1
        min = 100;
        temp_region = region;
        for k = 1:6
            dis = norm(pose(1:2)-waypoints(k,:));
            if dis<min
                min = dis;
                temp_region = k-1;
            end
        end
        if temp_region~=region
            current_label = prev_label;
            disp('interrupted');
            region = find(automaton.(current_label).state(2:8),1)-1
        end
        
        % check next possible states
        next_state_labels = state_labels(jsondata.nodes.(current_label).trans +1);
        system_liveness = false;
        for i=1:length(next_state_labels)
            temp_label = next_state_labels{i};
            temp_node = automaton.(temp_label);
            temp_a = temp_node.state(1);
            temp_d = temp_node.state(2);
            if alarm == temp_a && danger == temp_d  % if next state match the ENV signal
                next_label = temp_label
                system_liveness = true;
                break;  % next  state is found
            end
        end
        
        if ~system_liveness     % failed to find next state
            disp('System failed');
            break;
        end
        
        % execute next state
        next_node = automaton.(next_label);
        next_region = find(next_node.state(3:8),1)-1
        goal = waypoints(next_region+1,:);
        prev_label = current_label;
        current_label = next_label; % update state label
    end
    
    % plot the goal point
    g_fig.XData = goal(1);
    g_fig.YData = goal(2);
    
    %% control robot to go to the goal point
    dt = 0.1;
    %     close_enough = 0.2;
    %     dis = norm(goal-pose(1:2));
    %     while dis > close_enough
    % feedback linearization from AMR
    Px = goal(1); Py = goal(2);
    Rx = pose(1); Ry = pose(2); theta = pose(3);
    epsilon = 0.5;
    
    % find Vx, Vy
    Vx = Px-Rx;
    Vy = Py-Ry;
    % set velocity to max (without this, robot will slow down when
    % close to the current goal, but we want to have continues motion)
    Vx = 100*Vx; Vy = 100*Vy;
    % find v and w
    [fwdVel,angVel] = feedbackLin(Vx,Vy,theta,epsilon);
    % check for motor saturation
    [cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center);
    % update robot pose
    pose(1) = cmdV*dt*cos(theta)+pose(1);
    pose(2) = cmdV*dt*sin(theta)+pose(2);
    pose(3) = theta+cmdW*dt;
    
    % plot & update robot pose
    delete(r_body);
    delete(r_dir);
    [r_body,r_dir] = drawRobot(pose,r);
    legend([wall,g_fig,r_dir],{'wall','goal','robot'});
    pause(dt/2);
    
    
    %         dis = norm(goal-pose(1:2));
    %     end
%     frame = getframe(gif);
%     im = frame2im(frame);
%     [imind,cm] = rgb2ind(im,256);
    % Write to the GIF File
%     imwrite(imind,cm,filename,'gif','DelayTime',0.1,'WriteMode','append');
    
%     if step == 0
%         imwrite(imind,cm,filename,'gif','DelayTime',0.1, 'Loopcount',inf);
%     else
%         imwrite(imind,cm,filename,'gif','DelayTime',0.1,'WriteMode','append');
%     end
    
    step=step+1;
end



% gif=figure;
% h0 = plot(0,0,'.k','MarkerSize',30);   hold on  % Origin
% h1 = plot(x1,y1,'k.','MarkerSize',30);           % C1
% h2 = plot(x2,y2,'k.','MarkerSize',30);           % C2
% h3 = plot(x3,y3,'k.','MarkerSize',30);           % end point
% h4 = plot(x1,y1,'r.','LineWidth',1);            % path
% h5 = plot(x2,y2,'g.','LineWidth',1);            % path
% h6 = plot(x3,y3,'b.','LineWidth',1);            % path
% h = plot(x,y,'k','Linewidth',5); % create plot object
% grid on;
% axis equal
% size = L1 + L2 + L3;
% axis([-size, size, -size, size])
% title('animated triple pendulum (DAE)')
% xlabel('x position');
% ylabel('y position');
% shg
%
%
%
% while ~complete
%
%     h1.XData = x1;        h1.YData = y1;          %update points
%     h2.XData = x2;        h2.YData = y2;
%     h3.XData = x3;        h3.YData = y3;
%     h4.XData = [h4.XData,x1]; h4.YData = [h4.YData,y1]; %update path
%     h5.XData = [h5.XData,x2]; h5.YData = [h5.YData,y2];
%     h6.XData = [h6.XData,x3]; h6.YData = [h6.YData,y3];
%
%     h.XData = x; h.YData = y;
%
%
%     drawnow
%
%         % Capture the plot as an image
%     frame = getframe(gif);
%     im = frame2im(frame);
%     [imind,cm] = rgb2ind(im,256);
%      % Write to the GIF File
%       if t == 0
%           imwrite(imind,cm,filename,'gif','DelayTime',0.1, 'Loopcount',inf);
%       else
%           imwrite(imind,cm,filename,'gif','DelayTime',0.1,'WriteMode','append');
%       end
% end



function [r_body,r_dir] = drawRobot(pose,r)

x = pose(1);
y = pose(2);
th = pose(3);

% draw circle with origion [x,y] and radius r
r_body = rectangle('Position',[x-r,y-r,2*r,2*r],'Curvature',[1,1]);
axis equal
% draw a directional line showing robot's direction
hold on;
r_dir = quiver(x,y,2*r*cos(th),2*r*sin(th),'k');
end



function pressEnter(HObj, event)
import java.awt.*;
import java.awt.event.*;
rob = Robot;
rob.keyPress(KeyEvent.VK_ENTER)
rob.keyRelease(KeyEvent.VK_ENTER)
end
