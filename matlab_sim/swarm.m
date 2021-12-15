
clc;
clear all;
close all

load('map1.mat');
gif = figure;
filename='swarm_demo.gif';
wall = drawMap(map);
axis ([-2.5 17.5 -2.5 12.5]);
set(gcf,'position',[1000,100,1200,900])

virtual_wall = plot([0 15],[5,5],'g--');
plot([5 5],[0 10],'g--');
plot([10 10],[0 10],'g--');
fontSize = 20;
xlabel('x position','FontSize', fontSize);
ylabel('y position','FontSize', fontSize);


n_robots = 4; % number of robots in the swarm


title(['FMR project - controller synthesis for swarm robots: ' num2str(n_robots) ' robots'],'FontSize', fontSize);

initial_pose = randomPoses(n_robots,0);
poses = zeros(n_robots,3);
poses(:,1:2) = initial_pose;
r = 0.2;
epsilon = 0.2;


maxV = 2;
wheel2Center = 0.2;

%% create figure handles for animation
r_body = gobjects(n_robots,1);
r_dir = gobjects(n_robots,1);
[r_body,r_dir] = drawRobots(poses,r,r_body,r_dir);

goals = poses(:,1:2);
g_fig = gobjects(n_robots,1);
g_link = gobjects(n_robots,1);
for j=1:length(g_fig)
    g_fig(j) = plot(goals(j,1),goals(j,2),'r*');
    g_link(j) = plot([poses(j,1),goals(j,1)],[poses(j,2),goals(j,2)],'b--');
end

% highlight the targeted region
g_region = gobjects(4,1);
g_region(1) = plot([0 5],[10 10],'r-.','linewidth',2);
g_region(2) = plot([0 5],[5 5],'r-.','linewidth',2);
g_region(3) = plot([0 0],[5 10],'r-.','linewidth',2);
g_region(4) = plot([5 5],[5 10],'r-.','linewidth',2);

legend([wall,virtual_wall,g_region(1),g_fig(1),g_link(1),r_dir(1)],{'wall','border','target region','goals','assignment','robot'},'fontSize',fontSize*3/5);

frame = getframe(gif);
im = frame2im(frame);
[imind,cm] = rgb2ind(im,256);
imwrite(imind,cm,filename,'gif','DelayTime',0.1, 'Loopcount',inf);


%% LTL propositions:
% alarm, danger, r0 r1 r2 r3 r4 r5

fileName = 'fmr_swarm.txt'; % filename in JSON extension
fid = fopen(fileName); % Opening the file
raw = fread(fid,inf); % Reading the contents
str = char(raw'); % Transformation
fclose(fid); % Closing the file
jsondata = jsondecode(str); % Using the jsondecode function to parse JSON from string
automaton = jsondata.nodes;

state_labels = fieldnames(automaton);

% initialize
curr_label = state_labels{1};
next_label = curr_label;
inter_label = curr_label;

% ENV input
alarm = false;
danger = false;

% % preset waypoints
% waypoints = [2.5 7.5;       % corresponding to region r0 to r5
%     7.5 7.5;
%     12.5 7.5;
%     2.5 2.5;
%     7.5 2.5;
%     12.5 2.5];

max_step = 1000;  % max steps
dt = 0.1; % step size
step = 1;


% input prompt
prompt = 'Env signal(a for alarm, d for danger, ad for both, n for both off): ';

while(step < max_step)
    %% read change in ENV signal
    %     str = input(prompt,'s');
    t = timer('ExecutionMode', 'singleShot', 'StartDelay',0.1, 'TimerFcn', @pressEnter);
    start(t);
    str = input(prompt, 's');
    stop(t);
    delete(t);
    if length(str) <=2 && ~isempty(str)
        if str == 'a'
            alarm = true;
            danger = false;
            subtitle('alarm on','fontSize',fontSize);
            
            % plot target region r2
            g_region(1).XData = [10 15];
            g_region(1).YData = [10 10];
            g_region(2).XData = [10 15];
            g_region(2).YData = [5 5];
            g_region(3).XData = [10 10];
            g_region(3).YData = [5 10];
            g_region(4).XData = [15 15];
            g_region(4).YData = [5 10];
        elseif str == 'd'
            alarm = false;
            danger = true;
            subtitle('danger on','fontSize',fontSize);
            % plot target region r5
            g_region(1).XData = [10 15];
            g_region(1).YData = [5 5];
            g_region(2).XData = [10 15];
            g_region(2).YData = [0 0];
            g_region(3).XData = [10 10];
            g_region(3).YData = [0 5];
            g_region(4).XData = [15 15];
            g_region(4).YData = [0 5];
        elseif str == 'ad'
            alarm = true;
            danger = true;
            subtitle('alarm & danger on','fontSize',fontSize);
            % plot target region r5
            g_region(1).XData = [10 15];
            g_region(1).YData = [5 5];
            g_region(2).XData = [10 15];
            g_region(2).YData = [0 0];
            g_region(3).XData = [10 10];
            g_region(3).YData = [0 5];
            g_region(4).XData = [15 15];
            g_region(4).YData = [0 5];
        elseif str == 'n'
            alarm = false;
            danger = false;
            subtitle('both off','fontSize',fontSize);
            g_region(1).XData = [0 5];
            g_region(1).YData = [10 10];
            g_region(2).XData = [0 5];
            g_region(2).YData = [5 5];
            g_region(3).XData = [0 0];
            g_region(3).YData = [5 10];
            g_region(4).XData = [5 5];
            g_region(4).YData = [5 10];
        end
    end
    
    %% State Monitor
    %          check if the goal prop has been reached,if wrong, roll back to previous state
    goal_prop = find(automaton.(next_label).state(3:8)')-1;
    
    temp_prop = zeros(1,n_robots);
    for k = 1:n_robots
        temp_x = poses(k,1);
        temp_y = poses(k,2);
        if temp_y>=0 && temp_y<=5
            if temp_x <= 5
                temp_prop(k) = 3;
            elseif temp_x > 5 && temp_x <=10
                temp_prop(k) = 4;
            else
                temp_prop(k) = 5;
            end
        elseif temp_y<=10
            if temp_x <= 5
                temp_prop(k) = 0;
            elseif temp_x > 5 && temp_x <=10
                temp_prop(k) = 1;
            else
                temp_prop(k) = 2;
            end
        end
    end
    
    
    if ~(isempty(find(~ismember(goal_prop,temp_prop),1)) && isempty(find(~ismember(temp_prop,goal_prop),1)))
        %       if goal prop hasn't been reached, determine which state the system is in
        init_prop = find(automaton.(curr_label).state(3:8)')-1;
        if ~(isempty(find(~ismember(init_prop,temp_prop),1)) && isempty(find(~ismember(temp_prop,init_prop),1)))
            % the system is in the intermediate state
            curr_label = inter_label;
            %             else
            % the system is in the first state of the transition
            % nothing changes
            %                 curr_label = inter_label;
            %                 inter_label = inter_label;
            %                 next_label = inter_label;
        end
    else
        % the goal prop is true, update curr\inter\next\
        curr_label = next_label;
    end
    
    %% executing state machine
    
    % find next state given inputs
    [inter_label,new_next_label,system_liveness] = searchForNextState(automaton,curr_label,alarm,danger)
    
    if ~system_liveness     % failed to find next state
        disp('System failed');
        break;
    end
    
    next_region = find(automaton.(next_label).state(3:8),1)-1
    new_next_region = find(automaton.(new_next_label).state(3:8),1)-1
    next_label = new_next_label;
    %         if ~strcmp(next_label,new_next_label)
    if next_region ~= new_next_region
        %             next_label = new_next_label;
        % if next new state, execute automaton by assign new goal points
        %             next_node = automaton.(next_label);
        %             if ~isIntermediateState(next_node)
        next_region = new_next_region;
        goals = randomPoses(n_robots,next_region);
        %             else
        %                 disp('Error, goal state is intermediate state');
        %                 break;
        %             end
        % if same goal state, keep the same goal points
    end
    % update the goal point and control for each robot
    for ii = 1:n_robots
        % goal points and pose of each robot
        Px = goals(ii,1); Py = goals(ii,2);
        Rx = poses(ii,1); Ry = poses(ii,2); theta = poses(ii,3);
        
        g_fig(ii).XData =  Px;
        g_fig(ii).YData = Py;
        g_link(ii).XData = [Rx,Px];
        g_link(ii).YData = [Ry,Py];
        
        %% control robot to go to the goal point
        % feedback linearization from AMR
        
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
        poses(ii,1) = cmdV*dt*cos(theta)+Rx;
        poses(ii,2) = cmdV*dt*sin(theta)+Ry;
        poses(ii,3) = theta+cmdW*dt;
        
    end
    % plot & update robot pose
    delete(r_body);
    delete(r_dir);
    [r_body,r_dir] = drawRobots(poses,r);
    legend([wall,virtual_wall,g_region(1),g_fig(1),g_link(1),r_dir(1)],{'wall','border','target region','goals','assignment','robot'},'fontSize',fontSize*3/5);
    
    
    
    pause(dt/2);
    

        frame = getframe(gif);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
%     Write to the GIF File
        imwrite(imind,cm,filename,'gif','DelayTime',0.1,'WriteMode','append');
    
%         if step == 0
%             imwrite(imind,cm,filename,'gif','DelayTime',0.1, 'Loopcount',inf);
%         else
%             imwrite(imind,cm,filename,'gif','DelayTime',0.1,'WriteMode','append');
%         end
    
    step=step+1;
end





function [r_body,r_dir] = drawRobots(poses,r, r_body, r_dir)
[n,~] = size(poses);

for i = 1:n
    x = poses(i,1);
    y = poses(i,2);
    th = poses(i,3);
    
    % draw circle with origion [x,y] and radius r
    r_body(i) = rectangle('Position',[x-r,y-r,2*r,2*r],'Curvature',[1,1]);
    axis equal
    % draw a directional line showing robot's direction
    hold on;
    r_dir(i) = quiver(x,y,2*r*cos(th),2*r*sin(th),'k');
end
end



function pressEnter(HObj, event)
import java.awt.*;
import java.awt.event.*;
rob = Robot;
rob.keyPress(KeyEvent.VK_ENTER)
rob.keyRelease(KeyEvent.VK_ENTER)
end


function poses = randomPoses(n,region)
% return n random positions inside given region

if region >= 0 && region <= 2
    xlim = [1 4]+ 5*region;
    ylim = [6 9];
elseif region >=3 && region <= 5
    xlim = [1 4]+ 5*(region-3);
    ylim = [1 4];
else
    disp('wrong region input');
end

poses = zeros(n,2);
for i = 1:n
    poses(i,1) = rand*(xlim(2)-xlim(1))+xlim(1);
    poses(i,2) = rand*(ylim(2)-ylim(1))+ylim(1);
end
end

function bool = isIntermediateState(node)

system_prop = find(node.state(3:8)')-1;

bool = length(system_prop) ==2;
end


function [inter_label,next_label,system_liveness] = searchForNextState(automaton,curr_label,alarm,danger)
% check next possible states
state_labels = fieldnames(automaton);
next_state_labels = state_labels(automaton.(curr_label).trans +1);
system_liveness = false;
for i=1:length(next_state_labels)
    temp_label = next_state_labels{i};
    temp_node = automaton.(temp_label);
    temp_a = temp_node.state(1);
    temp_d = temp_node.state(2);
    if alarm == temp_a && danger == temp_d  % if next state match the ENV signal
        if isIntermediateState(temp_node)
            % if next state is an intermediate state
            inter_label = temp_label;
            % check beyond this intermediate state look for next one
            [~,next_label,system_liveness]=searchForNextState(automaton,temp_label,alarm,danger); % recursively search for next state with one sys prop true
        else
            % next state is a state with one system prop
            next_label = temp_label;
            inter_label = curr_label;
            system_liveness = true;
            break;  % next  state is found
        end
    end
end
end
