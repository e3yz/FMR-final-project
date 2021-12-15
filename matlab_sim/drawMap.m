function wall = drawMap(map,plotOpts)
% DrawMap: this function draw the walls of the map
%
%    INPUTS
%               map     Nx4 matrix, which contains the coordinates of
%                       all walls in the environment: [x1; y1; x2; y2].
%    OUTPUTS
%               plot contains the walls of the map
%   Cornell University
%   Autonomous Mobile Robots
%   Zhu, Yihan

hold on
axis equal
for i=1:length(map(:,1))
    wall(i)=plot([map(i,1),map(i,3)],[map(i,2),map(i,4)],'k','linewidth',5);
end

if exist('plotOpts','var')
    % Set plot options
    Nopts = numel(plotOpts)/2;
    plotOpts = reshape(plotOpts,2,Nopts)';
    for n = 1:Nopts
        set(wall,cell2mat(plotOpts(n,1)),cell2mat(plotOpts(n,2)));
    end
end

wall = wall(1);
end