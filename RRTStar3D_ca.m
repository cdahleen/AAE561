function [path, interpolated_path] = RRTStar3D_ca(x_max, y_max, z_max, start, goal, obs, buffer, stepsize, numNodes, rho)

% Set the starting point and final destination
q_start.coord = start;
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = goal;
q_goal.cost = 0;

% Initial the nodes array as starting point
nodes(1) = q_start;
figure(1)

% Plot obstacle
plot3(0,0,0)
hold on
patch([obs(1,1) obs(1,1) obs(1,2) obs(1,2)], ...
    [obs(2,1) obs(2,1) obs(2,1) obs(2,1)], ...
    [obs(3,1) obs(3,2) obs(3,2) obs(3,1)], 'blue')
patch([obs(1,1) obs(1,1) obs(1,2) obs(1,2)], ...
    [obs(2,1) obs(2,2) obs(2,2) obs(2,1)], ...
    [obs(3,2) obs(3,2) obs(3,2) obs(3,2)], 'blue')
patch([obs(1,1) obs(1,1) obs(1,2) obs(1,2)], ...
    [obs(2,2) obs(2,2) obs(2,2) obs(2,2)], ....
    [obs(3,2) obs(3,1) obs(3,1) obs(3,2)], 'blue')
patch([obs(1,1) obs(1,1) obs(1,2) obs(1,2)], ...
    [obs(2,1) obs(2,2) obs(2,2) obs(2,1)], ...
    [obs(3,1) obs(3,1) obs(3,1) obs(3,1)], 'blue')
patch([obs(1,1) obs(1,1) obs(1,1) obs(1,1)], ...
    [obs(2,1) obs(2,2) obs(2,2) obs(2,1)], ...
    [obs(3,2) obs(3,1) obs(3,2) obs(3,1)], 'blue')
patch([obs(1,2) obs(1,2) obs(1,2) obs(1,2)], ...
    [obs(2,2) obs(2,1) obs(2,1) obs(2,2)], ...
    [obs(3,1) obs(3,2) obs(3,2) obs(3,1)], 'blue')

for i = 1:1:numNodes
    % Sample a random point to move towards by step size EPS
    q_rand = [rand(1)*x_max rand(1)*y_max rand(1)*z_max];
    
    % Plot the random point as an X
    plot3(q_rand(1), q_rand(2), q_rand(3), 'x', 'Color',  [0 0.4470 0.7410])
    
    % Break if goal node is already reached
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_goal.coord
            % Set notification when goal is found
            fprintf('goal found')
            break
        end
    end
    
    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist_3d(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);
    
    % Steer towards the random point and add a new point to the path
    q_new.coord = steer3d(q_rand, q_near.coord, val, stepsize);
    % If no obstacle, draw and calculate the cost
    %if noCollision3D(q_new.coord, obs, buffer)
    if noCollision3Dintersect(q_near.coord, q_rand, obs, buffer)
        line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], [q_near.coord(3), q_new.coord(3)], 'Color', 'k', 'LineWidth', 2);
        drawnow
        hold on
        q_new.cost = dist_3d(q_new.coord, q_near.coord) + q_near.cost;
        
        % Within a radius of r, find all existing nodes
        q_nearest = [];
        r = 10;
        neighbor_count = 1;
        for j = 1:1:length(nodes)
            if (dist_3d(nodes(j).coord, q_new.coord)) <= r
                q_nearest(neighbor_count).coord = nodes(j).coord;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                neighbor_count = neighbor_count+1;
            end
        end
        
        % Initialize cost to currently known value
        q_min = q_near;
        C_min = q_new.cost;
        
        % Iterate through all nearest neighbors to find alternate lower
        % cost paths
        
        for k = 1:1:length(q_nearest)
            if q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord) < C_min
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord);
                %line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], [q_min.coord(3), q_new.coord(3)], 'Color', 'g');
                hold on
            end
        end
        
        % Update parent to least cost-from node
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
        end
        
        % Append to nodes
        nodes = [nodes q_new];
    end
end

D = [];
for j = 1:1:length(nodes)
    tmpdist = dist_3d(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];
while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], [q_end.coord(3), nodes(start).coord(3)], 'Color', 'r', 'LineWidth', 4);
    hold on
    q_end = nodes(start);
end

% Build Path
path(1,:) = nodes(end).coord;
i = 2;
p = nodes(end).parent;
while p ~= 0
    path(i,:) = nodes(p).coord;
    p = nodes(p).parent;
    i = i+1;
end

% Interpolate Path
interpolated_path = [];
for n = 2:length(path)
    x = linspace(path(n-1,1),path(n,1),rho);
    slope_y = (path(n,2)-path(n-1,2))/(path(n,1)-path(n-1,1));
    slope_z = (path(n,3)-path(n-1,3))/(path(n,1)-path(n-1,1));
    for k = 1:length(x)
        y(k) = slope_y*(x(k)-path(n-1,1))+path(n-1,2);
        z(k) = slope_z*(x(k)-path(n-1,1))+path(n-1,3);
    end
    xyz_matrix = [x; y; z];
    interpolated_path = [interpolated_path; xyz_matrix'];
end

end
