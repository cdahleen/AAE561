%% Steve Hannah
% Main file to interface with 3D RRT Star Algorithm
tic;
% Main Script
% clearvars -except k xstarDiffs xstarErrs rrtPathDists xstarDists elapsed_times
% close all
% clc

% Set Graph Size
xlim = 100;
ylim = 100;
zlim = 100;
% Set Starting and Ending Positions
start = [0 0 0];
goal = [50 50 50];
% Set the Number of Nodes and Step Size
% (Note: if you don't have enough nodes or too small of a step size
%        you are at risk of colliding the obstacle)
numNodes = 500;
stepsize = 1;

% Define the Obstacle - This is hardcoded, do not change
% (length, width, and height of a prism)
% [xrange; yrange; zrange]
obstacle = [10 30; 10 40; 10 20];
obstacle2 = [10,10,10;10,40,10;30,40,10;30,10,10;30,10,20;10,10,20;10,40,20;30,40,20];
buffer = 4;
% Define rho for interpolated path
rho = 3;
% Call function and recieve the 3D path
[path, P] = RRTStar3D_ca(xlim, ylim, zlim, start, goal, obstacle, buffer, stepsize, numNodes, rho);
savedir = [pwd '/figs/RRT.jpg'];
saveas(figure(1),savedir)

global rl ru
%%start bubble algorithim by defining constants

rl = 2;                    %%Lower bound of sphere radius 
ru = 5;                     %% upper bound of circular radius
r = ones(length(P),1);
%%initialize bubble centers at points P
A = P;     
%%interpolate object for edge detection
rho_obs = 4;
high_rho_obstacle = interp_obstacle(obstacle,rho_obs);

%%initialize starting bubbles
A(1,:) = P(1,:);
r(1) = ru;
%%refine bubbles
for i = 2:length(P)-1
    [A_temp,r_temp] = GenerateBubble3D(P(i,:),high_rho_obstacle,obstacle);
    A(i,:) = A_temp;
    r(i) = r_temp;
end

% optimize path
xstar = convex_opt3DCVX(A,A,r);

%%plot spheres after bubble algorithim
[X,Y,Z] = sphere;

figure(2)
for n = 1:length(A)
    X1 = X*r(n);
    Y1 = Y*r(n);
    Z1 = Z*r(n);
    surf(X1+A(n,1),Y1+A(n,2),Z1+A(n,3),'FaceAlpha',0.1,'EdgeColor','none')
    hold on;
end

s1 = plot3(A(:,1),A(:,2),A(:,3),'--','DisplayName','RRT Path');    %plot original path from RRT
s2 = plot3(xstar(1,:),xstar(2,:),xstar(3,:),'-*','DisplayName','Optimized Output');    %plot optimized path
s3 = plot3([0 10 50],[0 10 50], [0 20 50],'DisplayName','True optimal');
patch([obstacle(1,1) obstacle(1,1) obstacle(1,2) obstacle(1,2)], ...
    [obstacle(2,1) obstacle(2,1) obstacle(2,1) obstacle(2,1)], ...
    [obstacle(3,1) obstacle(3,2) obstacle(3,2) obstacle(3,1)], 'blue')
patch([obstacle(1,1) obstacle(1,1) obstacle(1,2) obstacle(1,2)], ...
    [obstacle(2,1) obstacle(2,2) obstacle(2,2) obstacle(2,1)], ...
    [obstacle(3,2) obstacle(3,2) obstacle(3,2) obstacle(3,2)], 'blue')
patch([obstacle(1,1) obstacle(1,1) obstacle(1,2) obstacle(1,2)], ...
    [obstacle(2,2) obstacle(2,2) obstacle(2,2) obstacle(2,2)], ....
    [obstacle(3,2) obstacle(3,1) obstacle(3,1) obstacle(3,2)], 'blue')
patch([obstacle(1,1) obstacle(1,1) obstacle(1,2) obstacle(1,2)], ...
    [obstacle(2,1) obstacle(2,2) obstacle(2,2) obstacle(2,1)], ...
    [obstacle(3,1) obstacle(3,1) obstacle(3,1) obstacle(3,1)], 'blue')
patch([obstacle(1,1) obstacle(1,1) obstacle(1,1) obstacle(1,1)], ...
    [obstacle(2,1) obstacle(2,2) obstacle(2,2) obstacle(2,1)], ...
    [obstacle(3,2) obstacle(3,1) obstacle(3,2) obstacle(3,1)], 'blue')
patch([obstacle(1,2) obstacle(1,2) obstacle(1,2) obstacle(1,2)], ...
    [obstacle(2,2) obstacle(2,1) obstacle(2,1) obstacle(2,2)], ...
    [obstacle(3,1) obstacle(3,2) obstacle(3,2) obstacle(3,1)], 'blue')
title('Optimization Results'); legend([s1,s2 s3])
savedir = [pwd '/figs/Post_Bubble.jpg'];
saveas(figure(2),savedir)

rrtPathDist = path_length(A');
xstarDist = path_length(xstar);
trueOpt = norm([10 10 20])+norm([40 40 30]);

xstarDiff = ((rrtPathDist-xstarDist)/rrtPathDist)*100;
xstarErr = ((xstarDist-trueOpt)/trueOpt)*100;
fprintf('Length of RRT path: %f \nLength of xStar path: %f\n',rrtPathDist,xstarDist)
fprintf('Percent difference of xstar vs RRT: %f\n',xstarDiff)
fprintf('Percent error of xstar vs true optimal distance %f\n',xstarErr)
elapsed_time=toc;