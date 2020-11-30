%% Steve Hannah
% Main file to interface with 3D RRT Star Algorithm

% Main Script
clear all
close all
clc

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

rl = 2;                   %%Max distance between adjacent points I.E rho
ru = 5;                     %% upper bound of circular radius
ro = 4;                     %%initialized bubble size;
r = ones(length(P),1);

r = r*ro;

%%initialize bubbles 
A = P;     
%%start center of bubbles at Points P
[X,Y,Z] = sphere;
X = X*ro;
Y = Y*ro;
Z = Z*ro;
figure(2)
for n = 1:length(P)
    surf(X+A(n,1),Y+A(n,2),Z+A(n,3))
    hold on;
end

%%plot interpolated trajectory and initialized bubbles
plot3(P(:,1),P(:,2),P(:,3),'LineWidth',15)
hold on;
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
savedir = [pwd '/figs/Pre_Bubble.jpg'];
saveas(figure(2),savedir)

%%interpolate object for edge detection
rho_obs = 4;
high_rho_obstacle = interp_obstacle(obstacle,rho_obs);
clearvars A r
A(1,:) = P(1,:);
r(1) = ru;
%%refine bubbles
for i = 2:length(P)-1
%     if norm(P(i,:) - A(i-1,:)) < .5*r(i-1)
%         fprintf('Dont Call generate Bubble \n')
%         A(i,:) = A(i-1,:);
%         r(i) = r(i-1);
%         continue
%     end
    fprintf('Call Generate Bubble \n')
    [A_temp,r_temp] = GenerateBubble3D(P(i,:),high_rho_obstacle);
    A(i,:) = A_temp;
    r(i) = r_temp;
end

%%plot after bubble algorithim
[X,Y,Z] = sphere;

figure(3)
for n = 1:length(A)
    X1 = X*r(n);
    Y1 = Y*r(n);
    Z1 = Z*r(n);
    surf(X1+A(n,1),Y1+A(n,2),Z1+A(n,3))
    hold on;
end
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
savedir = [pwd '/figs/Post_Bubble.jpg'];
saveas(figure(3),savedir)