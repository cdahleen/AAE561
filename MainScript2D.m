%% Steve Hannah + Cullen Dahleen
% Main file to interface with 2D RRT Star Algorithm

% Main Script
clear
close all
clc

% Set Graph Size
xlim = 100;
ylim = 100;

% Set Starting and Ending Positions
start = [0 0];
goal = [50 50];

% Set the Number of Nodes and Step Size
% (Note: if you don't have enough nodes or too small of a step size
%        you are at risk of colliding the obstacle)
numNodes = 500;
stepsize = 1;

% Define the Obstacle 
% (Uses rectangle function [x y w h] where x,y is lower left corner 
%  and w h are width and height)
obstacle = [10, 10, 20, 20];
obstacle2 = [10,10;10,30;30,30;30,10];
obs = polyshape(obstacle2);

% Call function and recieve the 2D path
[P] = RRTStar2D(xlim, ylim, start, goal, obstacle, stepsize, numNodes);
savedir = [pwd '/figs/RRT.jpg'];
saveas(figure(1),savedir)

global ru rl 

%%do interpolation to get desire density of points rho
rho = 2;
P = Interp_Trajectory(P,rho);

%%constants

rl = rho;                   %%Max distance between adjacent points I.E rho
ru = 5;                     %% upper bound of circular radius
r = ones(length(P),1);
r = r*4;

%%initialize bubbles 
A = P;     
%%start center of bubbles at Points P

%%plot interpolated trajectory and initialized bubbles
figure(2)
plot(P(:,1),P(:,2))
hold on;
viscircles(A,r)
hold on;
plot(obs)
savedir = [pwd '/figs/Pre_Bubble.jpg'];
saveas(figure(2),savedir)

%%perform trajectory tunnel optimization
for i = 2:(length(P) - 1)
    %if norm(P(i,:) - (P(i,:)-P(i-1,:))/norm(P(i,:)-P(i-1,:)) * A(i-1)) < .5*A(i-1)
    if norm(P(i,:) - A(i-1,:)) < .5*r(i-1)
        fprintf('Dont call Generate Bubble \n')
        A(i,:) = A(i-1,:);
        r(i) = r(i-1);
        continue 
    end
    fprintf('Call Generate Bubble \n')
    [A_temp,r_temp] = GenerateBubble(P(i,:));
    A(i,:) = A_temp;
    r(i) = r_temp;
end
obstacle2 = [10,10;10,30;30,30;30,10];
obs = polyshape(obstacle2);

%%plot after bubble algorithim
figure(3)
plot(P(:,1),P(:,2))
hold on;
viscircles(A,r)
hold on;
plot(obs)
savedir = [pwd '/figs/Post_Bubble.jpg'];
saveas(figure(3),savedir)