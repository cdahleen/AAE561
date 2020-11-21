%% Steve Hannah
% Main file to interface with 3D RRT Star Algorithm

% Main Script
clear
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
numNodes = 1500;
stepsize = 1;

% Define the Obstacle - This is hardcoded, do not change
% (length, width, and height of a prism)
% [xrange; yrange; zrange]
obstacle = [10 30; 10 40; 10 20];
buffer = 8;
% Call function and recieve the 3D path
[path] = RRTStar3D_ca(xlim, ylim, zlim, start, goal, obstacle, buffer, stepsize, numNodes);