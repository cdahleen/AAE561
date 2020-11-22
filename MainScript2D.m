%% Steve Hannah
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

% Call function and recieve the 2D path
[path] = RRTStar2D(xlim, ylim, start, goal, obstacle, stepsize, numNodes);