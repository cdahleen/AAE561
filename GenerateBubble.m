function [A,r] = GenerateBubble(P)
%%this function seeks to compute the largest bubble at the given point that
%%is collision free S.T ru and rl
global ru rl
%%this assumes 1 obstacle
obstacle2 = [10,10;10,30;30,30;30,10];
obs = polyshape(obstacle2);
[vertexid,boundaryid,ind] = nearestvertex(obs,P);
distance = norm(P-obstacle2(ind,:));
if distance >= ru
    A = P;
    r = ru;
elseif (distance <= ru) && (distance >= rl)
    A = P;
    r = distance;
elseif distance < rl
    %%this part needs to move the bubble away from obstace up to a certain
    %%amount
    direction = -(P-obstacle2(ind));
    step = rl - distance;
    A = P + direction*step;
    r = rl;
end



