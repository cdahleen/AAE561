function [P] = Interp_Trajectory(P,rho)
%%this code interpolates a 2-D path to a given point density
%from a input
%%trajectory
%
%
%Inputs:
%       P: reference trajectory
%       
%
%
%Ouputs:
%       A:
%       R
%       
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%perform Trajectory Interpolation to a given density
m=1;

while m < length(P)
    if norm(P(m+1,:) - P(m,:)) >= rho
        np = (P(m+1,:) - P(m,:))/2 + P(m,:);
        P = [P(1:m,:);np;P(m+1:end,:)];
        m=1;
    else
        m=m+1;
    end
end
    
