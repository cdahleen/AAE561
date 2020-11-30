function [A_temp,r_temp] = GenerateBubble3D(P,obstacle)
global ru rl

%%find closest edge point on obstacle
for i = 1:length(obstacle)
    distance(i) = norm(obstacle(i,:) - P);
end
[M,I] = min(distance);
if M > ru
    A_temp = P;
    r_temp = ru;
elseif (M < ru) && (M > rl)
    r_temp = M;
    A_temp = P;
elseif M < rl
    A_temp  = P + (rl-M)*norm(P - obstacle(I,:))
    r_temp = rl;
end
    
