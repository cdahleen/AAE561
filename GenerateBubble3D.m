function [A_temp,r_temp] = GenerateBubble3D(P,obstacle,obstacle_bounds)
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
%     A_temp = P;
%     r_temp = M;
    A_temp  = P + (rl-M)*norm(P - obstacle(I,:));
    r_temp = rl;
end
%%do cube calcs
xmnin = A_temp(1) - r_temp;
xmax = A_temp(1) + r_temp;
ymnin = A_temp(2) - r_temp;
ymax = A_temp(2) + r_temp;
xmnin = A_temp(3) - r_temp;
xmax = A_temp(3) + r_temp;

intersection = 1;
while intersection == 1
    xmin = A_temp(1) - r_temp;
    xmax = A_temp(1) + r_temp;
    ymin = A_temp(2) - r_temp;
    ymax = A_temp(2) + r_temp;
    zmin = A_temp(3) - r_temp;
    zmax = A_temp(3) + r_temp;
    if (xmin < obstacle_bounds(1,2) && xmax > obstacle_bounds(1,1))
        if (ymin < obstacle_bounds(2,2) && ymax > obstacle_bounds(2,1))
            if (zmin < obstacle_bounds(3,2) && zmax > obstacle_bounds(3,1))
                r_temp = r_temp - .25;
            else
                intersection = 0;
            end
        else 
            intersection = 0;
        end
    else
        intersection = 0;
    end
end

            
 
    
