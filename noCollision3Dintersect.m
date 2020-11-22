function nc = noCollision3Dintersect(A, B, o, buffer)

nc = 1; 
% Define min and max lwh for prism
minx = o(1,1)-buffer;
maxx = o(1,2)+buffer;
miny = o(2,1)-buffer;
maxy = o(2,2)+buffer;
minz = o(3,1)-buffer;
maxz = o(3,2)+buffer;

x = linspace(A(1),B(1),1e3);
% discretize along line, look for intersection with prism
slope_y = (B(2)-A(2))/(B(1)-A(1));
slope_z = (B(3)-A(3))/(B(1)-A(1));
for n = 1:length(x)
    % calculate y and z for discretized x
    y = slope_y*(x(n)-A(1))+A(2);
    z = slope_z*(x(n)-A(1))+A(3);
    % Check if new point is inside the prism, if so a collision is imminent
    if x(n)>minx && x(n)<maxx
        if y>miny && y<maxy
            if z>minz && z<maxz
                nc = 0;
                break
            end
        end
    end

end

end