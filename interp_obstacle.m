function vec = interp_obstacle(obstacle,rho)

X = obstacle(1,:);
Y = obstacle(2,:);
Z = obstacle(3,:);
vec = [];
for z = Z
    for x = X
        yvec = linspace(Y(1),Y(2),rho)';
        yvec = [x.*ones(rho,1),yvec,z.*ones(rho,1)];
        vec = [vec;yvec];
    end
    for y = Y
        xvec = linspace(X(1),X(2),rho)';
        xvec = [xvec,y.*ones(rho,1),z.*ones(rho,1)];
        vec = [vec;xvec];
    end
end

for x = X
    for y=Y
        zvec = linspace(Z(1),Z(2),rho)';
        zvec = [x.*ones(rho,1),y.*ones(rho,1),zvec];
        vec = [vec;zvec];
    end 
end