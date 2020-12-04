%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @Author Nick Johnson
% Function to comput length of path
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dist = path_length(p)
dist = 0;
for q = 2:length(p)
    dist = dist + norm(p(:,q)-p(:,q-1))^2;
end
return
end