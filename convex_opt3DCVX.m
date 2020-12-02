%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% @Author Nick Johnson
% Co-Authors: Steve Hannah, Bri Meyer, Cullen Dahleen
% AAE561 Final Project Optimization code
% Constrained Minimization Scheme from Ch.2 Notes Slides 55-60
% Sub-problems solved using fmincon directly
% Known bugs:
%       Bisection searchh in tstar function does not always properly
% converge due to 'minimum step size tolerance' of fmincon.  Bounds of 
% search become closer than the minimum step size tolerance so the search
% hangs when close to the optimal solution.  
% Workarounds:
%       if statement checking if the bounds are equal (meaning the search 
% is stuck) breaks from the loop to allow the algorithm to continue.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% inputs:   path0 = nx3 matrix of points representing the 3D path
%           centers = nx3 matrix of points representing the centers of
%           bubble constraints, often will be equal to x0
%           radii = 1xn vector of radii for spherical bubble constraints,
%           correstponding to centers matrix
% outputs:  xstar = 3xn
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function xstar = convex_opt3DCVX(path0,centers,radii)
%% Define Problem Parameters
global p A C R L mu n T stop tmax
p = path0';     % transpose nx3 -> 3xn
A = centers';   % transpose nx3 -> 3xn
r = radii;
% Remove endpoints from the path being optimzed.  The endpoints are fixed
% and should not be moved, hence we remove them and their associated
% constraints (which are automatically generated but not accurate)
x0 = p(:,2:(end-1));
C = A(:,2:(end-1));
R = radii(:,2:(end-1));
n = length(x0);                 % number of points in path excluding ends
x0 = x0(:);                     % transforms x0 from 3xn matrix to 3nx1 vector
% We transform the matrix of points into a 3nx1 vector so that the
% objective function and constraints can be represented by vector
% functions.  The transformation is done by:
% x0=[x1 x2 x3...]  ->  x0 = [x1 y1 z2 x2 y2 z2 x3 y3 z3...]
%    [y1 y2 y3...]
%    [z1 z2 z3...]
T = zeros(3,3*n,n);
% Create T tensor (3 x 3n x n) used for objective function
for i = 1:n-1
    Ti= zeros(3,3*n);
    Ti(:,(3*(i-1)+1):(3*(i-1)+3)) = -1.*eye(3);
    Ti(:,(3*(i-1)+4):(3*(i-1)+6)) = eye(3);
    T(:,:,i) = Ti;
end
% Compute Hessian of f0 which is sum from =1 to i=n-1 of 2*Ti'*Ti
H0 = 0;
for i = 1:(n-1)
    Hi = 2*T(:,:,i)'*T(:,:,i);
    H0 = H0 + Hi;
end
% Compute Hession of f1 which is equal to 2*eye(3)
H1 = 2.*eye(3);
K = 0.25;                           % Kappa used for inner loop termination
L = max(norm(H0),norm(H1));
mu = min(norm(H0),norm(H1));
tmax = 0;       % used in bisection search as upper limit of search
% calculated as function value of initial path since this is guaranteed to
% less than the optimal function value and t* is equal to optimal function
% value
for q = 2:length(p)
    tmax = tmax+ norm(p(:,q)-p(:,q-1))^2;
end
B = (sqrt(L)-sqrt(mu))/(sqrt(L)+sqrt(mu));  % beta, used for inner loop
k = 0;                          % initialize outer loop iteration counter
stop = 1e-3;                    % stopping condition sufficiently small
xk(:,1) = x0;                   % initialize xk sequence
yj = xk;                        % initialize yk for sequence generation
t0 = -1e-3;                     % initial t, negative to ensure t<t*
t = t0;
done = 0;                       % outer loop termination flag

%% Main Algorithm/Scheme
while(~done)    % outer loop
    k = k+1;
    % The following lines are to initialize histories of xf and f* values
    % in terms of L.  The values in the histories will correspond to
    % each j index of the xk sequence.  These are useful to have so that ex
    [xfL,fsL] = fstar(t,xk(:,1),L);
    [xfM,fsM] = fstar(t,xk(:,1),mu);
    fsLhist = [];
    fsLhist(1) = fsL;
    xfLhist = [];
    xfLhist(:,1) = xfL;
    j = 0;
    while(fsM <= (1-K)*fsL)     % inner loop
        j = j+1;
        [xk_jnew,fsnew] = fstar(t,yj,L);        % xk_j+1 next sequence val
        yjnew = xk_jnew + B.*(xk_jnew - xk(:,j));   %y_j+1
        yj = yjnew;
        xknew = [xk xk_jnew];   % append xk_j+1 to xk sequence
        xk = xknew;     % update xk sequence
        % The following lines compute xf and f* for L and store them
        % in the history vectors to correspond with xk
        [xfL,fsL] = fstar(t,xk_jnew,L);
        [xfM,fsM] = fstar(t,xk_jnew,mu);
        fsLhistNew = [fsLhist fsL];
        fsLhist = fsLhistNew;
        xfLhistNew = [xfLhist xfL];
        xfLhist = xfLhistNew;
        
        if(fsL < stop)      % Global stop criteria
            done = 1;       % outer loop stop flag
            break
        end
    end
    % Histories of xf and f* are indexed to compute j* and to
    % determine the starting x value of the next sequence xk+1
    jstar = find(fsLhist==min(fsLhist));
    xnew = xfLhist(:,jstar);
    xk = xnew;
    if(done); break; end        % break from outer loop
    t = tstar(xfLhist(:,end),t);    % update t using t* function
end

%ouptut:
xstar = reshape(xk,3,n);
xstar = [p(:,1) xstar p(:,end)];
return
end

%% Sub-problems and supporting functions

% f* function. since xf and f* are minimizing the same problem, this
% function outputs both values using fmincon
function [xf,fs] = fstar(tk,xkj,gam)
global n
% cvx solver used
cvx_begin quiet
    variable x(3*n,1)
    minimize(fstar_cost(tk,xkj,gam,x))
cvx_end
fs = cvx_optval;
xf = x;
end

% Objective function used in fmincon, effectively f_gamma from slide 55.
% Here, f(t,x,g) + g/2*norm(xbar-x)^2 is combined such that the g/2 term is
% added to each element of the max-type function and then the maximum is
% taken.  
function func = fstar_cost(tk,xkj,gam,x)
G1 = g0(tk,xkj,x) + gam/2*pow_pos(norm(x-xkj),2);
G2 = g1(xkj,x) + gam/2*pow_pos(norm(x-xkj),2);
G = [G1 G2];
func = max(G);
return
end

% t update function.  Root search for f*_mu via bisection method
function ts = tstar(x,t)
global mu stop tmax
% Bisection search
a = t;
b = tmax;
c= (a+b)/2;
[xf,fs] = fstar(c,x,mu);

while(abs(fs) > stop)
    if(fs < 0)
        b = c;
        c = (a+b)/2;
        [xf,fs] = fstar(c,x,mu);
    else
        a = c;
        c = (a+b)/2;
        [xf,fs] = fstar(c,x,mu);
    end
   if(a==b) % see header for bug notes
       break;
   end
end
ts = c;
return
end

%% Objective Function and Functional Constraints
% all functions are analytically coded in the forms required for f(t,xx,g)
% used for computing f*
function f = g0(t,xkj,x)
global n T p
grad=zeros(3*n,1);
%cost function is total distance of path xkj
% since xkj does not include the endpoints, we must include the distance
% from p(1) to xkj(1) and xkj(end) to p(end)
dist = norm([xkj(1);xkj(2);xkj(3)]-p(:,1))*norm([xkj(1);xkj(2);xkj(3)]-p(:,1));
dist = dist + norm(p(:,end)-[xkj(end-2);xkj(end-1);xkj(end)])*norm(p(:,end)-[xkj(end-2);xkj(end-1);xkj(end)]);

for i=1:n-1
    temp_dist = norm(T(:,:,i)*xkj)*norm(T(:,:,i)*xkj); % ||Ai*x||^2
    dist = dist+temp_dist;
    
    temp_grad = 2.*(T(:,:,i)'*T(:,:,i)*xkj);    % Ai'*Ai*x
    grad = grad+temp_grad;
end

f = dist + grad'*(x-xkj) - t; 
return
end

function f1 = g1(xkj,x)
global C R n
% contraint is that ||xkj-C||^2-R^2 <= 0
% NOTE Technically we have n total function constraints here, one for each
% bubble.  Since this function will return a vector, the
% subproblem objective function will concatenate the cost with this vector
% and take the max of all of them.
dist = cvx(zeros(1,n));
grad = cvx(zeros(1,n));
% It is easier to compute the distance between consecutive points with the
% xkj and decision variable (x) in matrix form of 3xn so they are converted
% from 3nx1 vectors to 3xn matrices
xkjM = reshape(xkj,3,n);
xM = reshape(x,3,n);
for i=1:n
    temp_dist = norm(xkjM(:,i)-C(:,i))^2 - R(i)^2;      % ||x-C||^2-R^2 <=0
    dist(i) = temp_dist;
    
    temp_grad = 2.*(xkjM(:,i)-C(:,i));          % 2*(x-C) = gradient
    grad(i) = temp_grad'*(xM(:,i)-xkjM(:,i));
end
f1 = (dist) + grad;
return
end
