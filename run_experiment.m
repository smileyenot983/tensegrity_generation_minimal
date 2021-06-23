function solution = run_experiment(seed,n_nodes,strut_constraint,strut_max_length,cable_constraint,cable_max_length,...
    projection_constraint,projection_axis,projection_max_length)
%RUN_EXPERIMENT Summary of this function goes here
%   Detailed explanation goes here
% this function takes as input generation parameters and outputs struct with 3 parameters:
% C - cable connectivity matrix, R - strut connectivity matrix, points -
% coordinates of nodes

% INPUTS:
% 1.seed - seed to avoid randomness in chosen points(int)
% 2.n_nodes - number of nodes, used in generation(int)
% 3.strut_constraint, cable_constraint, projection_constraint - whether to
%   use corresponding constraint or not(boolean)
% 4.strut_max_length, cable_max_length, projection_max_length - maximum
%   length for corresponding constraint(int or float)
% 5.projection_axis - axis of constrained direction


rng(seed);

%% GENERATING SET OF CANDIDATE POINTS(SPHERE)
% Define the number of points to place on the surface of the sphere.
numPoints = 500;
radius = 10;
% Get a 3-by-numPoints list of (x, y, z) coordinates.
r = randn(3, numPoints);
% At this point the points can be anywhere in 3D space,
% not just on the surface of a sphere, a shell of constant radius.
% Divide by the distance of each point from the origin.
% This will place each point out at a definite radius of 1, not scattered throughout the volume.
r = bsxfun(@rdivide, r, sqrt(sum(r.^2,1)));
% Now multiply by radius to make the shell out 
% at a specified radius instead of a radius of exactly 1
points = radius * r;

% after building grid of some shape we choose n points in this grid, to
% generate tensegrity
n = size(points,2);
m = 3;
% n_nodes= 20;
available_slots = 1:n;

random_permutation = randperm(numel(available_slots));

chosen_slots = random_permutation(1:n_nodes);

chosen_points = points(:,chosen_slots);
n = n_nodes;
points = chosen_points;

% saving vectors, describing connection between 2 nodes
Dir = zeros(n, n, 3);
for i = 1:n
    for j = 1:n
        
        Dir(i, j, :) = reshape( (points(:, i) - points(:, j)), [1, 1, 3] );
        
    end
end
Dir_t = permute(Dir, [3, 2, 1]);


%% OPTIMIZATION PROBLEM FORMULATION

% gravity force
external_force = repmat([0;0;-9.8], [1, n]);    


big_M = 10;

cvx_begin

% connectivity matrices
variable R(n, n) binary
variable C(n, n) binary

% force between 2 nodes
variable f(n, n)
% force that keeps m nodes fixed(m=3)
variable g(3, m)

% binary variables used in MICP
variable c_1(n) binary
variable c_2(n) binary

variable c_3 binary
variable c_4 binary

% minimize total amount of cable connection
minimize( sum(C(:)) )
subject to

% constraint on symmetricity
    C == C';
    R == R';
    
%     constraint to avoid self-connections
    diag(R) == zeros(n, 1);
    diag(C) == zeros(n, 1);
    
%     to make 3 cable connections from each node
    sum(C,1) == 3*ones(1,size(C,1))
    
%     to ensure that every node has exactly 1 strut connection
    sum(R, 1) == ones(1, n);
    sum(R, 2) == ones(n, 1);
    
%     to ensure that there is either cable, either strut, either no
%     connection
    P = C+R;
    P(:) <= ones(n*n, 1);
    
%     !!!increasing number of cables!!!
%     sum(P(:)) >= 45;
    
%    big M relaxation, force f- force between nodes, produced by cables and
%    struts
    for i = 1:n
        for j = 1:n
           f(i, j) <= big_M * C(i, j);
          -f(i, j) <= big_M * R(i, j);
        end
    end
    
% g - force which keeps 3 nodes fixed, other nodes do not have that force
    g_ext = [g, zeros(3, n-m)];
    
%     static stability constraint
    for i = 1:n 
        Dir_t(:, :, i) * f(i, :)' == external_force(:, i) + g_ext(:, i);
    end
    
%     constraint on max length of strut
    if strut_constraint==true
        for i=1:n        
            norm(sum(  repmat(R(i,:),3,1).*points,2  )-points(:,i)) <=strut_max_length;
        end
    end
    
%     constraint on max length of cable
    if cable_constraint==true
        for i=1:n
            for j = 1:n
                norm((points(:,i)-points(:,j)).*repmat(C(i,j),3,1)) <= cable_max_length;
            end
        end
    end

    x = [1;0;0];
    y = [0;1;0];
    z = [0;0;1];
    % constraint for restricting max projection in given direction
    
    if projection_constraint == true
    
        if projection_axis=='x'
            dir=x;
        elseif projection_axis =='y'
            dir=y;
        elseif projection_axis =='z'
            dir=z;
        elseif projection_axis =='xy'
            dir=[x,y];
        else
            dir=[0;0;0];
        end


    %     l_0 = 2.5;
        for j = 1:size(dir,2)
            for i=1:n
                norm(  (sum(repmat(R(i,:),3,1).*points,2) -points(:,i))'*dir(:,j) ) <= projection_max_length;
            end
        end
    
    end
       
    


cvx_end

C = full(C);
R = full(R);
f = full(f);

disp("sum: ");
disp(sum(P(:)));


%% Saving into a file

solution.points = points;
solution.C = C; 
solution.R = R;

end

