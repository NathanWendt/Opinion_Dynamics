function [itters] = itters2conv(jerk_node, capped, Adjacency, Graph)

N = length(Adjacency);
itters = 0;
% SETUP

%%%% VARIABLES

jerk_goal = 0.7;               % control target for the jerk

kp = 1.5;                        % proportional gain

k = 1;                         % discrete time counter
%max_k = 400;                   % max number of iterations
err = ones(k,2);               % errors of [Neighbor; Global]
converge = 0;                  % if converged = 1
% initializing state vector
x = zeros(N,k);                % series of state vector states
for i = 1:N
    x(i,1) = rand();      
end
x(jerk_node,1) = jerk_goal;
Neighbors = neighbors(Graph,jerk_node);   % neighbors of jerk
num_N = length(Neighbors);     % number of neighbors to jerk
N_avg = zeros(k,1);            % Series of neighbor averages
avg = zeros(k,1);              % Series of global averages
avg(1) = mean(x(:,1));             % average of initial conditions, for convergence criteria


%%%% State Matrices

% Initialize B Matrix
B = zeros(1,N)';
B(jerk_node,1) = 1;

% Initialize C Matrix
C = zeros(1,N);
for i=1:num_N
    index = Neighbors(i);
    C(1,index) = 1/num_N;
end

% Initialize A Matrix
Ap = kp*B*C; % Proportional feedback to A matrix
Adjacency = Adjacency-Ap;    % A with feedback


% Opinion Dispersion
while (~converge) 
    
    %State transition
    x_new = Adjacency*x(:,k)+kp*B*jerk_goal; 
    k=k+1;
    x(:,k) = x_new;
    
    % Neighbor Average
    temp_sum = 0;
    for iii = 1:num_N
        temp_sum = temp_sum + x(Neighbors(iii),k);
    end
    
    % Averages/Error Update
    N_avg(k) = temp_sum/num_N;
    avg(k) = mean(x(:,k));
    err(k,1) = jerk_goal - N_avg(k); %Neighbor Error
    err(k,2) = jerk_goal - avg(k);  %Global Error    
    
    
    % Apply Capping
    if (capped)
        if x(jerk_node,k)>1
            x(jerk_node,k) = 1;
        end
        if x(jerk_node,k)<0
            x(jerk_node,k) = 0;
        end
    end
    
    % Check converge
    if abs(avg(k-1)-avg(k))<0.0001 && k > 10
        converge = 1;
    end
    itters = itters+1;
end
end
