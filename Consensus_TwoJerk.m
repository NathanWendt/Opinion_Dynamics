%% LOAD DATA
clear all
load('G_dolph');
load('M_dolph');
load('x0_dolph');
load('A_dolph');
M = M_dolph;
G = G_dolph;
N = length(A);

%% SETUP

%%%% VARIABLES

jerk_goal(1,1) = 0.3;          % control target for jerk 1
jerk_goal(2,1) = 0.5;          % control target for jerk 2
jerk_node(1,1) = 15;           % jerk 1 node in network
jerk_node(2,1) = 14;           % jerk 2 node in network
capped = 1;                    % boolean for capping jerk's opinion by [0-1]
global_vision = 1;             % boolean for global or neighbor visability
random50 = 0;                  % boolean for random noise every 50 iterations
noise_factor = 0.01;           % scales random noise

kp(1,1) = 0.1;                 % proportional gain
kp(2,1) = 1;                   % proportional gain
ki(1,1) = 0.1;                 % integral gain
ki(2,1) = 0;                   % integral gain

k = 1;                         % discrete time counter
max_k = 1200;                  % max number of iterations
err = ones(k,2);               % errors of [Neighbor; Global]
converge = 0;                  % if converged = 1

num_nodes = numnodes(G);
Neighbors{1} = neighbors(G,jerk_node(1,1));   % neighbors of jerk
Neighbors{2} = neighbors(G,jerk_node(2,1));   % neighbors of jerk
num_N(1,1) = length(Neighbors{1});     % number of neighbors to jerk
num_N(2,1) = length(Neighbors{2});     % number of neighbors to jerk
N_avg(1,1) = zeros(1,k);            % Series of neighbor averages
N_avg(2,1) = zeros(1,k);            % Series of neighbor averages
avg = zeros(1,k);              % Series of global averages
avg(1) = mean(x0);             % average of initial conditions, for convergence criteria
x = zeros(N+2,k);              % series of state vector states + error state

%%%% State Matrices

% initializing state vector
x(1:N,1) = x0;                   
x(jerk_node(1,1),1) = jerk_goal(1,1);
x(jerk_node(2,1),1) = jerk_goal(2,1);

% Initialize B Matrix
B = zeros(2,N+2)';
B(jerk_node(1,1),1) = 1;
B(jerk_node(2,1),2) = 1;

% Initialize C Matrix
% % Neighbor Vision
if global_vision == 0
    C(1,:) = zeros(1,N+2);
    C(2,:) = zeros(1,N+2);
    Ci(1,:) = zeros(1,N+2); Ci(1,N+1)=1; % [0 0 . . . 1 0]
    Ci(2,:) = zeros(1,N+2); Ci(2,N+2)=1; % [0 0 . . . 0 1]
    
    for i=1:num_N(1,1) %% TODO Take out for loop and just do assignent
        index = Neighbors{1}(i);
        C(1,index) = 1/num_N(1,1);
    end
    
    for i=1:num_N(2,1)
        index = Neighbors{2}(i);
        C(2,index) = 1/num_N(2,1);
    end
    
% % Global Vision
else
    C(1,:) = zeros(1,N+2);
    C(2,:) = zeros(1,N+2);
    C(1,1:num_nodes) = (1/num_nodes)*ones(1,N);
    C(2,1:num_nodes) = (1/num_nodes)*ones(1,N);
    Ci(1,:) = zeros(1,N+2); Ci(1,N+1)=1; % [0 0 . . . 1 0]
    Ci(2,:) = zeros(1,N+2); Ci(2,N+2)=1; % [0 0 . . . 0 1]
end

% Initialize A Matrix
err_row(1,:) = -C(1,:);
err_row(1,N+1) = 1;
err_row(2,:) = -C(2,:);
err_row(2, N+2) = 1;
A = [A zeros(N,2)];            % NxN state matrix A
A = [A; err_row];
Ap = kp(1,1)*B(:,1)*C(1,:)+kp(2,1)*B(:,2)*C(2,:);                   % Proportional feedback to A matrix
Ai = ki(1,1)*B(:,1)*Ci(1,:)+ki(2,1)*B(:,2)*Ci(2,:);                  % Integral feedback to A matrix
A_feedback = A-Ap+Ai;          % A with feedback

% Update B matrix
B(N+1,1) = 1;                  % Term added for updating error state
B(N+2,2) = 1;
B(jerk_node(1,1),1) = kp(1,1);           % kp is internalized to B(jerk,k)
B(jerk_node(2,1),2) = kp(2,1);           % kp is internalized to B(jerk,k)


%% Opinion Dispersion
while (~converge) 
    
    %State transition
    x_new = A_feedback*x(:,k)+B*jerk_goal; 
    k=k+1;
    x(:,k) = x_new;
    
    % Neighbor sum
    temp_sum(1,1) = sum(x(Neighbors{1},k));
    temp_sum(2,1) = sum(x(Neighbors{2},k));  

    % Apply Capping
    if (capped)
        if x(jerk_node(1,1),k)>1
            x(jerk_node(1,1),k) = 1;
        elseif x(jerk_node(1,1),k)<0
            x(jerk_node(1,1),k) = 0;
        end
        
        if x(jerk_node(2,1),k)>1
            x(jerk_node(2,1),k) = 1;
        elseif x(jerk_node(2,1),k)<0
            x(jerk_node(2,1),k) = 0;
        end
    end
    
    % Averages/Error Update
    N_avg(1,k) = temp_sum(1,1)/num_N(1,1);
    N_avg(2,k) = temp_sum(2,1)/num_N(2,1);
    avg(k) = mean(x(1:N,k));
    err(1,k) = jerk_goal(1,1) - avg(k); %Neighbor Error
    err(2,k) = jerk_goal(2,1) - avg(k);  %Global Error  
    
    % Check converge
    if k > max_k
        converge = 1;
    end
end

%% FIGURES
figure(1);
plot(G);
title('Dolphin 62');

figure(2);
x_ax = 1:1:k;
goal(1,:) = jerk_goal(1,1)*ones(length(x_ax),1);
goal(2,:) = jerk_goal(2,1)*ones(length(x_ax),1);
plot(x_ax, avg,'r', x_ax, x(jerk_node(1,1),:), 'b', x_ax,N_avg(1,:), 'g', x_ax, goal(1,:), 'k-', x_ax, x(jerk_node(2,1),:), 'b--', x_ax,N_avg(2,:), 'g--', x_ax, goal(2,:), 'k--')
xlim([0 k+k/10])
limyu = max(max(x(jerk_node(1,1),:),x(jerk_node(2,1),:)))+0.15;
limyl = min([min(x(jerk_node(1,1),:))-0.15 min(x(jerk_node(2,1),:))-0.15 0]);
ylim([limyl limyu])
legend('Global Average','PI Jerk','PI Neighbors Average','PI Jerk Goal', 'P Jerk','P Neighbors Average','P Jerk Goal')
title1 = strcat('P: ',num2str(kp),', PI: ',num2str(ki),', Capped = ',num2str(capped));
title(title1)
xlabel('iterations')
ylabel('continuous opinion (0-1)')

figure(3)
plot(x_ax, err(1,:), 'b', x_ax, err(2,:),'g')
legend('PI error','P error')