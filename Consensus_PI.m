%% LOAD DATA

load('G_dolph');
load('M_dolph');
load('x0_dolph');
load('A_dolph');
M = M_dolph;
G = G_dolph;
N = length(A);

%% SETUP

%%%% VARIABLES

jerk_goal = 0.7;               % control target for the jerk
jerk_node = 15;                % jerk's node in network
stubborn_node = 14;            % stubborn agent's node in network
stubborn = 1;                  % boolean for pressence of stubborn agent
capped = 0;                    % boolean for capping jerk's opinion by [0-1]
global_vision = 1;             % boolean for gloabal visibility
random50 = 0;                  % boolean for random noise every 50 iterations
noise_factor = 0;              % scales random noise

controller = 0;                % controller type: proportional = 0, PI = 1;
kp = 5;                        % proportional gain
ki = -0.01;                       % integral gain

k = 1;                         % discrete time counter
max_k = 150;                   % max number of iterations
err = ones(k,2);               % errors of [Neighbor; Global]
converge = 0;                  % if converged = 1

num_nodes = numnodes(G);       % number of nodes
Neighbors = neighbors(G,15);   % neighbors of jerk
num_N = length(Neighbors);     % number of neighbors to jerk
N_avg = zeros(k,1);            % Series of neighbor averages
avg = zeros(k,1);              % Series of global averages
avg(1) = mean(x0);             % average of initial conditions, for convergence criteria
x = zeros(N+1,k);              % series of state vector states + error state

%%%% State Matrices

% initializing state vector
x(1:N,1) = x0;                   
x(jerk_node,1) = jerk_goal;

% Initialize B Matrix
B = zeros(1,N+1)';
B(jerk_node,1) = 1;

% Initialize C Matrix

if global_vision == 0
    C = zeros(1,N+1);
    Ci = zeros(1,N+1); Ci(1,N+1)=1; % [0 0 . . . 0 1]
    for i=1:num_N
        index = Neighbors(i);
        C(1,index) = 1/num_N;
    end

else
    C = zeros(1,N+1);
    C(1:num_nodes) = (1/num_nodes)*ones(1,N);
    Ci = zeros(1,N+1); Ci(1,N+1)=1; % [0 0 . . . 0 1]
end

% Initialize A Matrix
err_row = -C;
err_row(1,N+1) = 1;
A = [A zeros(N,1)];            % NxN state matrix A
A = [A; err_row];
Ap = kp*B*C;                   % Proportional feedback to A matrix
Ai = ki*B*Ci;                  % Integral feedback to A matrix
A_feedback = A-Ap+Ai;          % A with feedback
A_PF = A-Ap;

% Update B matrix
B(N+1,1) = 1;                  % Term added for updating error state
B(jerk_node,1) = kp;           % kp is internalized to B(jerk,k)


%% Opinion Dispersion
while (~converge) 
    
    %State transition
    x_new = A_feedback*x(:,k)+B*jerk_goal; 
    k=k+1;
    x(:,k) = x_new;
    
    % Neighbor sum
    temp_sum = 0;
    for iii = 1:num_N
        temp_sum = temp_sum + x(Neighbors(iii),k);
    end
    
    
    
    % Random impulses every 50 iterations
%     if mod(k,50) && random50 == 0
%         random_vector = rand(1,N)';
%         
%         x(1:N,k) = x(1:N,k) - noise_factor*rand(1,N)'; %random initial conditions between 0-1
%         x(jerk_node,k) = jerk_goal;
%     end
    
    temp_jerk = x(jerk_node,k);
    rand_factor = 2*rand()-1;
    
%     for iii = 1:N
%         rand_factor = 2*rand()-1;
%         if rand_factor > 0
%             x(iii,k) = min(x(iii,k)+noise_factor*rand_factor,1);
%         else
%             x(iii,k) = max(x(iii,k)+noise_factor*rand_factor,0);
%         end
%     end
%     
%     %x(1:N,k) = x(1:N,k) - noise_factor*rand(1,N)'; %random initial conditions between 0-1
%     x(jerk_node,k) = temp_jerk;

    % Apply Capping
    if (capped)
        if x(jerk_node,k)>1
            x(jerk_node,k) = 1;
        end
        if x(jerk_node,k)<0
            x(jerk_node,k) = 0;
        end
    end
    
    if (stubborn)
        x(stubborn_node,k) = x(stubborn_node,1);
    end
    
    % Averages/Error Update
    N_avg(k) = temp_sum/num_N;
    avg(k) = mean(x(1:N,k));
    err(k,1) = jerk_goal - N_avg(k); %Neighbor Error
    err(k,2) = jerk_goal - avg(k);  %Global Error    
    
    % Check converge
    if k > max_k
        converge = 1;
    end
end

%% FIGURES
% figure(1);
% plot(G,'Layout','force','NodeLabel',{});
% title('Dolphin 62');

figure(3);
x_ax = 1:1:k;
goal = jerk_goal*ones(1,length(x_ax));
p = plot(x_ax, avg, x_ax, x(15,:),x_ax,N_avg, x_ax, goal, 'k--');
xlim([2 k+k/10])
limyu = max(x(15,:))+0.15;
limyl = min(min(x(15,:))-0.15,0);
ylim([limyl limyu])
legend('Network Average','Selfish Agent','Neighbors Average','Selfish Agent Goal')
set(p, 'LineWidth',1.4)
set(gca,'FontSize',12, 'LineWidth',1.2)
title1 = strcat('P Jerk with global vision against stubborn agent, k_P: ',num2str(kp),', k_I: ',num2str(ki));%*,', Capped = ',num2str(capped)*%);
%title(title1)
xlabel('Time (k)')
ylabel('Aggregate Opinions')