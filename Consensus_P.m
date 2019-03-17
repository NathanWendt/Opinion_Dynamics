clear all

% for i = 1:N  %Setting up the state matrix
%     deg_i = sum(M(i,:));
%     A_const = 1/(deg_i);
%     A(i,:) = A_const*M(i,:);
% end


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
capped = 0;                    % boolean for capping jerk's opinion by [0-1]
random50 = 0;                  % boolean for random noise every 50 iterations
noise_factor = 0.01;           % scales random noise

controller = 0;                % controller type: proportional = 0, PI = 1;
kp = 0.5;                        % proportional gain

k = 1;                         % discrete time counter
max_k = 400;                   % max number of iterations
err = ones(k,2);               % errors of [Neighbor; Global]
converge = 0;                  % if converged = 1

Neighbors = neighbors(G,15);   % neighbors of jerk
num_N = length(Neighbors);     % number of neighbors to jerk
N_avg = zeros(k,1);            % Series of neighbor averages
avg = zeros(k,1);              % Series of global averages
avg(1) = mean(x0);             % average of initial conditions, for convergence criteria
x = zeros(N,k);                % series of state vector states

%%%% State Matrices

% initializing state vector
x(:,1) = x0;                   
x(jerk_node,1) = jerk_goal;

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
A = A-Ap;    % A with feedback


%% Opinion Dispersion
while (~converge) 
    
    %State transition
    x_new = A*x(:,k)+kp*B*jerk_goal; 
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

%     temp_jerk = x(jerk_node,k);
%     rand_factor = 2*rand()-1;
%     
%     for iii = 1:N
% %        rand_factor = 2*rand()-1;
%         if rand_factor > 0
%             x(iii,k) = min(x(iii,k)+noise_factor*rand_factor,1);
%         else
%             x(iii,k) = max(x(iii,k)+noise_factor*rand_factor,0);
%         end
%     end
%     
%     x(jerk_node,k) = temp_jerk;
    
    
    % Apply Capping
    if (capped)
        if x(15,k)>1
            x(15,k) = 1;
        end
        if x(15,k)<0
            x(15,k) = 0;
        end
    end
    
    % Check converge
%     if k > max_k
%         converge = 1;
%     end
    if abs(avg(k-1)-avg(k))<0.0001 && k > 10
        converge = 1;
    end
end

%% FIGURES
figure(1);
plot(G);
title('Dolphin 62');

figure(2);
x_ax = 1:1:k;
goal = jerk_goal*ones(length(x_ax));
plot(x_ax, avg,'r', x_ax, x(15,:), 'b', x_ax,N_avg, 'g', x_ax, goal, 'k--')
xlim([-5 k+k/10])
limyu = max(x(15,:))+0.15;
limyl = min(min(x(15,:))-0.15,0);
ylim([limyl limyu])
legend('Global Average','Jerk','Neighbors Average','Jerk Goal')
title1 = strcat('P: ',num2str(kp),', Capped = ',num2str(capped));
title(title1)
xlabel('iterations')
ylabel('continuous opinion (0-1)')
hold on

