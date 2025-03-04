clc; clear all; close all;

%parameters of the system
A1 = 28; A2 = 32; A3 = 28; A4 = 32;
a1 = 0.071; a2 = 0.057; a3 = 0.071; a4 = 0.057;
kc = 1; g = 981;

%Operating point parameters
h1_o = 12.4; h2_o = 12.7; h3_o = 1.8; h4_o = 1.4;
h_op = [h1_o; h2_o; h3_o; h4_o]; %operating point
T1 = (A1/a1) * sqrt(2*h1_o/981); T2 = (A2/a2)* sqrt(2*h2_o/981); T3 = (A3/a3) * sqrt(2 * h3_o /981); T4 = (A4/a4) * sqrt(2 * h4_o/981);
v1 = 3; v2 = 3;
k1 = 3.33; k2  = 3.35;
gamma1 = 0.70; gamma2 = 0.60;

% defining the A, B and H matrices
Ad = [-1/T1 0 A3/(A1*T3) 0; 0 -1/T2 0 A4/(A2*T4); 0 0 -1/T3 0; 0 0 0 -1/T4 ];

Bd = [gamma1*k1/A1 0; 0 gamma2*k2/A2; 0 (1-gamma2)*k2/A3; (1-gamma1)*k1/A4 0];

Hd = [kc 0 0 0; 0 kc 0 0; 0 0 kc 0; 0 0 0 kc];
Dd = zeros(size(Hd,1),size(Bd,2));


% MPC parameters
Np = 20;  % Prediction horizon
Nc = 5;   % Control horizon
Q = diag([0, 0, 100, 100]);  % State weights
R = diag([1, 1]);    % Input weights

% Constraints
u_min = 0 * ones(Nc * size(Bd,2), 1);
u_max = 20 * ones(Nc * size(Bd,2), 1);
du_min = -5 * ones(Nc * size(Bd,2), 1);
du_max = 5 * ones(Nc * size(Bd,2), 1);

%% Converting to an augmented matrix
A_a = [Ad, zeros(size(Ad,1)); Hd*Ad, eye(size(Hd))];
B_a = [Bd; Hd*Bd];
C_a = [zeros(size(Ad,1)), eye(size(Hd))];

%% building F and phi matrix
F = [];
phi = [];
for i = 1:Np
    F = [F; C_a*(A_a^i)];
    row = [];
    for j = 1:Nc
        if i >= j
            row = [row, C_a * A_a^(i-j) * B_a];
        else
            row = [row, zeros(size(Bd))];
        end
    end
    phi = [phi; row];   
end


% Cost matrices
Q_bar = kron(eye(Np), Q); % Weighting for predicted states
R_bar = kron(eye(Nc), R); % Weighting for control inputs

%set point
ref = [0; 0; 2.4; 2.8];
ref_matrix = repmat(ref,Np,1);
Nsim = 200; %sim iteration

x = h_op; % Initial state

x_log = [];
u_log = [];
y_log = [];

% Initial Kalman filter setup
x_hat = h_op; % Initial state estimate
P = eye(size(Ad)); % Initial error covariance matrix

%setting initial u
u_prev = [0;0];

n = Nc; % Number of blocks
m = 2; % Size of identity matrices

I_block = eye(m); % Create identity matrix block
T = blkdiag(I_block, I_block, I_block, I_block, I_block); % Initialize diagonal

for i = 2:n
    for j = 1:i-1
        T((i-1)*m+1:i*m, (j-1)*m+1:j*m) = I_block; % Add identity to lower diagonal blocks
    end
end

%% simulating mpc and kf
for i = 1:Nsim

    y_meas = Hd * x + randn(size(Hd,1),1) * 0.01; % measurement with noise
    [x_hat, P] = kalman_update(Ad, Bd, Hd, x_hat, P, y_meas); %kalman filter function call, state estimate
    
    x_aug = [x_hat - x ; y_meas]; %augmented state definition

    %quadratic function defined as 1/2 x' * H * x + f' x, we define H and f
    %below, for augmented ss, x --> Del u
    
    H_cost = phi' * Q_bar * phi + R_bar; 
    f_cost = -(ref_matrix - F * x_aug)' * Q_bar' * phi;

    %% finding Aineq and Bineq for constraints
    % rate of input change constraint
    Bineq_rate = [-du_min; du_max];
    Aineq_rate = [-eye(Nc * 2); eye(Nc * 2)];

    %input constraint
    Bineq_abs = [-u_min + repmat(u_prev, Nc, 1); u_max - repmat(u_prev, Nc, 1)];
    Aineq_abs = [-T; T];

    %stacking the matrices on top of each other
    Aineq = [Aineq_abs; Aineq_rate];
    Bineq = [Bineq_abs; Bineq_rate];

    % quadprog function call, returns minimizer
    del_u_opt = quadprog(H_cost,f_cost,Aineq, Bineq, [], [], [], [], [], optimoptions('quadprog', 'Display', 'off')); %delta u mpc
    del_u_applied = del_u_opt(1:size(Bd, 2)); %delta u

    u_applied = u_prev + del_u_applied;
    
    % Logging control and state variables
    u_log(:, i) = u_applied; 
    x_log(:, i) = x_aug;
    y_log(:, i) = C_a * x_aug; 

    % Apply input to the augmented plant
    x_aug = A_a * x_aug + B_a * del_u_applied;

    x = x + x_aug(1:4) + randn(size(Ad,1),1) * 0.1; %get x from del_x(= augmented) and process noise
    u_prev = u_applied;
    
end

%% KF function
function [x_hat, P] = kalman_update(Ad, Bd, Cd, x_hat, P, y_meas)
    % Kalman filter measurement update
    R_kal = eye(size(Cd, 1)) * 0.0001; % Measurement noise covariance
    Q_kal = eye(size(Ad, 1)) * 0.01; % Process noise covariance

    % Prediction step
    x_hat = Ad * x_hat;
    P = Ad * P * Ad' + Q_kal;

    % Update step
    K = P * Cd' / (Cd * P * Cd' + R_kal); % Kalman gain
    x_hat = x_hat + K * (y_meas - Cd * x_hat);
    P = (eye(size(K, 1)) - K * Cd) * P;
end

figure();
subplot(2,1,1)
plot(1:Nsim, y_log(3,:), '-b', LineWidth=1.5);title('Tank height 3 vs. iterations'); xlabel('Iterations (1 = 0.1s)'); ylabel('height of tank 3 (cm)');hold on;
plot(1:Nsim, repmat(ref(3), 1, Nsim), '--b', LineWidth=1.5); hold off;
legend('Controlled tank height 3 (cm)', 'Given reference')
subplot(2,1,2);
plot(1:Nsim, y_log(4,:), '-r', LineWidth=1.5);xlabel('Iterations (1 = 0.1s)'); ylabel('height of tank 4(cm)'); hold on;
plot(1:Nsim, repmat(ref(4),1,Nsim), '--r', LineWidth=1.5); title('Tank height 4 vs. iterations'); hold off;
legend('Controlled tank height 4 (cm)', 'Given reference')

figure();
subplot(2,1,1)
stairs(1:Nsim, u_log(1,:), '-b', LineWidth=1.5); title('Change in Input voltage v1 (V)')
subplot(2,1,2);
stairs(1:Nsim, u_log(2,:), '-r', LineWidth=1.5); title('Change in Input voltage v2 (V)')






