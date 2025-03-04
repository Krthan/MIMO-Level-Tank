% Parameters for the particle filter
num_particles = 1000;     % Number of particles
num_iterations = 1000;      % Number of time steps
state_dim = 4;            % State dimension
measurement_dim = 2;      % Measurement dimension

% Process and measurement noise
process_noise_std = 0.1;
measurement_noise_std = 0.01;
    
%parameters of the system
A1 = 28; A2 = 32; A3 = 28; A4 = 32;
a1 = 0.071; a2 = 0.057; a3 = 0.071; a4 = 0.057;
kc = 0.5; g = 981;

%Operating point parameters
h1_o = 12.4; h2_o = 12.7; h3_o = 1.8; h4_o = 1.4;
h_op = [h1_o; h2_o; h3_o; h4_o]; %operating point
T1 = (A1/a1) * sqrt(2*h1_o/981); T2 = (A2/a2)* sqrt(2*h2_o/981); T3 = (A3/a3) * sqrt(2 * h3_o /981); T4 = (A4/a4) * sqrt(2 * h4_o/981);
v1 = 3; v2 = 3;
u = [v1;v2];
k1 = 3.33; k2  = 3.35;
gamma1 = 0.70; gamma2 = 0.60;

% defining the A, B and H matrices
A = [-1/T1 0 A3/(A1*T3) 0; 0 -1/T2 0 A4/(A2*T4); 0 0 -1/T3 0; 0 0 0 -1/T4 ];

B = [gamma1*k1/A1 0; 0 gamma2*k2/A2; 0 (1-gamma2)*k2/A3; (1-gamma1)*k1/A4 0];

H = [kc 0 0 0; 0 kc 0 0 ];

%% Intialization step - Step 1
% Initial state and particles
x_true = h_op;         % True initial state
P0 = chol(process_noise_std) * eye(4); %intial co-variance matrix
L  = chol(P0);
particles =  randn(num_particles,state_dim) * L + ones(num_particles, state_dim) * h_op; % Initial particles
particles = particles'; %transpose to match the state vector for calculating the dynamics
weights = ones(1, num_particles) / num_particles;           % Equal weights initially

% Storage for estimates
state_estimates = zeros(state_dim, num_iterations);
true_values = readmatrix("measurement_state_time_true.txt"); %read values from data generated
measurements = true_values(1:num_iterations,2:3)'; %measured values 
true_states = true_values(1:num_iterations,4:7)'; %state values

for k = 1:num_iterations % Iterations
    %% Prediction of each particle - Prediction Step - Step 2
    % True state evolution
    process_noise = process_noise_std * randn(state_dim, 1);
    true_states_with_noise(:,k) = true_states(:,k) + process_noise; %adding noise to the true state
    
    % Measurement with noise
    measurement_noise = measurement_noise_std * randn;
    measurements_with_noise(:,k) = measurements(:,k) + measurement_noise; %adding noise to the measurements

    % Propagate each particle through the system dynamics with process noise
    particles = A * particles + process_noise_std * randn(state_dim, num_particles) + h_op;

    %% Update weights based on measurement likelihood - Update Step - Step 3
    for i = 1:num_particles
        predicted_measurement = H * particles(:, i);
        measurement_error = measurements_with_noise(:,k) - predicted_measurement;
        weights(i) = exp(-0.5 * (measurement_error' * measurement_error/measurement_noise_std)); %exponential weight distribution
    end

    % Normalize weights
    weights = weights / sum(weights);

    % Estimate state as the weighted average of particles
    state_estimates(:, k) = particles * weights';

    %%  Resample particles based on their weights
    cumulative_weights = cumsum(weights);
    new_particles = zeros(state_dim, num_particles); %generate new particle variable
    for i = 1:num_particles
        random_num = rand;
        index = find(cumulative_weights >= random_num, 1, 'first');
        new_particles(:, i) = particles(:, index); %through rejection sampling create new particles
    end
    particles = new_particles;
    weights = ones(1, num_particles) / num_particles; % Reset weights to unity
end

%% Plotting the true state and the estimated state
figure()
plot(1:num_iterations, true_states(1,:), 'r-', 'LineWidth', 2); hold on; %Tank 1
plot(1:num_iterations, state_estimates(1,:), 'b--', 'LineWidth', 2);
legend('True State', 'Estimated State');
xlabel('Time Step');
ylabel('Height in cm');
title('Tank 1');

figure()
plot(1:num_iterations, true_states(2,:), 'r-', 'LineWidth', 2); hold on; %Tank 2
plot(1:num_iterations, state_estimates(2,:), 'b--', 'LineWidth', 2);
legend('True State', 'Estimated State');
xlabel('Time Step');
ylabel('Height in cm');
title('Tank 2');


figure()
plot(1:num_iterations, true_states(3,:), 'r-', 'LineWidth', 2); hold on; %Tank 3
plot(1:num_iterations, state_estimates(3,:), 'b--', 'LineWidth', 2);
legend('True State', 'Estimated State');
xlabel('Time Step');
ylabel('Height in cm');
title('Tank 3');


figure()
plot(1:num_iterations, true_states(4,:), 'r-', 'LineWidth', 2); hold on; % Tank 4
plot(1:num_iterations, state_estimates(4,:), 'b--', 'LineWidth', 2); 
legend('True State', 'Estimated State');
xlabel('Time Step');
ylabel('Height in cm');
title('Tank 4');

%% Errors plotting
errors_states = true_states - state_estimates;
figure()
subplot(2,1,1)
plot(1:num_iterations,errors_states(1,:), 'r', 'LineWidth', 2); hold on;
title('Error in tank 1', 'FontSize', 14, 'FontWeight','bold')

subplot(2,1,2)
plot(1:num_iterations,errors_states(2,:), 'b', 'LineWidth', 2); hold on;
title('Error in tank 2', 'FontSize', 14, 'FontWeight','bold')

figure()
subplot(2,1,1)
plot(1:num_iterations,errors_states(3,:), 'r', 'LineWidth', 2); hold on;
title('Error in tank 3', 'FontSize', 14, 'FontWeight','bold')

subplot(2,1,2)
plot(1:num_iterations,errors_states(4,:), 'b', 'LineWidth', 2); hold on;
title('Error in tank 4', 'FontSize', 14, 'FontWeight','bold')

