clc; close all; clear all;

%parameters of the system
A1 = 28; A2 = 32; A3 = 28; A4 = 32;
a1 = 0.071; a2 = 0.057; a3 = 0.071; a4 = 0.057;
kc = 0.5; g = 981;

%Operating point parameters
h1_o = 12.4; h2_o = 12.7; h3_o = 1.8; h4_o = 1.4;
h_op = [h1_o; h2_o; h3_o; h4_o]; %operating point
T1 = (A1/a1) * sqrt(2*h1_o/981); T2 = (A2/a2)* sqrt(2*h2_o/981); T3 = (A3/a3) * sqrt(2 * h3_o /981); T4 = (A4/a4) * sqrt(2 * h4_o/981);
v1 = 3; v2 = 3;
k1 = 3.33; k2  = 3.35;
gamma1 = 0.70; gamma2 = 0.60;

% defining the A, B and H matrices
A = [-1/T1 0 A3/(A1*T3) 0; 0 -1/T2 0 A4/(A2*T4); 0 0 -1/T3 0; 0 0 0 -1/T4 ];

B = [gamma1*k1/A1 0; 0 gamma2*k2/A2; 0 (1-gamma2)*k2/A3; (1-gamma1)*k1/A4 0];

H = [kc 0 0 0; 0 kc 0 0 ];

N = 10000; % number of iterations

%% Kalman filter
X_post(:,1) = h_op; % intialization
P_0 = 1 * eye(4); % initial co-variance matrix
P_post(:,:,1) = P_0; % intial covariance
P_post_value(1) = det(P_post(:,:,1)); 
Q = 0.1 * eye(4) ; R = 0.01 * eye(2);
true_values = readmatrix("measurement_state_time_true.txt"); %read values from data generated
z_meas = true_values(:,2:3)'; %measured values 
x_true = true_values(:,4:7)'; %state values
rng(10, 'twister'); %random number generation setting

for i = 2:N+1
%measurement noise is  10 times lesser compared to process noise is the
%assumption
    w = sqrt(0.1) * randn(size(A,1),1); %state/process noise
    v = sqrt(0.01) * rand(size(H,1),1); %measurement noise
    
    %Prediction steps start here
    X_pri(:,i) = A * X_post(:,i-1) + B * [v1;v2] + h_op + w; %apriori  state estimate

    P_pri(:,:,i) = A * P_post(:,:,i-1) * A' + Q; %apriori covariance 

    P_pri_value(i) = det(P_pri(:,:,i));

    K(:,:,i) = P_pri(:,:,i) * H' * inv(H * P_pri(:,:,i) * H' + R); %kalman gain

    z_est(:,i) = H * X_pri(:,i); %measurement estimate from apriori estimate

    error_pri(:,i) = z_meas(:,i) + v - z_est(:,i); %Innovation/ apriori residue

    %prediction steps end here

    %correction steps start here

    X_post(:,i) = X_pri(:,i) + K(:,:,i) * error_pri(:,i); %Aposteriori state estimate

    error_post(:,i) = z_meas(:,i) + v - (H * X_post(:,i)); %aposteriori residue 

    P_post(:,:,i) = P_pri(:,:,i) - K(:,:,i) * H * P_pri(:,:,i); %aposteriori covariance

    %correction steps end here

    P_post_value(i) = det(P_post(:,:,i));

end

X_pri(:,1) = [];
%% Tank 1
% This section plots Posterior, true and prior states estimated by the
% Kalman filter
figure;
plot(X_pri(1,1:50)', 'LineWidth', 1.5)
hold on;
plot(x_true(1,1:50),'LineStyle', '-')
hold on;
plot(X_post(1,1:50)', 'LineWidth', 1.5)
title('Tank 1')
legend('Priori Estimate', 'True Value', 'Posteriori Estimate')
hold off;
%% Tank 2
% This section plots Posterior, true and prior states estimated by the
% Kalman filter
figure;
plot(X_pri(2,1:50)', 'LineWidth', 1.5)
hold on;
plot(x_true(2,1:50), 'LineStyle', '-')
hold on;
plot(X_post(2,1:50)', 'LineWidth', 1.5)
title('Tank 2')
legend('Priori Estimate', 'True Value', 'Posteriori Estimate')
hold off;

%% Tank 3
% This section plots Posterior, true and prior states estimated by the
% Kalman filter
figure;
plot(X_pri(3,1:50)', 'LineWidth', 1.5)
hold on;
plot(x_true(3,1:50),'LineStyle', '-')
hold on;
plot(X_post(3,1:50)', 'LineWidth', 1.5)
title('Tank 3')
legend('Priori Estimate', 'True Value', 'Posteriori Estimate')
hold off;

%% Tank 4
% This section plots Posterior, true and prior states estimated by the
% Kalman filter
figure;
plot(X_pri(4,1:50)', 'LineWidth', 1.5)
hold on;
plot(x_true(4,1:50),'LineStyle', '-')
hold on;
plot(X_post(4,1:50)', 'LineWidth', 1.5)
title('Tank 4')
legend('Priori Estimate', 'True Value', 'Posteriori Estimate')
hold off;

%% Determinent of covariance
% This section plots the determinant of the posterior covariance and prior
% covariance for the first 10 iterations
figure()
plot(P_post_value(1:10), 'LineWidth', 2);
hold on;
plot(P_pri_value(2:10), 'LineWidth', 2);
xlabel('Iterations')
ylabel('Determinent of Covaraince matrices')
legend('Aposteriori Covariance', 'Apriori Covariance')
hold off;

%%
% This section plots the prior residues or Innovations for the first 50
% iterations
figure()
plot(error_pri(:,1:50)', 'LineWidth', 2);
xlabel('Iterations')
ylabel('error in cm')
title('Apriori residue/Innovations')

%%
% This section plots the posterior residues for the first 50 iterations
figure()
error_post(:,1) = [];
plot(error_post(:,1:50)', 'LineWidth', 2)
xlabel('Iterations')
ylabel('error in cm')
title('Aposteriori residue')

%%
%This section plots the Kalman gains of each state
figure;
K(:,:,1) = [];
for i = 1:size(K, 1) % Loop through each state
    subplot(size(K, 1), 1, i); % Create a subplot for each state
    plot(1:N, squeeze(K(i, :, :)), 'LineWidth', 2);
    title(['Kalman Gain for State ' num2str(i)]);
    xlabel('Iteration');
    ylabel('Kalman Gain');
    legend('Gain 1', 'Gain 2'); 
    grid on;
end


