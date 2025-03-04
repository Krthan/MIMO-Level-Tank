%% Data Generation code for the Quadruple-Tank problem
% Variable initialization

A1 = 28; A2 = 32; A3 = 28; A4 = 32; %(in cm^2)

a1 = 0.071; a2 = 0.057; a3 = 0.071; a4 = 0.057; %(in cm^2)

kc = 0.5; 

g = 981; % in cm/s^2

gamma1 = 0.7; gamma2 = 0.6;

k1 = 3.33; k2 = 3.35;

v1 = 3; v2 = 3;

h0 = [12.4; 12.7; 1.8; 1.4]; %operating point/inital condition

params = [A1, A2, A3, A4, a1, a2, a3, a4, kc, g, gamma1, gamma2, k1, k2, v1, v2]; %combining all variables for input into the function

step_size = 0.05;
tspan = 0:step_size:500; %10000 time values for 500 seconds

[t,x] = ode45(@(t,x)fourtank_non_lin(t,x,params), tspan, h0); %function call, diff eqn solver ode45
%%
%plots the variation of heights in the tanks
figure;
plot(x(1:10000,1), 'LineWidth', 1.5)
hold on;
plot(x(1:10000,2), 'LineWidth', 1.5)
hold on;
plot(x(1:10000,3), 'LineWidth', 1.5)
hold on;
plot(x(1:10000,4), 'LineWidth', 1.5)
hold off;
title('Tank Height')
xlabel('Iterations')
ylabel('Height in each tank (in cm)')
legend('Tank 1', 'Tank 2', 'Tank 3', 'Tank 4')

%%

x(end,:); % steady state values 
C = [kc 0 0 0; 0 kc 0 0]; %measurement matrix
z_true = (C * x')'; %measurement values
time_state_meas_true_array = [t z_true x]; %making a matrix to be written in a file
writematrix(time_state_meas_true_array, 'measurement_state_time_true', 'Delimiter', '\t') %writing the data file

