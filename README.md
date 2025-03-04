## MIMO Nonlinear level tank Problem
This repo contains implementation of filtering (Kalman and Particle) and MPC controller in MATLAB. The system is adapted from the paper, The Quadruple-Tank Process: A Multivariable Laboratory Process with an
Adjustable Zero”, by Karl Henrik Johansson

The system dynamics can be referred in the first few pages of https://github.com/Krthan/MIMO-Level-Tank/blob/main/kalman_filter.pdf (kalman_filter.pdf). This non-linear system dynamics is then run through ode45 to generate artificial data from a specific starting point https://github.com/Krthan/MIMO-Level-Tank/blob/main/Data_generation.m (Data_generation.m). The above script generates 10000 data points which I've used in for filtering: KF (Kalman Filter) and PF (Particle Filter) implementation
