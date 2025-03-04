# MIMO Nonlinear level tank Problem
## Introduction
This repo contains implementation of filtering (Kalman and Particle) and MPC controller in MATLAB. The system is adapted from the paper, The Quadruple-Tank Process: A Multivariable Laboratory Process with an
Adjustable Zero”, by Karl Henrik Johansson

The system dynamics can be referred in the first few pages of https://github.com/Krthan/MIMO-Level-Tank/blob/main/kalman_filter.pdf (kalman_filter.pdf). This non-linear system dynamics is then run through ode45 to generate artificial data from a specific starting point https://github.com/Krthan/MIMO-Level-Tank/blob/main/Data_generation.m (Data_generation.m). The above script generates 10000 data points which I've used in for filtering: KF (Kalman Filter) and PF (Particle Filter) implementation

The Kalman filter https://github.com/Krthan/MIMO-Level-Tank/blob/main/KalmanFilter.m has been implemented using the Co-variance matrix method. The Particle filter https://github.com/Krthan/MIMO-Level-Tank/blob/main/ParticleFilter.m with liklihood density method.

Next, the system was regulated to different states with https://github.com/Krthan/MIMO-Level-Tank/blob/main/constrained_mpc.m (constrained) and without https://github.com/Krthan/MIMO-Level-Tank/blob/main/unconstrained_mpc.m (unconstrained) state, input and input-rate constraints using a Model Predictive Controller. The results of filtering and regulation is provided below:

# Results
## Data generation
![tank_height_data_generated](https://github.com/user-attachments/assets/3207a10e-02e6-4d5d-bb42-46c872648b13)

## Kalman filter
![Tank1](https://github.com/user-attachments/assets/40b6d779-9327-4419-842d-b3ec54c265b2)

![Tank2](https://github.com/user-attachments/assets/a4e698d6-fd5c-46c8-b051-7e6aaa9c6c60)

![Tank3](https://github.com/user-attachments/assets/752b65fc-8ff2-446e-9015-6953213f419a)

![Tank4](https://github.com/user-attachments/assets/8d5390fd-15fd-46f6-96ca-02b0e68f077c)

![Kalmangain](https://github.com/user-attachments/assets/05449fe5-3698-496e-be4e-e28e45668ada)

![Innovations](https://github.com/user-attachments/assets/258a694f-26c7-4b18-a030-676c5245a135)

![Covariance](https://github.com/user-attachments/assets/adda8dc5-0acd-4d4e-952e-bb6e7287c6e4)

![apost_residue](https://github.com/user-attachments/assets/b200ec90-4ffd-4073-a686-2dc93e9f8e5a)

## Particle Filter
![Tank1](https://github.com/user-attachments/assets/956dcabf-c8b3-4286-aca4-dc84397dbc58)

![Tank2](https://github.com/user-attachments/assets/90f7f62c-8151-4b20-a84a-0d05dfdd202c)

![Tank3](https://github.com/user-attachments/assets/cffd10dd-4d97-4b8b-bc34-37b937b33fc7)

![Tank4](https://github.com/user-attachments/assets/bd7b7262-27ce-492f-9680-52d9c8ac5e5a)

![error_tank_1_2](https://github.com/user-attachments/assets/1dc53c45-a0c6-4563-85ca-ad08921ee12e)

![error_tank_3_4](https://github.com/user-attachments/assets/80f73019-f624-4454-8476-15eff8c55b76)

## Unconstrained MPC
![tank height](https://github.com/user-attachments/assets/4d59cb3d-e06b-4cf9-8156-ae3cf83c6bf0)

![change_in_input_voltage](https://github.com/user-attachments/assets/cbd8b387-b0d0-41ff-bdba-a322e816bded)

## Constrained MPC
![tank height](https://github.com/user-attachments/assets/3a7b8b76-2d9b-4db9-92f9-650a5f87e4cd)

![input](https://github.com/user-attachments/assets/f1e56903-6f9c-4ec7-b1bb-b0b3981801ba)












