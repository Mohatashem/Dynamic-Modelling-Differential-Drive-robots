close all
clear all
clc
% Key Parameters of the cart
R = 0.15 %Wheel Radius in metres
L = 3.1 % Axel distance between the wheels in metres
d = 0.0  % Distance of COM from the Axel    metres

M = 6000 % Mass of the entire robot including wheels and actuators kg
J = 7000     % Moment of Inertia with respect to centre of Mass kg*m^2


% Forward Differential Kinematics Model
vu_r = 1.5   % right wheel linear velocity in metres
vu_l = 1.5 % left wheel linear velocity in metres

x_dot_robot_frame = (vu_r+vu_l)/2 % average linear velocity of the frame
x_ddot_robot_frame = 0.5
theta_dot = R*(vu_r-vu_l)/(2*L)% angular velocity of the COM
theta_ddot = 0.0% angular accelaration COM



d_psi_r = vu_r/R   % right wheel angular velocity   rad/s
d_psi_l =  vu_r/R   % left wheel angular velocity   rad/s
dd_psi_r = x_ddot_robot_frame/R  % right wheel angular accelaration rad/s^2    assuming for now
dd_psi_l = x_ddot_robot_frame/R  % left wheel angular accelaration  rad/s^2    assuming for now

% Parameters of dynamic Model in terms of Rotational velocities and
% Actuator Torques using Newton-Euler Approach
r_first_term = (((R*(M*d^2+J))/(4*L^2))+((M*R)/4))*dd_psi_r
r_second_term = (((-1*R*(M*d^2+J))/(4*L^2))+((M*R)/4))*dd_psi_l
r_third_term = ((M*d*R^2)/(4*L^2))*(d_psi_l^2)
r_fourth_term = ((M*d*R^2)/(4*L^2))*d_psi_r*d_psi_l

l_first_term  = (((R*(M*d^2+J))/(4*L^2))+((M*R)/4))*dd_psi_l
l_second_term = (((-1*R*(M*d^2+J))/(4*L^2))+((M*R)/4))*dd_psi_r
l_third_term =   ((M*d*R^2)/(4*L^2))*(d_psi_r^2)
l_fourth_term = ((M*d*R^2)/(4*L^2))*d_psi_r*d_psi_l


% Torque Calculation based on Dynamic Model
tau_r = R*(r_first_term+r_second_term-r_third_term+r_fourth_term)
tau_l = R*(l_first_term+l_second_term-l_third_term+r_fourth_term)


