close all
clear all
clc

% Objective of this code is to create smooth trajectory to traverse a distance. The output of this code
% should be linear distance, velocity and acceleration. The obtained profiles will be used to
% create angular velocities and acceleration profiles

% Necessart boundary conditions for kinematics
t0 = 0
n = 0.1
tif = 20
s0 = 0
sf = 5
s0_d = 0
sf_d = 0
s0_dd = 0
s0_dd = 0

t = tif
kin_mat =[0 ;5 ;0 ;0 ;0 ;0]
time_mat = [0 0 0 0 0 1; 
           t.^5 t.^4 t.^3 t.^2 t 1;
           0 0 0 0 1 0;
           5*t.^4 4*t.^3 3*t.^2 2*t 1 0;
           0 0 0 2 0 0;
           20*t.^3 12*t.^2 6*t 2 0 0]
% calculating Polynomial trajectory
coeffs = inv(time_mat)*kin_mat
t_profile = t0:n:tif
s = coeffs(1)*t_profile.^5+coeffs(2)*t_profile.^4+coeffs(3)*t_profile.^3+coeffs(4)*t_profile.^2+coeffs(5)*t_profile+coeffs(6)*ones(1,length(t_profile))
s_dot = 5*coeffs(1)*t_profile.^4+4*coeffs(2)*t_profile.^3+3*coeffs(3)*t_profile.^2+2*coeffs(4)*t_profile+coeffs(5)*ones(1,length(t_profile))
s_ddot = 20*coeffs(1)*t_profile.^3+12*coeffs(2)*t_profile.^2+6*coeffs(3)*t_profile+2*coeffs(4)*ones(1,length(t_profile))

figure(1)
subplot(3,1,1)
plot(t_profile,s)
ylabel('displacement profile')
subplot(3,1,2)
plot(t_profile,s_dot)
ylabel('velocity profile')
subplot(3,1,3)
plot(t_profile,s_ddot)
ylabel('acceleration profile')