% The Objective of this code is to obtain angular velocity and acceleration profiles
% based on which the worst case scenario torque profile will be obtained.
R =     % R is the wheel Radius
psi_dot_r = s_dot/R
psi_dot_l = s_dot/R

theta_dot = (R/2L)*(psi_dot_r-psi_dot_l)  % this might also be obtained from a sensor or we might have to use a logic condition later
