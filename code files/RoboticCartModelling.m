%% Step 1%% Worst case velocity profiling
clc
close all
clear all
% Distance parameters
s_i = 0.0
s_f = 20.0 % in metres

% max acceleration

a_max = 0.5 % in metres per second squared

% velocity
v_i = 0.0 
v_max = sqrt(v_i^2 +2*a_max*(s_f-s_i))  % in metres per second

% time parameters
t_acc = v_max/a_max
% time taken for v_max   % in seconds
tt = 2*(s_f-s_i)/(v_i+v_max)  % in seconds
t_const_v = ((s_f-s_i)-a_max*t_acc^2)/v_max
