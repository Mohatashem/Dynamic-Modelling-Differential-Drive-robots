clc
close all
clear all
% setting time in seconds

n =  0.1  % sampling

w_init = 0.0
theta_pos = 2 % rad  i.e. the position that needs to be reached or the displacement that needs to be carried out
w_max = 0.5   % rad/s
a_max = 0.5   % rad/s^2
% choosing  constant accelaration time from w_max and a_max

t_acc = w_max/a_max
% choosing constant velocity time from t_acc, a_max, w_max and theta_pos
t_const =  (theta_pos - a_max*t_acc^2)/w_max

t0 = 0 % initial time
t1 = t_acc
ti1 = t0:n:t1    % time interval in seconds
ti2 = t1:n:t_const+t1
ti3 = (t_const+t1):n:(t_const+2*t1)
w1 = a_max*ti1
w2 = w_max*ones(1,length(ti2))
w3 = w_max-a_max*ti1 % multiplying by ti1 cuz it gives the correct number of values, but ti3 is the real interval needed for plotting


t = [ti1 ti2 ti3]
w_profile = [w1 w2 w3]

figure
subplot(2,1,1)
plot(t,w_profile)
ylabel('Angular Velocity Profile in rad/s   ')
xlabel('time')

a1 = (w_max-w_init)*ones(1,length(ti1))
a2 = (w_max-w_max)*ones(1,length(ti2))
a3 = (w_init-w_max)*ones(1,length(ti3))
a_profile = [a1 a2 a3]
subplot(2,1,2)
plot(t, a_profile)
ylabel('Angular Acceleration/Torque Profile ')
xlabel('time')
