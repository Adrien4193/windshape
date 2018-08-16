% This little script displays the step response and optionally the
% Bode diagram of the low-pass filter used for the WindShape project.

clc
close all

% Control is done at 60 Hz.
Ts = 1/60;
z = tf('z',Ts);

% Parameter of the filter.
% It is 0.5 for the derivative of the PIDs and 200 for the assignment.
a = 0.5;

H = 1 / (a + 1 - a * z^-1);

step(H)
stepinfo(H)
%bode(H)
