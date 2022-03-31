clc; clear all; close all;
set(0, 'DefaultFigureColor', 'w')

% % Panda Robot struct containing relevant kinematic info
run('make_panda.m') % panda robot struct

input_thetas = [0, 0, 0, 0, 0, 0, 0];
% % FK_space.m
T_final = FK_space(panda, input_thetas)

% % FK_body.m
T_final = FK_body(panda, input_thetas)

% J_space.m
Js = J_space(panda, input_thetas)

% J_body.m
Jb = J_body(panda, input_thetas)

% singularity test

% plot angular manipulability ellipsoid
ellipsoid_plot_angular(panda, input_thetas)

% plot linear manipulabilty ellipsoid
ellipsoid_plot_linear(panda, input_thetas)