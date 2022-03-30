clc; clear all; close all;
set(0, 'DefaultFigureColor', 'w')

% Panda Robot struct containing relevant kinematic info
panda.n_joints = 7;
panda.M = [1 0 0 0.088; 0 -1 0 0;0 0 -1 0.926;0 0 0 1];
panda.screw_axes(:,:,1) = [0 -1 0 0;1 0 0 0;0 0 0 0;0 0 0 0];
panda.screw_axes(:,:,2) = [0 0 1 -0.333;0 0 0 0;-1 0 0 0;0 0 0 0];
panda.screw_axes(:,:,3) = [0 -1 0 0;1 0 0 0;0 0 0 0;0 0 0 0];
panda.screw_axes(:,:,4) = [0 0 -1 0.649;0 0 0 0;1 0 0 -0.088;0 0 0 0];
panda.screw_axes(:,:,5) = [0 -1 0 0;1 0 0 0;0 0 0 0;0 0 0 0];
panda.screw_axes(:,:,6) = [0 0 -1 1.033;0 0 0 0;1 0 0 0;0 0 0 0];
panda.screw_axes(:,:,7) = [0 1 0 0;-1 0 0 0.088;0 0 0 0;0 0 0 0];
panda.qs(:,:,1) = [0 0 0.333];
panda.qs(:,:,2) = [0 0 0.333];
panda.qs(:,:,3) = [0 0 0.649];
panda.qs(:,:,4) = [0.088 0 0.649];
panda.qs(:,:,5) = [0 0 1.033];
panda.qs(:,:,6) = [0 0 1.033];
panda.qs(:,:,7) = [0.088 0 1.033];

input_thetas = [0, 0, 0, 0, 0, 0, 0];

% FK_space.m
FK_space(panda, input_thetas);

% FK_body.m
FK_body(panda, input_thetas);