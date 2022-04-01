clc; clear all; close all;
set(0, 'DefaultFigureColor', 'w')

% % Panda Robot struct containing relevant kinematic info
run('make_panda.m') % panda robot struct

panda_matlab = loadrobot('frankaEmikaPanda', 'DataFormat','column');

% make 1x9 to make compatible with panda_matlab.
% note that our functions only use 1x7
% q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0, 0]';
q = [0 0 0 0 0 0 0 0 0]';

% % singularity test
% display(at_singularity(panda, q))

% % % FK_space.m
T_final = FK_space(panda, q, 0)
T_final_matlab = getTransform(panda_matlab, q, 'panda_link8');
% 
% % % FK_body.m
T_final = FK_body(panda, q, 0)
T_final_matlab = getTransform(panda_matlab, q, 'panda_link8');
% 
% % J_space.m
Js = J_space(panda, q)
Js_matlab = geometricJacobian(panda_matlab, q, 'panda_link8');

% % J_body.m
Jb = J_body(panda, q)
Adjoint(panda.M)*Jb
% 
% % plot angular manipulability ellipsoid
% ellipsoid_plot_angular(panda, q)
% 
% % plot linear manipulabilty ellipsoid
% ellipsoid_plot_linear(panda, q)

% J_inverse_kinematics
Ti = [1 0 0 0.088; 
      0 -1 0 0; 
      0 0 -1 0.926; 
      0 0 0 1];
Tf = [1 0 0 0.029;
      0 -1 0 0; 
      0 0 -1 0.8; 
      0 0 0 1];
q0 = [0 0 0 0 0 0 0]';
[q, idx] = J_inverse_kinematics(panda, Ti, Tf, q0, 200)

T_final_matlab = getTransform(panda_matlab, [q;0;0], 'panda_link8')


