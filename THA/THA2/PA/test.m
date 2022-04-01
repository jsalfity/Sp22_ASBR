clc; clear all; close all;
set(0, 'DefaultFigureColor', 'w')

% % Panda Robot struct containing relevant kinematic info
run('make_panda.m') % panda robot struct
q = zeros(1,7);

panda_matlab = loadrobot('frankaEmikaPanda', 'DataFormat','row');
q_matlab = zeros(1,9);

% % FK_space.m
T_final = FK_space(panda, q, 0)
T_final_matlab = getTransform(panda_matlab, q_matlab, 'panda_link8')

% % FK_body.m
T_final = FK_body(panda, q, 0)
T_final_matlab = getTransform(panda_matlab, q_matlab, 'panda_link8')

% J_space.m
Js = J_space(panda, q)
Js_matlab = geometricJacobian(panda_matlab, q_matlab, 'panda_link8')

% J_body.m
Jb = J_body(panda, q)
Jb_matlab = geometricJacobian(panda_matlab, q_matlab, 'panda_link8')

% singularity test


% plot angular manipulability ellipsoid
ellipsoid_plot_angular(panda, q)

% plot linear manipulabilty ellipsoid
ellipsoid_plot_linear(panda, q)

