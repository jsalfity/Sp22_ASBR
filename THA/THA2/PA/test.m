clc; clear all; close all;
set(0, 'DefaultFigureColor', 'w')
eps = 0.01;


% % Panda Robot struct containing relevant kinematic info
run('make_panda.m') % panda robot struct

panda_matlab = loadrobot('frankaEmikaPanda', 'DataFormat','column');

% make 1x9 to make compatible with panda_matlab.
% note that our functions only use 1x7
% q = [0 0 0 0 0 0 0 0 0]';
% q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0, 0]';
q = [1.3342, -0.5504, 0.4871, -2.7483, 2.3544, 3.2988, 1.8413, 0.0104, 0.0238]';

% % singularity test
% display(at_singularity(panda, q))

% plot panda_matlab robot
figure 
axis equal
show(panda_matlab, q, 'Visuals','off')

% FK_space.m
T_final = FK_space(panda, q,  panda.M, 1)
T_final_matlab = getTransform(panda_matlab, q, 'panda_link8')
assert(norm(T_final_matlab - T_final) > eps)

% FK_body.m
T_final = FK_body(panda, q,  panda.M, 1)
T_final_matlab = getTransform(panda_matlab, q, 'panda_link8')
assert(norm(T_final_matlab - T_final) > eps)

% J_space.m
Js = J_space(panda, q)
Js_matlab = geometricJacobian(panda_matlab, q, 'panda_link8')

% J_body.m
Jb = J_body(panda, q);
% assert(norm(Adjoint(panda.M)*Jb - Js) < );


% plot angular manipulability ellipsoid
ellipsoid_plot_angular(panda, q)

% plot linear manipulabilty ellipsoid
ellipsoid_plot_linear(panda, q)


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
[q_inverse, idx_inverse, e_inverse] = J_inverse_kinematics(panda, Ti, Tf, q0, 200);
Jb = J_body(panda, q);

% J_transpose_kinematics
% NOTE!! The Lyapunov method using J_transpose takes significantly long
% iterations to converge than the J_inverse method under this initial and
% final confiugrations. It makes sense since the Lyapunov stability
% condition only guarantees an asymptotic stability. In order to decrease
% the number of iterations, we can also compare the difference of two
% consecutive errors and set a threshold. However, for a fair comparison,
% we might want to keep this consistent with the J_inverse method, and we
% can mention this in our report.
[q_transpose, idx_transpose, e_transpose] = J_transpose_kinematics(panda, Ti, Tf, q0, 0.1, 5000);

% Convergence plot for a more explicit comparison between J transpose and J
% inverse
figure
subplot(1,2,1)
plot(e_inverse)
xlabel('iteration')
ylabel('error norm')
title('J inverse')
subplot(1,2,2)
plot(e_transpose)
xlabel('iteration')
ylabel('error norm')
title('J transpose')

% T_final_matlab = getTransform(panda_matlab, [q;0;0], 'panda_link8')
% assert(norm(Tf - T_final_matlab) < eps)
% figure
% show(panda_matlab, [q;0;0]);


