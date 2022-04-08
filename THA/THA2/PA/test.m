clc; clear all; close all;
set(0, 'DefaultFigureColor', 'w')
eps = 0.1; % I have to make this larger, otherwise the FK test will fail


% % Panda Robot struct containing relevant kinematic info
run('make_panda.m') % panda robot struct

panda_matlab = loadrobot('frankaEmikaPanda', 'DataFormat','column');

% make 1x9 to make compatible with panda_matlab.
% note that our functions only use 1x7
% q = [0 0 0 0 0 0 0 0 0]';
% q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0, 0]';
q = [1.33, -0.55, 0.48, -2.74, 2.35, 3.29, 1.84, 0.01, 0.02]';

% % singularity test
% display(at_singularity(panda, q))

% plot panda_matlab robot
figure
axis equal
subplot(121)
show(panda_matlab, q)
title('Matlab Robotics Systems Toolbox Visualization')
subplot(122)
show(panda_matlab, q, 'Visuals','off')

% FK_space.m
T_final_s = FK_space(panda, q,  panda.M, 1)
T_final_matlab = getTransform(panda_matlab, q, 'panda_link8')
assert(norm(T_final_matlab - T_final_s) < eps)

% FK_body.m
T_final_b = FK_body(panda, q,  panda.M, 1)
T_final_matlab = getTransform(panda_matlab, q, 'panda_link8')
assert(norm(T_final_matlab - T_final_b) < eps)

% J_space.m
Js = J_space(panda, q)
Js_matlab = geometricJacobian(panda_matlab, q, 'panda_link8')

% J_body.m
Jb = J_body(panda, q);

% Check the correctness of Jacobian calculation using Adjoint
assert(norm(Adjoint(T_final_s)*Jb - Js) < getGlobaleps);

% plot angular manipulability ellipsoid
ellipsoid_plot_angular(panda, q, 1)
% plot linear manipulabilty ellipsoid
ellipsoid_plot_linear(panda, q, 1)


% J_inverse_kinematics
for n_test = 1:5
    % always at home position
    q0 = [0 0 0 0 0 0 0]';
    Ti = [1 0 0 0.088; 
      0 -1 0 0; 
      0 0 -1 0.926; 
      0 0 0 1];

    qf = randomConfiguration(panda_matlab);
    Tf_matlab = getTransform(panda_matlab, qf, 'panda_link8');

    % J_inverse_kinematics
    [q_inverse, idx_inverse, e_inverse] = J_inverse_kinematics(panda, Ti, ...
                                                                Tf_matlab, ...
                                                                q0, 200);
    % q_inverse will not always be qf
    % but Tf should be Tf_matlab
    Tf_inverse = FK_space(panda, q_inverse, Ti, 0);
    assert(norm(Tf_matlab - Tf_inverse) < eps)

    figure
    axis equal
    subplot(121)
    show(panda_matlab, [q0;0;0])
    subtitle('Initial Configuration')
    subplot(122)
    show(panda_matlab, [q_inverse;0;0])
    subtitle('Final Configuration')

    % J_transpose_kinematics
    % NOTE!! The Lyapunov method using J_transpose takes significantly long
    % iterations to converge than the J_inverse method under this initial and
    % final confiugrations. It makes sense since the Lyapunov stability
    % condition only guarantees an asymptotic stability. In order to decrease
    % the number of iterations, we can also compare the difference of two
    % consecutive errors and set a threshold. However, for a fair comparison,
    % we might want to keep this consistent with the J_inverse method, and we
    % can mention this in our report.
    [q_transpose, idx_transpose, e_transpose] = J_transpose_kinematics(panda, ...
                                                                    Ti, Tf_matlab, ...
                                                                    q0, 0.1, 5000);

    % q_inverse will not always be qf
    % but Tf_transpose should be Tf_matlab
    Tf_transpose = FK_space(panda, q_transpose, Ti, 0);
    assert(norm(Tf_matlab - Tf_transpose) < eps)

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

    % REDUDANCY RESOLUTION
    [q_rr, idx_rr, e_rr] = redundancy_resolution(panda, Ti, Tf_matlab, ...
                                                 q0, 200, eye(7));

    % q_inverse will not always be qf
    % but Tf should be Tf_matlab
    Tf_rr = FK_space(panda, q_rr, Ti, 0);
    assert(norm(Tf_matlab - Tf_rr) < eps)
end