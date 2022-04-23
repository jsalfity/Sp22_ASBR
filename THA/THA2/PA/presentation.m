clc; clear all; close all;
set(0, 'DefaultFigureColor', 'w')
eps = 0.1; % I have to make this larger, otherwise the FK test will fail

%% Panda Robot struct containing relevant kinematic info
run('make_panda.m') % panda robot struct

panda_matlab = loadrobot('frankaEmikaPanda', 'DataFormat','column');

q0 = [0 0 0 0 0 0 0]';

Ti = [1 0 0 0.088; 
      0 -1 0 0; 
      0 0 -1 0.926; 
      0 0 0 1];

% qf = randomConfiguration(panda_matlab);
% Tf_matlab = getTransform(panda_matlab, qf, 'panda_link8');
Tf = [1 0 0 0.088; 
      0 -1 0 0.5; 
      0 0 -1 0.626; 
      0 0 0 1];

% %% J Inverse
% % [q_inverse, e_inverse] = J_inverse_kinematics(panda, Ti, ...
% %                                                      Tf, ...
% %                                                      q0, 200);
% % 
% % % q_inverse will not always be qf
% % % but Tf should be Tf_matlab
% % Tf_inverse = FK_space(panda, q_inverse(:, end), Ti, 0);
% % assert(norm(Tf - Tf_inverse) < eps)
% % 
% % for idx = 1:size(q_inverse, 2)
% %     figure(1)
% %     show(panda_matlab, [q_inverse(:, idx);0;0]);
% % 
% %     % plot angular manipulability ellipsoid
% %     ellipsoid_plot_angular(panda, q_inverse(:, idx), 0, 2);
% %     % plot linear manipulabilty ellipsoid
% %     ellipsoid_plot_linear(panda, q_inverse(:, idx), 0, 3);
% % 
% % end
% 
%% J Transpose
% [q_transpose, e_transpose] = J_transpose_kinematics(panda, Ti, Tf, ...
%                                                     q0, 0.1, 5000);
% for idx = 1:10:size(q_transpose, 2)
%     figure(2)
%     show(panda_matlab, [q_transpose(:, idx);0;0]);
% 
%     % plot angular manipulability ellipsoid
%     ellipsoid_plot_angular(panda, q_transpose(:, idx), 0, 4)
%     % plot linear manipulabilty ellipsoid
%     ellipsoid_plot_linear(panda, q_transpose(:, idx), 0, 5)
% 
%     pause(.01)
% end
% 
% Tf_transpose = FK_space(panda, q_transpose(:,end), Ti, 0);
% assert(norm(Tf - Tf_transpose) < eps)

%% REDUDANCY RESOLUTION
[q_rr, idx_rr, e_rr] = redundancy_resolution(panda, Ti, Tf, ...
                                             q0, 200, 2*eye(7));
Tf_rr = FK_space(panda, q_rr(:,end), Ti, 0);
assert(norm(Tf - Tf_rr) < eps)

for idx = 1:size(q_rr, 2)
    figure(2)
    show(panda_matlab, [q_rr(:, idx);0;0]);

    % plot angular manipulability ellipsoid
    ellipsoid_plot_angular(panda, q_rr(:, idx), 0, 4)
    % plot linear manipulabilty ellipsoid
    ellipsoid_plot_linear(panda, q_rr(:, idx), 0, 5)
    
    if idx == 1
        pause(40)
    else
        pause(0.01)
    end
end
