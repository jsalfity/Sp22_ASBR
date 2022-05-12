clc; clear all; close all;
set(0, 'DefaultFigureColor', 'w')
eps = 0.001;    % error for convergence alg
tol = 0.003;    % tolerance for robot end effector position
run('make_panda.m')

% % Panda Robot struct containing relevant kinematic info
% run('make_panda.m') % panda robot struct
q_init = [1.33 -0.55 0.48 -2.74 2.35 3.29 1.84]';
% q_init = zeros(7,1);

% panda robot joint limits
% https://frankaemika.github.io/docs/control_parameters.html
q_ub = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]';
q_lb = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]';

% make panda from matlab
panda_matlab = loadrobot('frankaEmikaPanda', 'DataFormat','column');
q_rand = randomConfiguration(panda_matlab);
T_rand = getTransform(panda_matlab, ...
                      randomConfiguration(panda_matlab), ...
                      'panda_link8');
% q_init = q_rand(1:7);

p_tip = [0;0;0.1;1];

T_robot_init = FK_space(panda, q_init, panda.M, 0);
p_init = T_robot_init*p_tip;
p_goal = p_init(1:3)+[0.15;0.15;0.15]; % For some reason this works only
% for PA 1a
% p_goal = p_init(1:3)+[0.12;0.15;0.15]; % For some reason this works only
% for PA 1b
i = 0;

%% PA 1a
[q_final,orientation_1] = tool_tip_near_point(p_tip, p_goal, q_init, q_ub, q_lb, tol);

hold on
f1 = plot3(p_goal(1),p_goal(2),p_goal(3),'rx','LineWidth',8);
hold on
f2 = plot3(p_init(1),p_init(2),p_init(3),'bx','LineWidth',8);

legend([f1 f2],'Target','Initial')
    
% T_robot = getTransform(panda_matlab, [q_new;0;0], 'panda_link8');

T_robot = FK_space(panda, q_final,  panda.M, 0);
T_tip = T_robot * p_tip;
error = norm(T_tip(1:3)-p_goal);

disp(['Distance between the transformed tip to the goal point is ',...
    num2str(error)])

disp('The joint angle is ')
q_final

%% PA 1b
w_p = 1*eye(3); % Weights for position
w_a = 5*eye(3); % Weights for orientation

p_goal = p_init(1:3)+[0.15;0.15;0.15];

[q_final,orientation_2] = min_orientation_change(p_tip, p_goal, q_init, q_ub, q_lb, tol, w_p, w_a);

hold on
f1 = plot3(p_goal(1),p_goal(2),p_goal(3),'rx','LineWidth',8);
hold on
f2 = plot3(p_init(1),p_init(2),p_init(3),'bx','LineWidth',8);

legend([f1 f2],'Target','Initial')
    
% T_robot = getTransform(panda_matlab, [q_new;0;0], 'panda_link8');

T_robot = FK_space(panda, q_final,  panda.M, 0);
T_tip = T_robot * p_tip;
error = norm(T_tip(1:3)-p_goal);

disp(['Distance between the transformed tip to the goal point is ',...
    num2str(error)])

disp('The joint angle is ')
q_final

figure
f1 = plot(orientation_1);
hold on
f2 = plot(orientation_2);
legend([f1 f2],['Part 1','Part 2'])

%% PA 1c
% q_final = tool_tip_near_point(p_tip, p_goal, q_init, q_ub, q_lb, tol);
% 
% hold on
% f1 = plot3(p_goal(1),p_goal(2),p_goal(3),'rx','LineWidth',8);
% hold on
% f2 = plot3(p_init(1),p_init(2),p_init(3),'bx','LineWidth',8);
% 
% legend([f1 f2],'Target','Initial')
%     
% % T_robot = getTransform(panda_matlab, [q_new;0;0], 'panda_link8');
% 
% T_robot = FK_space(panda, q_final,  panda.M, 0);
% T_tip = T_robot * p_tip;
% error = norm(T_tip(1:3)-p_goal);
% 
% disp(['Distance between the transformed tip to the goal point is ',...
%     num2str(error)])
% 
% disp('The joint angle is ')
% q_final