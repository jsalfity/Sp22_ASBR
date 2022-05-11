clc; clear all; close all;
set(0, 'DefaultFigureColor', 'w')
eps = 0.001;    % error for convergence alg
tol = 0.003;    % tolerance for robot end effector position

% % Panda Robot struct containing relevant kinematic info
% run('make_panda.m') % panda robot struct
% q_init = [1.33, -0.55, 0.48, -2.74, 2.35, 3.29, 1.84]';
q_init = zeros(7,1);

% panda robot joint limits
% https://frankaemika.github.io/docs/control_parameters.html
q_ub = [2.8973, 1.7628, 2.8973, -0.00698, 2.8973, 3.7525, 2.8973]';
q_lb = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]';

% make panda from matlab
panda_matlab = loadrobot('frankaEmikaPanda', 'DataFormat','column');
q_rand = randomConfiguration(panda_matlab);
T_rand = getTransform(panda_matlab, ...
                      randomConfiguration(panda_matlab), ...
                      'panda_link8');
run('make_panda.m')
% p_goal = T_rand(1:3, 4);
p_goal = [0.08;0;0.826];

% PA 1a
p_tip = [0 0 0.1 1]';
zeta = 1;
eta = 1;
q_delta = tool_tip_near_point(panda_matlab, p_tip, p_goal, ...
                              q_init, q_ub, q_lb, tol);
q_new = q_init+q_delta;
T_robot = FK_space(panda, q_new,  panda.M, 1);
% T_robot = getTransform(panda_matlab, q_robot, 'panda_link8');
% T_robot = FK_space(panda, q_robot, panda.M, 0);
T_tip = T_robot * p_tip;

T_tip 
p_goal

dis_error = sqrt((T_tip(1)-p_goal(1))^2+(T_tip(2)-p_goal(2))^2+(T_tip(3)-p_goal(3))^2)


% Plot the tip position after tranformation and the allowed region
% Make unit sphere
[x,y,z] = sphere;
% Scale to desire radius.
radius = tol;
x = x * radius;
y = y * radius;
z = z * radius;
% Plot as surface.
figure
surf(x+p_goal(1),y+p_goal(2),z+p_goal(3)) 
hold on
plot3(T_tip(1),T_tip(2),T_tip(3),'ro')

% Label axes.
xlabel('X', 'FontSize', 20);
ylabel('Y', 'FontSize', 20);
zlabel('Z', 'FontSize', 20);
axis equal;


% % plot panda_matlab robot
% figure
% axis equal
% show(panda_matlab, q_robot)
% % FK_space(panda, q_robot, panda.M, 1)
% hold on;
% plot3(p_goal(1), p_goal(2), p_goal(3), 'rx')
% title('Matlab Robotics Systems Toolbox Visualization')
