function [q_curr,orientation,tip_pos_all] = tool_tip_near_point(p_tip, p_goal, q_init, ...
    q_ub, q_lb, tol)
% tool_tip_near_point Place robot and new tool tip near point p_gal
%   param p_tip: new tool tip
%   param p_goal: goal tool tip
%   param q_init: initial joint configuration
%   param q_ub: upper bound for joint limit
%   param q_lb: lower bound for joint limit
%   param tol: tolerance to keep the tip within

%   return q_delta: joint command
%   return orientation: change of tip orientation

%   reference: ASBR W16 L1-L2

error = 1;
tip_pos_all = [];

robot = loadrobot('frankaEmikaPanda', 'DataFormat','column');
run('make_panda.m')

A = [];
m = 100; %number of vertices for polygon
n = 100; % number of vertices for polygon
for i=1:m
    A = [A; [cos(i*2*pi/n)*cos(i*2*pi/m) cos(i*2*pi/n)*sin(i*2*pi/m) sin(i*2*pi/n) 0 0 0]];
end

% Plot the virtual wall
n_vec = [0 0 1];
wall_loc = [0;0;0.45];

orientation = [];

while error >= tol
    figure(1)
    show(robot, [q_init;0;0])
    % Tranformation matrix at the current pose
    T_robot = FK_space(panda, q_init,  panda.M, 0);
    R = T_robot(1:3, 1:3);
    t = T_robot * p_tip;
    t = t(1:3);
    % space jacobian at the current pose
    J = J_space(panda, q_init);

    J_alpha = J(1:3, :);    % rotational
    J_eps = J(4:6, :);      % translation
    
    % Distance between the new tool tip and goal
    C = -vec_2_skew_mat(t)*J_alpha + J_eps;
    d = (t - p_goal);

    d = [d;0;0;0];
    
    % From polygon approximation
    b = tol * ones(m, 1) - A*d;
    
    options = optimoptions('lsqlin','Algorithm','active-set','MaxIterations',2000,'TolCon',1e-08);
    q_delta = lsqlin(C, -d(1:3), A*J, b, [], [], q_lb-q_init, q_ub-q_init,zeros(7,1),options);
    
    q_init = q_init+q_delta;
    T_robot = FK_space(panda, q_init,  panda.M, 0);
    T_tip = T_robot * p_tip;
    error = norm(T_tip(1:3)-p_goal);
    tip_pos_all = [tip_pos_all,T_tip(1:3)];

    i = i+1;
    if i>=10000
        break
    end

    pause(0.01)

    C2 = -vec_2_skew_mat(R*[0;0;1])*J_alpha;
    orientation = [orientation;norm(C2*q_delta)];
end

hold on
w = null(n_vec); % Find two orthonormal vectors which are orthogonal to v
[P,Q] = meshgrid(-1.5:1.5); % Provide a gridwork (you choose the size)
X = wall_loc(1)+w(1,1)*P+w(1,2)*Q; % Compute the corresponding cartesian coordinates
Y = wall_loc(2)+w(2,1)*P+w(2,2)*Q; %   using the two vectors in w
Z = wall_loc(3)+w(3,1)*P+w(3,2)*Q;
surf(X,Y,Z)

q_curr = q_init;
end