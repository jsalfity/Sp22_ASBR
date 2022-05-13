function [q_curr,tip_pos_all] = virtual_wall(p_tip, p_goal, q_init, ...
    q_ub, q_lb, tol,n_vec,wall_loc,d_wall)
% tool_tip_near_point Place robot and new tool tip near point p_gal
%   param p_tip: new tool tip
%   param p_goal: goal tool tip
%   param q_init: initial joint configuration
%   param q_ub: upper bound for joint limit
%   param q_lb: lower bound for joint limit
%   param tol: tolerance to keep the tip within
%   param n: normal vector of the virtual wall
%   param d: minimum distance from the virtual wall

%   return q_delta: joint command

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

i = 1;

while error >= tol
    figure(1)
%     plot3(p_goal(1),p_goal(2),p_goal(3),'rx','LineWidth',8)
    show(robot, [q_init;0;0])
%     hold on
    T_robot = FK_space(panda, q_init,  panda.M, 0);
    R = T_robot(1:3, 1:3);
    t = T_robot * p_tip;
    t = t(1:3);
    
    J_s = J_space(panda, q_init);

    J_alpha = J_s(1:3, :);    % rotational
    J_eps = J_s(4:6, :);      % translation
    
    C = -vec_2_skew_mat(t)*J_alpha + J_eps;

    d = (t - p_goal);

    d = [d;0;0;0];

    b = tol * ones(m, 1) - A*d;

    J_b = J_body(panda, q_init);

    J_alpha = J_b(1:3, :);    % rotational
    J_eps = J_b(4:6, :);      % translation
    
    p_tip = [0;0;0.1;1];
    t_new = T_robot*p_tip;
    t_new = t_new(1:3);
    b2 = -(d_wall+n_vec*wall_loc-n_vec*t_new);
    A2 = -n_vec*R*(-vec_2_skew_mat([0;0;0.1])*J_alpha + J_eps);

    A_ = [A*J_s;A2];
    B = [b;b2];
    
    options = optimoptions('lsqlin','Algorithm','active-set','MaxIterations',2000,'TolCon',1e-08);
    q_delta = lsqlin(C, -d(1:3), A_, B, [], [], q_lb-q_init, q_ub-q_init,zeros(7,1),options);
    
    q_init = q_init+q_delta;
    T_robot = FK_space(panda, q_init,  panda.M, 0);
    T_tip = T_robot * p_tip;
    error = norm(T_tip(1:3)-p_goal);
    i = i+1;
    if i>=100
        break
    end
    tip_pos_all = [tip_pos_all,T_tip(1:3)];
%     show(panda_matlab, [q_init;0;0])
%     hold on
    pause(0.01)
end

hold on
w = null(n_vec); % Find two orthonormal vectors which are orthogonal to v
[P,Q] = meshgrid(-1.5:1.5); % Provide a gridwork (you choose the size)
wall_loc(3) = wall_loc(3)-d_wall;
X = wall_loc(1)+w(1,1)*P+w(1,2)*Q; % Compute the corresponding cartesian coordinates
Y = wall_loc(2)+w(2,1)*P+w(2,2)*Q; %   using the two vectors in w
Z = wall_loc(3)+w(3,1)*P+w(3,2)*Q;
surf(X,Y,Z)

q_curr = q_init;
end