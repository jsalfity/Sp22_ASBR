function [q_delta] = min_orientation_change(p_tip, p_goal, q_init, ...
    q_ub, q_lb, tol, w_p, w_a)
% tool_tip_near_point Place robot and new tool tip near point p_gal
%   param p_tip: new tool tip
%   param p_goal: goal tool tip
%   param q_init: initial joint configuration
%   param q_ub: upper bound for joint limit
%   param q_lb: lower bound for joint limit
%   param tol: tolerance to keep the tip within
%   param w_p: weight for position matching
%   param w_a: weight for orientation-change minimization

%   return q_delta: joint command

%   reference: ASBR W16 L1-L2

error = 1;

robot = loadrobot('frankaEmikaPanda', 'DataFormat','column');
run('make_panda.m')

A = [];
m = 100; %number of vertices for polygon
n = 100; % number of vertices for polygon
for i=1:m
    A = [A; [cos(i*2*pi/n)*cos(i*2*pi/m) cos(i*2*pi/n)*sin(i*2*pi/m) sin(i*2*pi/n) 0 0 0]];
end

while error >= tol
    figure(2)
%     plot3(p_goal(1),p_goal(2),p_goal(3),'rx','LineWidth',8)
    show(robot, [q_init;0;0])
%     hold on
    T_robot = FK_space(panda, q_init,  panda.M, 0);
    R = T_robot(1:3, 1:3);
    t = T_robot * p_tip;
    t = t(1:3);
    
    J = J_space(panda, q_init);

    J_alpha = J(1:3, :);    % rotational
    J_eps = J(4:6, :);      % translation
    
    C1 = -vec_2_skew_mat(t)*J_alpha + J_eps;
    C2 = -vec_2_skew_mat(R*[0;0;1])*J_alpha;
    d = (t - p_goal); % negative to account for ||Cx - d||

    d = [d;0;0;0];

    b = tol * ones(m, 1) - A*d;
    
    options = optimoptions('lsqlin','Algorithm','active-set','MaxIterations',2000,'TolCon',1e-20);
    q_delta = lsqlin(w_p*C1+w_a*C2, -w_p*d(1:3), A*J, b, [], [], q_lb-q_init, q_ub-q_init,zeros(7,1),options);
    
    q_init = q_init+q_delta;
    T_robot = FK_space(panda, q_init,  panda.M, 0);
    T_tip = T_robot * p_tip;
    error = norm(T_tip(1:3)-p_goal);
    i = i+1;
    if i>=10000
        break
    end
%     show(panda_matlab, [q_init;0;0])
%     hold on
    pause(0.01)
end
q_curr = q_init;
end