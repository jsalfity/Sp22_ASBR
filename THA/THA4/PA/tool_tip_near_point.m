function [q_curr] = tool_tip_near_point(robot, p_tip, p_goal, q_init, ...    
        q_ub, q_lb, ...
        zeta, eta, ...
        eps, tol);
%tool_tip_near_point Place robot and new tool tip near point p_gal
%   param robot: robot configuration
%   param p_tip: new tool tip
%   param p_goal: goal tool tip
%   param q_init: initial joint configuration
%   param q_ub: upper bound for joint limit
%   param q_lb: lower bound for joint limit
%   param zeta: weighting for objective function
%   param eta: weighting for movement
%   param eps: epsilon for algorithm convergence
%   param tol: tolerance to keep the tip within

%   return q_curr: joint angles

%   reference: ASBR W16 L1-L2

        % set q_current
        q_curr = q_init;

        % iterate through joint angles
        for i = 1:100

        % Find tooltip position vector 
        T_robot = getTransform(robot, q_curr, 'panda_link8');   %matlab
        %         T_robot = FK_space(robot, q_curr, robot.M, 0);
        R = T_robot(1:3, 1:3);
        t = T_robot * p_tip;
        t = t(1:3);

        if norm(t - p_goal) < eps
        % found it !
        break
        end

        % space jacobian
        J = geometricJacobian(robot, q_curr, 'panda_link8');    %matlab
        %         J = J_space(robot, q_curr);
        J_alpha = J(1:3, :);    % rotational
        J_eps = J(4:6, :);      % translation

        % Matrices for Optimization
        % cost function
        C = -vec_2_skew_mat(t)*J_alpha + J_eps;
        d = -(t - p_goal); % negative to account for ||Cx - d||

        % no constraints
        %         q_delta = lsqr(C, d);

        % with constraint
        % Implement Virtual Fixture through:
        % Telerobotic Control by Virtual Fixtures for Surgical Applications
        % equation 22.11
        A = [];
        m = 10; %number of vertices for polygon
        n = 10; % number of vertices for polygon
        for i=1:m
        A = [A; [cos(i*2*pi/n)*cos(i*2*pi/m) cos(i*2*pi/n)*sin(i*2*pi/m) sin(i*2*pi/n)]];
        end

        b = tol * ones(m, 1) - A*d;

        %         C = [sqrt(zeta)*C; sqrt(eta)*-vec_2_skew_mat(R(:,3))*J(1:3, :)];
        %         d = [sqrt(zeta)*d; sqrt(eta)*zeros(3, 1)];

        q_delta = lsqlin(C, d, A, b, [], [], q_lb-q_curr, q_ub-q_curr);

        % i'm not sure whats causing this error??
        if size(q_delta, 1) ~= 9
        continue
        end

        q_curr = q_curr + q_delta;

        end

end