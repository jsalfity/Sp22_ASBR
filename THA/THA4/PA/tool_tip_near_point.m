function [q_curr] = tool_tip_near_point(robot, p_tip, p_goal, q_init, ...    
                                            q_ub, q_lb, eps, tol);
%tool_tip_near_point Place robot and new tool tip near point p_gal
%   param robot: robot configuration
%   param p_tip: new tool tip
%   param p_goal: goal tool tip
%   param q_init: initial joint configuration
%   param q_ub: upper bound for joint limit
%   param q_lb: lower bound for joint limit
%   param eps: epsilon for algorithm convergence
%   param tol: tolerance to keep the tip within

%   return q_curr: joint angles

    % set q_current
    q_curr = q_init;

    % iterate through joint angles
    for i = 1:100

        % Find tooltip position vector 
        T_robot = getTransform(robot, q_curr, 'panda_link8');
%         T_robot = FK_space(robot, q_curr, robot.M, 0);

        t = T_robot * p_tip;
        t = t(1:3);
      
        if norm(t - p_goal) < eps
            % found it !
            break
        end
 
        % space jacobian
        J = geometricJacobian(robot, q_curr, 'panda_link8');
%         J = J_space(robot, q_curr);
        J_alpha = J(1:3, :);    % rotational
        J_eps = J(4:6, :);      % translation
        
        % Matrices for Optimization
        % cost function
        C = -vec_2_skew_mat(t)*J_alpha + J_eps;
        d = -(t - p_goal); % negative to account for ||Cx - d||

        % constraint
        % TODO how do we set this up?
%         A = zeros(robot.n_joints, robot.n_joints);
%         b = ones(robot.n_joints, 1);
      
        % solve lsqr with linear constraints
        q_delta = lsqr(C, d);
%         q_delta = lsqlin(C, d, A, b, [], [], q_lb-q_curr, q_ub-q_curr);
        q_curr = q_curr + q_delta;
        
    end

end
