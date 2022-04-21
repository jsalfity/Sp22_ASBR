function [q, e] = J_transpose_kinematics(robot, Ti, Tf, q0, alpha, max_iterations)
% J_transpose_kinematics Summary of this function goes here
%   param: robot (struct with n_joints, M, screw_axes, qs)
%   param: Ti   (Initial Transformation)
%   param: Tf   (Final Transformation)
%   param: q0   (initial guess of joint angles)
%   param: alpha   (step size)


    q = [q0];
    T_sd = Tf;         % Desired configuration in space frame
    T_bd = FK_body(robot, q0, Ti, 0) \  T_sd; % Desired configuraiton in body frame
    
    V_b_skew = logm(T_bd);
    V_b = vector_from_skew(V_b_skew);
    omega = V_b(1:3);
    v = V_b(4:6);
    
    K = alpha*eye(6);
    
    idx = 0;
    
    q_dot = zeros(max_iterations+1, robot.n_joints);
    
    e = zeros(1,max_iterations+1);
    
    while (idx <= max_iterations) && (norm(omega) > getGlobaleps ...
            || norm(v) > getGlobaleps)

        q_dot(idx+1, :) = J_body(robot,q(:, end))'*K*V_b;

        q = [q trapz(q_dot)'];
        
        T_bd = FK_body(robot, q(:, end), Ti, 0) \  T_sd;
    
        V_b_skew = logm(T_bd);
        V_b = vector_from_skew(V_b_skew);
        omega = V_b(1:3);
        v = V_b(4:6);
        
        idx = idx + 1;
        
        e(1,idx+1) = norm(v) + norm(omega);
    end

