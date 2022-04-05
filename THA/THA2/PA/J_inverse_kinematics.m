function [q, idx, e] = J_inverse_kinematics(robot, Ti, Tf, q0, max_iterations)
%J_inverse_kinematics Summary of this function goes here
%   param: robot (struct with n_joints, M, screw_axes, qs)
%   param: Ti   (Initial Transformation)
%   param: Tf   (Final Transformation)
%   param: theta0

%   reference: MR 6.2.2

    % initialization
    q = q0;
    T_sd = Tf;         % Desired configuration in space frame
    T_bd = FK_body(robot, q, Ti, 0) \  T_sd; % Desired configuraiton in body frame

    V_b_skew = logm(T_bd);
    V_b = vector_from_skew(V_b_skew);
    omega = V_b(1:3);
    v = V_b(4:6);

    idx = 0;
    
    e = zeros(1,max_iterations+1);

    while (norm(omega) > getGlobaleps ...
          || norm(v) > getGlobaleps)  ...
          && idx < max_iterations

        q = q + pinv(J_body(robot, q))*V_b;

        T_bd = FK_body(robot, q, Ti, 0) \  T_sd;

        V_b_skew = logm(T_bd);
        V_b = vector_from_skew(V_b_skew);
        omega = V_b(1:3);
        v = V_b(4:6);

        idx = idx + 1;
        
        e(1,idx+1) = norm(v) + norm(omega);
    end
end