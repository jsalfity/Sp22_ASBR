function [q] = J_inverse_kinematics(robot, Ti, Tf, q0, max_iterations)
%J_inverse_kinematics Summary of this function goes here
%   param: robot (struct with n_joints, M, screw_axes, qs)
%   param: Ti   (Initial Transformation)
%   param: Tf   (Final Transformation)
%   param: theta0

%   reference: MR 6.2.2

    % initialization
    q = q0;
    T_sd = Tf;         % Desired configuration in space frame
    T_bs = FK_body(robot, q, 0);

    T_bd = T_bs * T_sd; % Desired configuraiton in body frames

    V_b_skew = logm(T_bd);
    V_b = vector_from_skew(V_b_skew); % function to write
    omega = V_b(1:3);
    v = V_b(4:6);

    i = 0;
    while (norm(omega) > getGlobaleps ...
          || norm(v) > getGlobaleps)  ...
          && i < max_iterations

        q = q + pinv(J_body(robot, q))*V_b;

        T_bs = FK_body(robot, q, 0);
        T_bd = T_bs * T_sd;

        V_b_skew = logm(T_bd);
        V_b = vector_from_skew(V_b_skew);
        omega = V_b(1:3);
        v = V_b(4:6);

        i = i + 1;
    end
end