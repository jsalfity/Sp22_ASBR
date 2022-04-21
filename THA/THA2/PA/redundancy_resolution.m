function [q, idx, e] = redundancy_resolution(robot, Ti, Tf, q0, max_iterations, K)
%redundancy_resolution Summary of this function goes here
%   param: robot (struct with n_joints, M, screw_axes, qs)
%   param: Ti   (Initial Transformation)
%   param: Tf   (Final Transformation)
%   param: q0   (Initial joint angle guess)
%   param: K    (Weighting matrix for manipulabilty)

%   return: q (final joint angles)
%   return: idx (n_iterations completed)
%   return: e (error array)

%   reference: ASBR W10L1

    % initialization
    q = [q0];
    T_sd = Tf;         % Desired configuration in space frame
    T_bd = FK_body(robot, q, Ti, 0) \  T_sd; % Desired configuraiton in body frame

    V_b_skew = logm(T_bd);
    V_b = vector_from_skew(V_b_skew);
    omega = V_b(1:3);
    v = V_b(4:6);

    idx = 0;

    e = zeros(1, max_iterations+1);

    w_previous = 0;
    while (norm(omega) > getGlobaleps ...
          || norm(v) > getGlobaleps)  ...
          && idx < max_iterations

        % calculate useful quantities to be used in update equation
        Jb = J_body(robot, q(:,end));
        A = Jb * Jb';

        % manipulabilty calculations
        w = sqrt(det(A));
        if size(q,2) < 2
            % first iteration shouldn't divide by zero
            dot_q = zeros(1,robot.n_joints)';
        else
            dwdq = (w - w_previous) / (q(:,end) - q(:,end-1));
            dot_q = K * (dwdq');
        end

        % update equation
        delta_theta = pinv(Jb)*V_b + ...
                      (eye(robot.n_joints) - pinv(Jb)*Jb) * dot_q;
        q = [q q(:, end) + delta_theta];

        T_bd = FK_body(robot, q(:, end), Ti, 0) \  T_sd;

        V_b_skew = logm(T_bd);
        V_b = vector_from_skew(V_b_skew);
        omega = V_b(1:3);
        v = V_b(4:6);

        % store manipulability measure for dw/dq calculation
        w_previous = w;

        idx = idx + 1;

        e(1,idx+1) = norm(v) + norm(omega);
        linear_volume = J_ellipsoid_volume(Jb(4:6,:)*Jb(4:6,:)');
        angular_volume = J_ellipsoid_volume(Jb(1:3,:)*Jb(1:3,:)');

    end
end