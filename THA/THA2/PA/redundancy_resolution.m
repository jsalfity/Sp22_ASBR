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
    q = q0;
    T_sd = Tf;         % Desired configuration in space frame
    T_bd = FK_body(robot, q, Ti, 0) \  T_sd; % Desired configuraiton in body frame

    V_b_skew = logm(T_bd);
    V_b = vector_from_skew(V_b_skew);
    omega = V_b(1:3);
    v = V_b(4:6);

    idx = 0;

    e = zeros(1, max_iterations+1);

    while (norm(omega) > getGlobaleps ...
          || norm(v) > getGlobaleps)  ...
          && idx < max_iterations

        % calculate useful quantities to be used in update equation
        Jb = J_body(robot, q);
        A = Jb * Jb';
        dot_q = K * ((1/2) * ( 1/sqrt(det(A)) ) ... 
                        * det(A) * trace(pinv(A) * gradient(A)))';

        % update equation
        q = q + pinv(Jb)*V_b + ...
            (eye(robot.n_joints) - pinv(Jb)*Jb) * dot_q;

        T_bd = FK_body(robot, q, Ti, 0) \  T_sd;

        V_b_skew = logm(T_bd);
        V_b = vector_from_skew(V_b_skew);
        omega = V_b(1:3);
        v = V_b(4:6);

        idx = idx + 1;

        e(1,idx+1) = norm(v) + norm(omega);
    end
end