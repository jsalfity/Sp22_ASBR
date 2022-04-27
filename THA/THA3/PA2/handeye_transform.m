function [R_x, p_x, T_x] = handeye_transform(qA, pA, qB, pB)
%handeye_transform Perform Eye In Hand Calibration, AX=XB Algorithm
    % param: qA (nx4 quaternion measurements)
    % param: pA (nx3 translation vector measurements)
    % param: qB (nx4 quaternion measurements)
    % param: pB (nx3 translation vector measurements)

    % return: Rx (Optimal 3x3 Rotation Matrix)
    % return: px (Optimal 1x3 translation vector)
    % return: Tx (Optimal Transformation Matrix)

    % reference: ASBR W12L1 Course Notes

    % Check the data for proper dimensions
    if size(qA, 2) ~= 4 || size(qB, 2) ~= 4 ...         % proper quaternion
            || size(pA, 2) ~= 3 || size(pA, 2) ~= 3 ... % proper trans vec
            || size(qA, 1) ~= size(pA, 1) ...           % consistent n_obs
            || size(qB, 1) ~= size(pB, 1) ...           % consistent n_obs
            || size(qB, 1) ~= size(qA, 1)               % consistent n_obs
        assert
    end


    n_observations = size(qA, 1);

    M = [];
    for n = 1:n_observations

        % grab info
        sA = qA(n, 1);
        vA = qA(n, 2:end)';     % transpose to column vector

        sB = qB(n, 1);
        vB = qB(n, 2:end)';     % transpose to column vector

        % build M_qAqB
        M_qAqB = zeros(4);
        M_qAqB(1, 1) = sA-sB;
        M_qAqB(1, 2:end) = -(vA-vB)';
        M_qAqB(2:end, 1) = vA-vB;
        M_qAqB(2:end, 2:end) = (sA-sB)*eye(3) + vec_2_skew_mat(vA+vB);

        M = [M;
             M_qAqB];

    end

    % quaternion is 4th column of e-vector
    [U, S, V] = svd(M);
    q_x = V(:,4);
    R_x = quaternion_2_rotation(q_x);

    % least squares, solving Ax = b
    A = zeros(3*n_observations, 3);
    b = zeros(3*n_observations, 1);
    for n = 1:n_observations

        idx = 3*(n-1)+1;
        A(idx:(idx+2), 1:3) = quaternion_2_rotation(qA(n,:))-eye(3);

        vA = qA(n, 2:end)';     % transpose to column vector
        vB = qB(n, 2:end)';     % transpose to column vector
        b(idx:(idx+2), 1) = R_x*vB - vA;
    end

    p_x = lsqr(A, b);

    T_x = zeros(4);
    T_x(4,4) = 1;
    T_x(1:3, 1:3) = R_x;
    T_x(1:3, 4) = p_x;
end
