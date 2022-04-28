function R = quaternion_2_rotation(q)
    % param: q (4x1 quaternion)
    % return: R (3x3 rotation matrix)
    % reference: ASBR W3L1 pg11, MR Appendix B.3

    q0 = q(1);
    q1 = q(2);
    q2 = q(3);
    q3 = q(4);

    R = zeros(3,3);
    % 1st column
    R(1, 1) = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    R(2, 1) = 2 * (q0*q3 + q1*q2);
    R(3, 1) = 2 * (q1*q3 - q0*q2);

    % 2nd column
    R(1, 2) = 2 * (q1*q2 - q0*q3);
    R(2, 2) = q0*q0 - q1*q1 + q2*q2 - q3*q3;
    R(3, 2) = 2 * (q0*q1 + q2*q3);

    % 3rd column
    R(1, 3) = 2 * (q0*q2 + q1*q3);
    R(2, 3) = 2 * (q2*q3 - q0*q1);
    R(3, 3) = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    return
end