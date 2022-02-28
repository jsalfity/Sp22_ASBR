% PA 1b
function q = rotation_2_quaternion(R)
    % param: R (rotation matrix)
    % return: q (4x1 quaternion matrix)
    % reference: ASBR W3L1 pg11, MR Appendix B3

    q0 = 0.5 * sqrt(R(1,1) + R(2,2) + R(3,3) + 1);
    q1 = 0.5 * sign(R(3,2) - R(2,3)) * sqrt(R(1,1) - R(2,2) - R(3,3) + 1);
    q2 = 0.5 * sign(R(1,3) - R(3,1)) * sqrt(R(2,2) - R(3,3) - R(1,1) + 1);
    q3 = 0.5 * sign(R(2,1) - R(1,2)) * sqrt(R(3,3) - R(1,1) - R(2,2) + 1);

    q = [q0; q1; q2; q3];
   
    return
end
