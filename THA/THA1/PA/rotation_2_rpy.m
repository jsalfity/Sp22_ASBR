% PA 1c
function [phi, theta, psi] = rotation_2_rpy(R)
    % param: R (3x3 rotation matrix)
    % return: phi
    % return: theta
    % return: psi 
    % reference: ASBR W3L1 Pg6

    phi = atan2(R(2,1), R(1,1));
    theta = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
    psi = atan2(R(3,2), R(3,3));

end