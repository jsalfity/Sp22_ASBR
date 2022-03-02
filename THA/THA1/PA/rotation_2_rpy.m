% PA 1c
function [phi, theta, psi] = rotation_2_rpy(R, theta_in)
    % param: R (3x3 rotation matrix)
    % return: phi
    % return: theta
    % return: psi 
    % reference: ASBR W3L1 Pg7

    % check R is in SO(3)
    if ~RinSO3(R)
        phi = nan;
        theta = nan;
        psi = nan;
        return
    end

    if theta_in > -pi/2 && theta_in < pi/2
        phi = atan2(R(2,1), R(1,1));
        theta = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
        psi = atan2(R(3,2), R(3,3));
    elseif theta_in > pi/2 && theta_in < 3*pi/2
        phi = atan2(-R(2,1), -R(1,1));
        theta = atan2(-R(3,1), -sqrt(R(3,2)^2 + R(3,3)^2));
        psi = atan2(-R(3,2), -R(3,3));

    end

end