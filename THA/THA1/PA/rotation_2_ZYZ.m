% PA 1c
function [phi, theta, psi] = rotation_2_ZYZ(R)
    % param: R (3x3 rotation matrix)
    % return: phi
    % return: theta
    % return: psi 
    % reference: ASBR W3L1 Pg4

   phi = atan2(R(2,3), R(1,3));
   theta = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3));
   psi = atan2(R(3,2), -R(3,1));

end