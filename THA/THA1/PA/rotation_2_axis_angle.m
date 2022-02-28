% PA 1a
function [theta, omega] = rotation_2_axis_angle(R)
    % param: R (3x3 rotation matrix)
    % return: omega, theta (axis-angle representation)
    % reference: ASBR notes, W2L2, slide 9. MR Section 3.2

    % check R is in SO(3)
    if (norm(R) - 1)        > getGlobaleps || ...
        norm(R*R' - eye(3)) > getGlobaleps || ...
        (det(R) - 1)    > getGlobaleps

        theta = nan;
        omega = nan;
        return
    end

    if R == eye(3)
        theta = 0;
        omega = NaN;
      
    elseif sum(diag(R)) == -1
        theta = pi;
        if R(3,3) ~= -1
            omega = (1/sqrt(2*(1+R(3,3)))) * [R(1,3); R(2,3); 1+R(3,3)];
        elseif R(2,2) ~= -1
            omega = (1/sqrt(2*(1+R(2,2)))) * [R(1,2); 1+R(2,2); R(3,2)];
        else %R(1,1) ~= 0
            omega = (1/sqrt(2*(1+R(1,1)))) * [1+R(1,1); R(2,1); 1+R(3,1)];
        end
    else
        theta = acos(0.5 * (sum(diag(R)) - 1 ));
        omega_hat = (R - R') / (2 * sin(theta)); % screw symmetric matrix
        omega_1 = omega_hat(3,2);
        omega_2 = omega_hat(1,3);
        omega_3 = omega_hat(2,1);
        
        omega = [omega_1; omega_2; omega_3];
       
    end
 
    return 
end
