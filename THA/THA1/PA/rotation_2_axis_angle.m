% PA 1a
function [theta, omega] = rotation_2_axis_angle(R)
    % param: R (3x3 rotation matrix)
    % return: omega, theta (axis-angle representation)
    % reference: ASBR notes, W2L2, slide 9


    if R == eye(3)
        theta = 0;
        omega = NaN;
      
    elseif sum(diag(R)) == -1
        theta = pi;
        omega = (1/sqrt(2*(1+R(3,3)))) * [R(1,2); R(2,3); 1+R(3,3)];

    else
        theta = acos(0.5 * (sum(diag(R)) - 1 ));
        omega_hat = (R - R') / (2 * sin(theta));
        omega_1 = omega_hat(3,2);
        omega_2 = omega_hat(1,3);
        omega_3 = omega_hat(2,1);
        
        omega = [omega_1; omega_2; omega_3];
       
    end
 
    return 
end
