% PA 1a
function [omega, theta] = axis_angle(R)
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
        omega = (R - R') / (2 * sin(theta));
    end
 
    return 
end
