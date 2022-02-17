% PA 2a
function R = axis_angle_2_rotation(omega, theta)
    % param: omega (3x1 orientation vector)
    % param: theta (angle [radian])
    % return: R (rotation matrix)
    % reference: MR Proposition 3.11, Example


    omega_hat = [   0         -omega(3)  omega(2);
                 omega(3)       0        -omega(1);
                 -omega(2)    omega(1)     0      ];

    R = eye(3) + sin(theta) * omega_hat + (1 -  cos(theta))*omega_hat.^2;
   
    return
end
