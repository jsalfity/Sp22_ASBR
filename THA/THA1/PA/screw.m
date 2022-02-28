function [Tfinal] = screw(T, q, shat, h, theta)
    % param: T (4x4 transformation matrix)
    % param: s (screw axis  [q, shat, h] in fixed frame s) 
    % param: theta (total distance traveled along screw axis theta)

    % return: Tfinal

    % reference: ASBR W5L1

    S = [shat; -cross(shat,q)+h*shat];
    omega = S(1:3);
    v = S(4:6);

    omega_hat =  [   0         -omega(3)  omega(2);
                 omega(3)       0        -omega(1);
                 -omega(2)    omega(1)     0      ];

    thetas = [0, theta/4,  theta/2, 3*theta/4, theta];

    figure(1)
    % origin frame
    plot3(0, 0, 0, '.k')
    hold on
    line([0 1], [0 0], [0 0], 'Color', 'r', 'LineStyle', '--')
    line([0 0], [0 1], [0 0], 'Color', 'g', 'LineStyle', '--')
    line([0 0], [0 0], [0 1], 'Color', 'b', 'LineStyle', '--')

    for theta_ = thetas
        R = axis_angle_2_rotation(shat,theta_);
        p = (eye(3)*theta_+(1-cos(theta_))*omega_hat+(theta_-sin(theta_))*omega_hat^2)*v;
        T_t = [R p;0 0 0 1]*T;
        
        R_t = T_t(1:3,1:3);
        P_t = [T_t(1,4), T_t(2,4), T_t(3,4)];
        P_x = P_t(1);
        P_y = P_t(2);
        P_z = P_t(3);

        % rotated frame
        plot3(P_t(1), P_t(2), P_t(3), '.r')
        line(([0 R_t(1,1)]+P_x), ([0 R_t(2,1)]+P_y), ([0 R_t(3,1)]+P_z), 'Color', 'r');
        line(([0 R_t(1,2)]+P_x), ([0 R_t(2,2)]+P_y), ([0 R_t(3,2)]+P_z), 'Color', 'g');
        line(([0 R_t(1,3)]+P_x), ([0 R_t(2,3)]+P_y), ([0 R_t(3,3)]+P_z), 'Color', 'b');
        hold on

        grid on
        xlim([-4 8])
        ylim([-4 8])
        zlim([-4 8])

        xlabel("x-axis")
        ylabel("y-axis")
        zlabel("z-axis")
        legend("Global Origin", ...
               "Body Frame x-axis", ...
               "Body Frame y-axis", ...
               "Body Frame z-axis", ...
               "Body Frame Origin")

    end
    Tfinal = 0;

end