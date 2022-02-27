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

    for t = thetas
        theta = t;
        R = axis_angle_2_rotation(shat,theta);
        p = (eye(3)*theta+(1-cos(theta))*omega_hat+(theta-sin(theta))*omega_hat^2)*v;
        T_t = [R p;0 0 0 1]*T;
        
        R_t = T_t(1:3,1:3);
        P_t = [T_t(1,4), T_t(2,4), T_t(3,4)];

        figure(1)
        % origin frame
        plot3(0, 0, 0, '.k')
        hold on
        line([0 1], [0 0], [0 0], 'Color', 'r')
        line([0 0], [0 1], [0 0], 'Color', 'g')
        line([0 0], [0 0], [0 1], 'Color', 'b')

        % rotated frame
        plot3(P_t(1), P_t(2), P_t(3), '.r')
        line(([0 1]+P_t(1)), ([0 0]+P_t(2)), ([0 0]+P_t(3)), 'Color', 'r');
        line(([0 0]+P_t(1)), ([0 1]+P_t(2)), ([0 0]+P_t(3)), 'Color', 'g');
        line(([0 0]+P_t(1)), ([0 0]+P_t(2)), ([0 1]+P_t(3)), 'Color', 'b');
        hold on

        grid on
        xlim([-1 8])
        ylim([-1 8])
        zlim([-1 8])
        

    end
    Tfinal = 0;

end