function [Tfinal] = screw(T, q, shat, h, thetas)
    % param: T (4x4 transformation matrix)
    % param: s (screw axis  [q, shat, h] in fixed frame s) 
    % param: thetas (range of distances traveled along screw axis theta)

    % return: Tfinal

    % reference: ASBR W5L1
    Tfinal = nan;
    S = [shat; -cross(shat,q)+h*shat];
    omega = S(1:3);
    v = S(4:6);

    omega_hat =  [   0         -omega(3)  omega(2);
                 omega(3)       0        -omega(1);
                 -omega(2)    omega(1)     0      ];


    figure(1)
    % origin frame
    plot_handle(1) = plot3(0, 0, 0, '.k')
    hold on
    plot_handle(2) = line([0 1], [0 0], [0 0], 'Color', 'r', 'LineStyle', '--');
    plot_handle(3) =line([0 0], [0 1], [0 0], 'Color', 'g', 'LineStyle', '--');
    plot_handle(4) = line([0 0], [0 0], [0 1], 'Color', 'b', 'LineStyle', '--');

    for theta_ = thetas
        R = axis_angle_2_rotation(shat,theta_);
        p = (eye(3)*theta_+(1-cos(theta_))*omega_hat+(theta_-sin(theta_))*omega_hat^2)*v;
        T_t = [R p;0 0 0 1]*T;

        R_t = T_t(1:3,1:3);
        P_t = [T_t(1,4), T_t(2,4), T_t(3,4)];
        P_x = P_t(1);
        P_y = P_t(2);
        P_z = P_t(3);

        % plot rotated frame
        plot_handle(5) = line(([0 R_t(1,1)]+P_x), ...
                              ([0 R_t(2,1)]+P_y), ...
                              ([0 R_t(3,1)]+P_z), ...
                              'Color', 'r', 'LineWidth', 2);
        plot_handle(6) = line(([0 R_t(1,2)]+P_x), ...
                              ([0 R_t(2,2)]+P_y), ...
                              ([0 R_t(3,2)]+P_z), ...
                              'Color', 'g', 'LineWidth', 2);
        plot_handle(7) = line(([0 R_t(1,3)]+P_x), ...
                              ([0 R_t(2,3)]+P_y), ...
                              ([0 R_t(3,3)]+P_z), ...
                              'Color', 'b', 'LineWidth', 2);
        hold on

        if theta_ == thetas(end)
            Tfinal = T_t;
        end
    end

    % plot screw axis for Tfinal
    % origin
    T0 = [1 0 0 0;
          0 1 0 0;
          0 0 1 0;
          0 0 0 1];

%     T_1 = T0 * inv(Tfinal);
    T_1 = Tfinal;           % isn't this correct ?
    R_1 = T_1(1:3,1:3);

    if R_1 == eye(3)
        omega = [0;0;0];
        v_norm = sqrt(T_01(1,4)^2+T_01(2,4)^2+T_1(3,4)^2);
        v = T_1(1:3,4)/v_norm;
        theta = v_norm;
        theta_dot = v_norm;

        h = omega'*v/theta_dot;
        q = cross(omega,v)/theta_dot;

    else
        [theta, omega] = rotation_2_axis_angle(R_1);
        w = [0 -omega(3) omega(2);
             omega(3) 0 -omega(1);
             -omega(2) omega(1) 0];
        G_inv = 1/theta*eye(3) - 1/2*w + (1/theta-1/2*cot(theta/2))*w^2;
        v = G_inv * T_1(1:3,4);
        omega_norm = (omega(1)^2 + omega(2)^2 + omega(3)^2)^(1/2);
        omega = omega/omega_norm;
        v = v/omega_norm;
        theta_dot = omega_norm;

        h = omega'*v/theta_dot;
        q = cross(omega,v)/theta_dot;
    end

    s1 = [omega;v];
    % theta = theta;

    disp('screw axis: ')
    disp(s1)
    disp('theta: ')
    disp(theta)

    x1 = q(1);
    y1 = q(2);
    z1 = q(3);

    if omega == [0;0;0]
        omega = v;
    end

    x2 = q(1) + omega(1);
    y2 = q(2) + omega(2);
    z2 = q(3) + omega(3);

    nx = x2 - x1;
    ny = y2 - y1;
    nz = z2 - z1;
    len = 1000;
    xx = [x1 - len*nx, x2 + len*nx];
    yy = [y1 - len*ny, y2 + len*ny];
    zz = [z1 - len*nz, z2 + len*nz];
    plot_handle(8) = plot3(xx, yy, zz,'--','Color','b');
    hold on
    plot_handle(9) = plot3(q(1),q(2),q(3),'o','Color','r');

    xlim([-3 8])
    ylim([-3 8])
    zlim([0 8])

    xlabel('x-axis')
    ylabel('y-axis')
    zlabel('z-axis')
    grid on
    legend(plot_handle([1, 2, 3, 4, 5, 6, 7, 8, 9]), ...
           {'Global Origin', ...
           'Global Frame x-axis', ...
           'Global Frame y-axis', ...
           'Global Frame z-axis', ...
           'Body Frame x-axis', ...
           'Body Frame y-axis', ...
           'Body Frame z-axis', ...
           'Screw Axis', ...
           'q'})

end