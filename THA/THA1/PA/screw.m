function [T_final, S_final, q_final] = screw(T_input, q, shat, h, thetas)
    % param: T_input (4x4 transformation matrix)
    % param: s (screw axis  [q, shat, h] in fixed frame s) 
    % param: thetas (range of distances traveled along screw axis theta)

    % return: T_final

    % reference: ASBR W5L1
    S_input = [shat; -cross(shat,q)+h*shat];
    omega = S_input(1:3);
    v = S_input(4:6);

    omega_hat =  [   0         -omega(3)  omega(2);
                 omega(3)       0        -omega(1);
                 -omega(2)    omega(1)     0      ];


    figure(1)
    % origin frame
    plot_handle(1) = plot3(0, 0, 0, '.k');
    hold on
    plot_handle(2) = line([0 1], [0 0], [0 0], 'Color', 'r', 'LineStyle', '--');
    plot_handle(3) =line([0 0], [0 1], [0 0], 'Color', 'g', 'LineStyle', '--');
    plot_handle(4) = line([0 0], [0 0], [0 1], 'Color', 'b', 'LineStyle', '--');

    T_final = nan; % return value

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%$$$ plot rotated frames %%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for theta_ = thetas
        R = axis_angle_2_rotation(shat,theta_);
        p = (eye(3)*theta_+(1-cos(theta_))*omega_hat+(theta_-sin(theta_))*omega_hat^2)*v;
        T_t = [R p;0 0 0 1]*T_input;

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
            T_final = T_t;
        end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%% plot screw axis for T_final %%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    R_final = T_final(1:3,1:3);
    p_final = T_final(1:3,4);

    S_final = zeros(6,1); % Return value

    % Reference MR 3.3.3.2 Matrix Logarithm of Rigid-Body Motions
    if R_final == eye(3)
        omega = [0;0;0];
        v = p_final/norm(p_final);
        theta = norm(p_final);
        theta_dot = norm(v);

        h = omega'*v/theta_dot;
        q = cross(omega,v)/theta_dot;

        % screw axis
        S_final = [omega;v];

    else
        [theta, omega] = rotation_2_axis_angle(R_final);
        omega_hat = [0 -omega(3) omega(2);
                     omega(3) 0 -omega(1);
                     -omega(2) omega(1) 0];

        % Rodrigues Formula to obtain axis and angle
        G_inv = 1/theta*eye(3) - 1/2*omega_hat + (1/theta-1/2*cot(theta/2))*omega_hat^2;
        v = G_inv * p_final;
        theta_dot = norm(omega);

        h = omega'*v/theta_dot;
        q = cross(omega,v)/theta_dot;

        % screw axis
        S_final = [omega;v] / norm(omega);
    end

    disp('screw axis: ')
    disp(S_final)
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
    len = 100;
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