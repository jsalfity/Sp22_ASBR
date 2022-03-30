function [T] = FK_space(robot, input_thetas)
%FK_space calculates the forward kinematics with matrix exponential
%   param: robot (struct with n_joints, M, screw_axes, qs)
%   param: thetas (1xn_joints array
%   return: T_final

%   reference: MR 4.1.1

    figure(1)
    % plot origin frame
    plot_handle(1) = plot3(0, 0, 0, '.k');
    hold on
    plot_handle(2) = line([0 1], [0 0], [0 0], 'Color', 'r', 'LineStyle', '--');
    plot_handle(3) =line([0 0], [0 1], [0 0], 'Color', 'g', 'LineStyle', '--');
    plot_handle(4) = line([0 0], [0 0], [0 1], 'Color', 'b', 'LineStyle', '--');

    % perform space form of exponential products
    % iterate through list in reverse
    T = robot.M;
    for idx = robot.n_joints: -1: 1
        %%%%%% TODO
        %%%%% plot screw axis for T %%%%%

        s = robot.space.screw_axes(:, :, idx);
        S = screw_axis_2_se3(s);
        % q = robot.space.qs(:, :, idx);
        theta = input_thetas(idx);

        % calculate the next transformation
        T = expm(S*theta)*T;

        R_t = T(1:3,1:3);
        P_t = [T(1,4), T(2,4), T(3,4)];
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

    end
    xlabel('x-axis')
    ylabel('y-axis')
    zlabel('z-axis')
    grid on
    legend(plot_handle([1, 2, 3, 4, 5, 6, 7]), ...
           {'Global Origin', ...
           'Global Frame x-axis', ...
           'Global Frame y-axis', ...
           'Global Frame z-axis', ...
           'Body Frame x-axis', ...
           'Body Frame y-axis', ...
           'Body Frame z-axis', ...
           })

end
