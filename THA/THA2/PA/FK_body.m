function [T] = FK_body(robot, q, init_pose, viz)
%FK_body calculates the forward kinematics with matrix exponential in body
% frame
%   param: robot (struct with n_joints, M, screw_axes, qs)
%   param: q (1xn_joint angle array)
%   param: init_pose (initial position of the end effector)
%   param: viz (bool) visualization flag
%   return: T_final

%   reference: MR 4.1.3

    if viz
        figure
        % plot origin frame
        plot_handle(1) = plot3(0, 0, 0, '.k');
        hold on
        plot_handle(2) = line([0 0.1], [0 0], [0 0], ...
                              'Color', 'r', 'LineStyle', '--');
        plot_handle(3) = line([0 0], [0 0.1], [0 0], ...
                               'Color', 'g', 'LineStyle', '--');
        plot_handle(4) = line([0 0], [0 0], [0 0.1], ...
                               'Color', 'b', 'LineStyle', '--');
        axis equal
        grid on
    end
    
       % perform space form of exponential products
    % iterate through list in reverse
    % M for each joint (treat each end joint as an end effector)
    t = zeros(28,4);
    t(1:4,:) = [1 0 0 0;0 1 0 0;0 0 1 0.333;0 0 0 1];
    t(5:8,:) = [1 0 0 0;0 0 1 0;0 -1 0 0.333;0 0 0 1];
    t(9:12,:) = [1 0 0 0;0 1 0 0;0 0 1 0.649;0 0 0 1];
    t(13:16,:) = [1 0 0 0.088;0 0 -1 0;0 1 0 0.649;0 0 0 1];
    t(17:20,:) = [1 0 0 0;0 1 0 0;0 0 1 1.033;0 0 0 1];
    t(21:24,:) = [1 0 0 0;0 0 -1 0;0 1 0 1.033;0 0 0 1];
    t(25:28,:) = robot.M;
    % perform space form of exponential products
    % iterate through list in reverse
    T = init_pose;
    for idx = 1:robot.n_joints
        %%%%%% TODO
        %%%%% plot screw axis for T %%%%%

        s = robot.body.screw_axes(:, :, idx);
        S = screw_axis_2_se3(s);
        % q = robot.qs(:, :, idx);
        theta = q(idx);

        % calculate the next transformation
        T = T*expm(S*theta);
        
    end

    if viz
        for idx = 1: robot.n_joints
            t_1 = t((idx-1)*4+1:idx*4,:);
            for y = 1:idx
                s = robot.body.screw_axes(:, :, y);
                S = screw_axis_2_se3(s);
                theta = q(y);
                t_1 = t_1*expm(S*theta);
            end
            R_t = t_1(1:3,1:3);
            P_t = [t_1(1,4), t_1(2,4), t_1(3,4)];
            P_x = P_t(1);
            P_y = P_t(2);
            P_z = P_t(3);

            % plot rotated frame
            plot_handle(5) = line((0.1*[0 R_t(1,1)]+P_x), ...
                                  (0.1*[0 R_t(2,1)]+P_y), ...
                                  (0.1*[0 R_t(3,1)]+P_z), ...
                                    'Color', 'r', 'LineWidth', 2);
            plot_handle(6) = line((0.1*[0 R_t(1,2)]+P_x), ...
                                  (0.1*[0 R_t(2,2)]+P_y), ...
                                  (0.1*[0 R_t(3,2)]+P_z), ...
                                   'Color', 'g', 'LineWidth', 2);
            plot_handle(7) = line((0.1*[0 R_t(1,3)]+P_x), ...
                                  (0.1*[0 R_t(2,3)]+P_y), ...
                                  (0.1*[0 R_t(3,3)]+P_z), ...
                                    'Color', 'b', 'LineWidth', 2);
        end

    end

    if viz
        title('FK\_body')
        xlabel('x-axis')
        ylabel('y-axis')
        zlabel('z-axis')
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

end