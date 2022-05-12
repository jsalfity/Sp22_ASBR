function [Jb] = J_body(robot, q)
%J_space Summary of this function goes here
%   param: robot (struct with n_joints, M, screw_axes, qs)
%   param: q (nx1 input thetas)
%   return: Js in body form

%   reference: MR 5.1.2

    Jb(:, robot.n_joints) = robot.body.screw_axes(:,:,robot.n_joints);
    T = eye(4);
    % go in reverse
    for idx = robot.n_joints - 1: -1: 1
        b = robot.body.screw_axes(:,:,idx+1);
        B = screw_axis_2_se3(b);
        T = T * expm(-1 * B * q(idx+1));
        Jb(:, idx) = Adjoint(T) * robot.body.screw_axes(:,:,idx);
    end
end