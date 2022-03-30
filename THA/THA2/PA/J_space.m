function [Js] = J_space(robot, input_thetas)
%J_space Summary of this function goes here
%   param: robot (struct with n_joints, M, screw_axes, qs)
%   return: Js in space form

%   reference: MR 5.1.1

    Js(:, 1) = robot.space.screw_axes(:,:,1);
    T = eye(4);
    for idx = 2: robot.n_joints
        s = robot.space.screw_axes(:,:,idx-1);
        S = screw_axis_2_se3(s);
        T = T * expm(S * input_thetas(idx-1));
        Js(:, idx) = Adjoint(T) * robot.space.screw_axes(:,:,idx);
    end
end