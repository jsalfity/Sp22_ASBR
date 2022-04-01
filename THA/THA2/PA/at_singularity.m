function [output] = at_singularity(robot, input_thetas)
%at_singularity Determine if robot at given configuration is at singularity
%   param: robot (struct with n_joints, M, screw_axes, qs)
%   param: thetas (1xn) joints array
%   return: output (bool)

    Js = J_space(robot, input_thetas);
    output = (rank(Js) < 6);

end