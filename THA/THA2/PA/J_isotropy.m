function [mu1] = J_isotropy(A)
%J_isotropy Calculates the isotropy, a single scalar measure, defining how
%easily the robot can move at a given posture
%   param: A (3x3 matrix, usually calculated like:
%                         Js = J_space(robot, input_thetas);
%                         A = Js(1:3,:)*Js(1:3,:)';
%   return: mu1 (float) isotropy measure

%   reference: MR 5.4

    % calculate e-vecs and e-vals
    evals = eig(A);
    eval_min = min(evals);
    eval_max = max(evals);

    mu1 = sqrt(eval_max / eval_min);
end