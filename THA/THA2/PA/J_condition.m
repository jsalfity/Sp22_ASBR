function [mu2] = J_condition(A)
%J_condition Calculates the condition number, a single scalar mesasure, 
% defining the sensitivity of the matrix A to scaling errors
%   param: A (3x3 matrix, usually calculated like:
%                         Js = J_space(robot, input_thetas);
%                         A = Js(1:3,:)*Js(1:3,:)';
%   return: mu2 (float) condition measure

%   reference: MR 5.4

    % calculate e-vecs and e-vals
    evals = eig(A);
    eval_min = min(evals);
    eval_max = max(evals);

    mu2 = eval_max / eval_min;
end