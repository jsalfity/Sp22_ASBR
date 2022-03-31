function [volume] = J_ellipsoid_volume(A)
%J_ellispoid_volume Calculates the condition number, a single scalar mesasure, 
% defining the sensitivity of the matrix A to scaling errors
%   param: A (3x3 matrix, usually calculated like:
%                         Js = J_space(robot, input_thetas);
%                         A = Js(1:3,:)*Js(1:3,:)';
%   return: volume (float) volume

%   reference: MR 5.4

    volume = sqrt(det(A));
end