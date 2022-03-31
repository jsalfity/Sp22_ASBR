function [isotropy, condition, volume] = ellipsoid_plot_linear(robot, input_thetas)
%ellipsoid_plot_linear
%   param: robot (struct with n_joints, M, screw_axes, qs)
%   param: thetas (1xn) joints array
%   return: isotropy, condition, volume (floats)

%   reference: MR 5.4

    % calculate J_body
    Jb = J_body(robot, input_thetas);
    A = Jb(4:6,:)*Jb(4:6,:)';
    [V, D] = eig(A);

    % calculate isotropy, condition, volume
    isotropy = J_isotropy(A);
    condition = J_condition(A);
    volume = J_ellipsoid_volume(A);

    % centered at zero with axis length equal to sqrt(eigenvalues)
    [X, Y, Z] = ellipsoid(0,0,0, sqrt(D(1,1)), sqrt(D(2,2)), sqrt(D(3,3)));
    % rotation comes from eigenvectors
    [phi, theta, psi] = rotation_2_rpy(V, 0);

    figure(99)
    hsurf = surf(X, Y, Z);
    axis equal

    % rotate according to eigenvectors
    direction = [1 0 0]; %rotate the surface plot psi degrees around its x-axis
    rotate(hsurf, direction, psi)
    direction = [0 1 0]; %rotate the surface plot theta degrees around its y-axis
    rotate(hsurf, direction, theta)
    direction = [0 0 1]; %rotate the surface plot phi degrees around its z-axis
    rotate(hsurf, direction, phi)
    xlabel('v_1');
    ylabel('v_2');
    zlabel('v_3');

end