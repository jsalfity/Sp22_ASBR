function [isotropy, condition, volume] = ellipsoid_plot_angular(robot, q, ...
                                                                verbose, ...
                                                                figure_num)
%ellipsoid_plot_angular
%   param: robot (struct with n_joints, M, screw_axes, qs)
%   param: q (1xn) joints array
%   param: verbose (bool) to report the ellipsoid metrics
%   return: isotropy, condition, volume (floats)

%   reference: MR 5.4

    % calculate J_space
    Js = J_space(robot, q);
    A = Js(1:3,:)*Js(1:3,:)';
    [V, D] = eig(A);

    % calculate isotropy, condition, volume
    isotropy = J_isotropy(A);
    condition = J_condition(A);
    volume = J_ellipsoid_volume(A);

    % centered at zero with axis length equal to sqrt(eigenvalues)
    [X, Y, Z] = ellipsoid(0,0,0, sqrt(D(1,1)), sqrt(D(2,2)), sqrt(D(3,3)));
    % rotation comes from eigenvectors
    [phi, theta, psi] = rotation_2_rpy(V, 0);

    figure(figure_num)
    hsurf = surf(X, Y, Z);
    axis equal

    % rotate according to eigenvectors
    direction = [1 0 0]; %rotate the surface plot psi degrees around its x-axis
    rotate(hsurf, direction, psi)
    direction = [0 1 0]; %rotate the surface plot theta degrees around its y-axis
    rotate(hsurf, direction, theta)
    direction = [0 0 1]; %rotate the surface plot phi degrees around its z-axis
    rotate(hsurf, direction, phi)
    xlabel('\omega_1');
    ylabel('\omega_2');
    zlabel('\omega_3');
    
    txt_isotropy = ['isotropy: ' num2str(isotropy)];
    text(3, 1, txt_isotropy)
    txt_condition = ['condition: ' num2str(condition)];
    text(3, 2, txt_condition)
    txt_volume = ['volume: ' num2str(volume)];
    text(3, 3, txt_volume)

    title("Angular Manipulability Ellpsoid")

    if verbose
        volume
        isotropy
        condition
    end

end