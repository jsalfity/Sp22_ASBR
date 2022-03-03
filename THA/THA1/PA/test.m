clc; clear all;
set(0, 'DefaultFigureColor', 'w')
eps = getGlobaleps;

% 1a rotation_2_axis_angle
% To find an axis angle from a rotation matrix:
% The first singularity case is checked if R = I, because there is no
% rotation in this case.
% The second case checks trace(R) == -1, as it would be a rotation by pi.
% Finally, if those checks pass, the axis and angle are returned.
    %case 1
    R = eye(3);
    [theta, omega] = rotation_2_axis_angle(R);
    assert(isnan(omega))
    assert(theta == 0)
    
    % case 2
    R = [1 0 0; 0 -1 0; 0 0 -1];
    [theta, omega] = rotation_2_axis_angle(R);
    assert( abs(theta - pi) < eps)

    % case 3 (MR Example 3.12)
    R = [0.8658  -0.2502  0.4333; 
         0.2502   0.9665  0.0581;
         -0.4333  0.0581  0.8994];
    [theta, omega] = rotation_2_axis_angle(R);
    assert(norm(omega - [0;0.866;0.5]) < eps);
    assert(abs(theta - 0.524) < eps);

% 1b rotation_2_quaternion
% reference: rotm2quat() provided by matlab
    R = eye(3);
    q = rotation_2_quaternion(R);
    qmatlab = rotm2quat(R);
    assert(norm(q - qmatlab') < eps);
    
    R = [0.8658  -0.2502  0.4333; 
         0.2502   0.9665  0.0581;
         -0.4333  0.0581  0.8994];
    q = rotation_2_quaternion(R);
    qmatlab = rotm2quat(R);
    assert(norm(q - qmatlab') < eps);

    R = [0 0 1; 0 1 0; -1 0 0];
    q = rotation_2_quaternion(R);
    qmatlab = rotm2quat(R);
    assert(norm(q - qmatlab') < eps);

% 1c rotation_2_ZYZ
    R = eye(3);
    [phi, theta, psi] = rotation_2_ZYZ(R, -pi/2);
    angles_matlab = rotm2eul(R, 'ZYZ');
    assert(norm([angles_matlab(1) - phi; ...
                 angles_matlab(2) - theta; ...
                 angles_matlab(3) - psi]) < eps);
    
    R = [0.8658  -0.2502  0.4333; 
         0.2502   0.9665  0.0581;
         -0.4333  0.0581  0.8994];
    [phi, theta, psi] = rotation_2_ZYZ(R, -pi/2);
    angles_matlab = rotm2eul(R, 'ZYZ');
    assert(norm([angles_matlab(1) - phi; ...
                 angles_matlab(2) - theta; ...
                 angles_matlab(3) - psi]) < eps);

    R = [0 0 1; 0 1 0; -1 0 0];
    [phi, theta, psi] = rotation_2_ZYZ(R, -pi/2);
    angles_matlab = rotm2eul(R, 'ZYZ');
    assert(norm([angles_matlab(1) - phi; ...
                 angles_matlab(2) - theta; ...
                 angles_matlab(3) - psi]) < eps);

% 1d rotation_2_rpy
% reference: rotm2eul() provided by matlab
    R = eye(3);
    [phi, theta, psi] = rotation_2_rpy(R, 0);
    angles_matlab = rotm2eul(R);
    assert(norm([angles_matlab(1) - phi; ...
                 angles_matlab(2) - theta; ...
                 angles_matlab(3) - psi]) < eps);
    
    R = [0.8658  -0.2502  0.4333; 
         0.2502   0.9665  0.0581;
         -0.4333  0.0581  0.8994];
    [phi, theta, psi] = rotation_2_rpy(R, 0);
    angles_matlab = rotm2eul(R);
    assert(norm([angles_matlab(1) - phi; ...
                 angles_matlab(2) - theta; ...
                 angles_matlab(3) - psi]) < eps);

    R = [0 0 1; 0 1 0; -1 0 0];
    [phi, theta, psi] = rotation_2_rpy(R, 0);
    angles_matlab = rotm2eul(R);
    assert(norm([angles_matlab(1) - phi; ...
                 angles_matlab(2) - theta; ...
                 angles_matlab(3) - psi]) < eps);
 
% 2a axis_angle_2_rotation (MR Example 3.12)
    omega = [0 0.866 0.5];
    theta = 0.524;
    R = axis_angle_2_rotation(omega, theta);
    R_book = [0.8658  -0.2502  0.4333; 
              0.2502   0.9665  0.0581;
              -0.4333  0.0581  0.8994];
    assert(norm(R - R_book) < eps);

% 2b quaternion_2_rotation
    q = [0.7071 0.7071 0 0];
    R = quaternion_2_rotation(q);
    Rmatlab = quat2rotm(q);
    assert(norm(R - Rmatlab) < eps);
    
    q = [1 0 0 0];
    R = quaternion_2_rotation(q);
    Rmatlab = quat2rotm(q);
    assert(norm(R - Rmatlab) < eps);

    q = [0.9659 0 0.2243 0.1295];
    R = quaternion_2_rotation(q);
    Rmatlab = quat2rotm(q);
    assert(norm(R - Rmatlab) < eps);

%% 3 screw axis 
clear all; close all; clc;
set(0, 'DefaultFigureColor', 'w')
q = [0,2,0]';
shat = [0, 0, 1]';
h = 2;
theta = pi;
T = [1 0 0 2;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];

thetas = [0, theta/4,  theta/2, 3*theta/4, theta];
[Tfinal] = screw(T, q, shat, h, thetas);
