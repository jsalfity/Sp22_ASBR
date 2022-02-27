clc; clear all;

eps = 1e-3;

% 1a rotation_2_axis_angle
    %case 1
    R = eye(3);
    [theta, omega] = rotation_2_axis_angle(R);
    assert(isnan(omega))
    assert(theta == 0)
    
    % case 2
    R = [1 0 1; -1 -2 1; 0 0 0];
    [theta, omega] = rotation_2_axis_angle(R);
    assert(theta == pi)

    % case 3 (MR Example 3.12)
    R = [0.8658  -0.2502  0.4333; 
         0.2502   0.9665  0.0581;
         -0.4333  0.0581  0.8994];
    [theta, omega] = rotation_2_axis_angle(R);
    assert(norm(omega - [0;0.866;0.5]) < eps);
    assert(abs(theta - 0.524) < eps);

% 1b rotation_2_quaternion
    
% 1c rotation_2_ZYZ

% 1c rotation_2_rpy

% 2a axis_angle_2_rotation (MR Example 3.12)
    omega = [0 0.866 0.5];
    theta = 0.524;
    R = axis_angle_2_rotation(omega, theta);
    R_book = [0.8658  -0.2502  0.4333; 
              0.2502   0.9665  0.0581;
              -0.4333  0.0581  0.8994];
    assert(norm(R - R_book) < eps);

% 2b quaternion_2_rotation

%% 3 screw axis 
clear all; close all; clc;
q = [0,2,0]';
shat = [0, 0, 1]';
h = 2;
theta = pi;
T = [1 0 0 2;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];

[Tfinal] = screw(T, q, shat, h, theta);
