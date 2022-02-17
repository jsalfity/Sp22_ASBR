clc; clear all;

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

    % case 3

% 1b rotation_2_quaternion
    
% 1c ZYZ

% 1c roll-pitch-yaw


% 2a axis_angle_2_rotation
    omega = [0 0.866 0.5];
    theta = 0.524;
    R = axis_angle_2_rotation(omega, theta);

% 2b quaternion_2_rotation
