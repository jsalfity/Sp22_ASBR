% Panda Robot struct containing relevant kinematic info
panda.n_joints = 7;
panda.M = [1 0 0 0.088; ...
           0 -1 0 0; ...
           0 0 -1 0.926; ...
           0 0 0 1];

% q_panda_matlab_home = [0 0 0 -1.5708 0 0 0 0 0 ];
% T = getTransform(panda_matlab, q_panda_matlab_home, 'panda_link7');
% M = inv(T)
% panda.M =   [0   0   -1    0.6435;
%              0   -1   0    0;
%             -1   0    0    0.4665;
%              0   0    0    1.0000];


panda.space.screw_axes(:,:,1) = [0; 0; 1;     0;      0;      0];
panda.space.screw_axes(:,:,2) = [0; 1; 0;     -0.333; 0;      0];
panda.space.screw_axes(:,:,3) = [0; 0; 1;     0;      0;      0];
panda.space.screw_axes(:,:,4) = [0; -1; 0;    0.649; 0;      -0.088];
panda.space.screw_axes(:,:,5) = [0; 0; 1;     0;      0;      0 ];
panda.space.screw_axes(:,:,6) = [0; -1; 0;    1.033;  0;      0];
panda.space.screw_axes(:,:,7) = [0; 0; -1;    0;    0.088;   0];

% M for each joint (treat each end joint as an end effector)
panda.space.t = zeros(28,4);
panda.space.t(1:4,:) = [1 0 0 0;0 1 0 0;0 0 1 0.333;0 0 0 1];
panda.space.t(5:8,:) = [1 0 0 0;0 0 1 0;0 -1 0 0.333;0 0 0 1];
panda.space.t(9:12,:) = [1 0 0 0;0 1 0 0;0 0 1 0.649;0 0 0 1];
panda.space.t(13:16,:) = [1 0 0 0.088;0 0 -1 0;0 1 0 0.649;0 0 0 1];
panda.space.t(17:20,:) = [1 0 0 0;0 1 0 0;0 0 1 1.033;0 0 0 1];
panda.space.t(21:24,:) = [1 0 0 0;0 0 -1 0;0 1 0 1.033;0 0 0 1];
panda.space.t(25:28,:) = panda.M;

% panda.space.screw_axes_q(:,:,1) = [0; 0; 0.333];
% panda.space.screw_axes_q(:,:,2) = [0; 0; 0.333];
% panda.space.screw_axes_q(:,:,3) = [0; 0; 0.649];
% panda.space.screw_axes_q(:,:,4) = [0.088; 0; 0.649];
% panda.space.screw_axes_q(:,:,5) = [0; 0; 1.033];
% panda.space.screw_axes_q(:,:,6) = [0; 0; 1.033];
% panda.space.screw_axes_q(:,:,7) = [0.088; 0; 1.033];


panda.body.screw_axes(:,:,1) = [0; 0; -1;     0;      -0.088;      0];
panda.body.screw_axes(:,:,2) = [0; -1; 0;     0.593;    0;         0.088];
panda.body.screw_axes(:,:,3) = [0; 0; -1;     0;      -0.088;      0];
panda.body.screw_axes(:,:,4) = [0; 1; 0;    -0.277;     0;         0];
panda.body.screw_axes(:,:,5) = [0; 0; -1;     0;      -0.088;      0 ];
panda.body.screw_axes(:,:,6) = [0; 1; 0;    0.107;       0;      -0.088];
panda.body.screw_axes(:,:,7) = [0; 0;  1;    0;         0;   0];

% M for each joint (treat each end joint as an end effector)
panda.body.t = zeros(28,4);
panda.body.t(1:4,:) = [1 0 0 0;0 1 0 0;0 0 1 0.333;0 0 0 1];
panda.body.t(5:8,:) = [1 0 0 0;0 0 1 0;0 -1 0 0.333;0 0 0 1];
panda.body.t(9:12,:) = [1 0 0 0;0 1 0 0;0 0 1 0.649;0 0 0 1];
panda.body.t(13:16,:) = [1 0 0 0.088;0 0 -1 0;0 1 0 0.649;0 0 0 1];
panda.body.t(17:20,:) = [1 0 0 0;0 1 0 0;0 0 1 1.033;0 0 0 1];
panda.body.t(21:24,:) = [1 0 0 0;0 0 -1 0;0 1 0 1.033;0 0 0 1];
panda.body.t(25:28,:) = panda.M;
% panda.body.screw_axes_q(:,:,1) = [-0.088; 0; 0.593];
% panda.body.screw_axes_q(:,:,2) = [-0.088; 0; 0.593];
% panda.body.screw_axes_q(:,:,3) = [-0.088; 0; 0.277];
% panda.body.screw_axes_q(:,:,4) = [0.088; 0; 0.649];
% panda.body.screw_axes_q(:,:,5) = [-0.088; 0; 0.277];
% panda.body.screw_axes_q(:,:,6) = [-0.088; 0; 0.277];
% panda.body.screw_axes_q(:,:,7) = [nan];