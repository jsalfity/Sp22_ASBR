% Panda Robot struct containing relevant kinematic info
panda.n_joints = 7;
panda.M = [1 0 0 0.088; ...
           0 -1 0 0; ...
           0 0 -1 0.926; ...
           0 0 0 1];

panda.space.screw_axes(:,:,1) = [0; 0; 1;     0;      0;      0];
panda.space.screw_axes(:,:,2) = [0; 1; 0;     -0.333; 0;      0];
panda.space.screw_axes(:,:,3) = [0; 0; 1;     0;      0;      0];
panda.space.screw_axes(:,:,4) = [0; -1; 0;    0.649; 0;      -0.088];
panda.space.screw_axes(:,:,5) = [0; 0; 1;     0;      0;      0 ];
panda.space.screw_axes(:,:,6) = [0; -1; 0;    1.033;  0;      0];
panda.space.screw_axes(:,:,7) = [0; 0; -1;    0;    0.088;   0];

% panda.space.qs(:,:,1) = [0; 0; 0.333];
% panda.space.qs(:,:,2) = [0; 0; 0.333];
% panda.space.qs(:,:,3) = [0; 0; 0.649];
% panda.space.qs(:,:,4) = [0.088; 0; 0.649];
% panda.space.qs(:,:,5) = [0; 0; 1.033];
% panda.space.qs(:,:,6) = [0; 0; 1.033];
% panda.space.qs(:,:,7) = [0.088; 0; 1.033];

panda.body.screw_axes(:,:,1) = [0; 0; -1;     0;      -0.088;      0];
panda.body.screw_axes(:,:,2) = [0; -1; 0;     0.593;    0;         0.88];
panda.body.screw_axes(:,:,3) = [0; 0; -1;     0;      -0.088;      0];
panda.body.screw_axes(:,:,4) = [0; 1; 0;    -0.277;     0;      -0.088];
panda.body.screw_axes(:,:,5) = [0; 0; -1;     0;      -0.088;      0 ];
panda.body.screw_axes(:,:,6) = [0; 1; 0;    0.107;       0;      -0.088];
panda.body.screw_axes(:,:,7) = [0; 0;  1;    0;         0;   0];

% panda.body.qs(:,:,1) = [-0.088; 0; 0.593];
% panda.body.qs(:,:,2) = [-0.088; 0; 0.593];
% panda.body.qs(:,:,3) = [-0.088; 0; 0.277];
% panda.body.qs(:,:,4) = [0.088; 0; 0.649];
% panda.body.qs(:,:,5) = [-0.088; 0; 0.277];
% panda.body.qs(:,:,6) = [-0.088; 0; 0.277];
% panda.body.qs(:,:,7) = [nan];