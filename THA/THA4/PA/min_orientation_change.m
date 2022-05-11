function [q_delta] = min_orientation_change(p_tip, p_goal, q_init, ...
    q_ub, q_lb, tol, w_p, w_a)
% tool_tip_near_point Place robot and new tool tip near point p_gal
%   param p_tip: new tool tip
%   param p_goal: goal tool tip
%   param q_init: initial joint configuration
%   param q_ub: upper bound for joint limit
%   param q_lb: lower bound for joint limit
%   param tol: tolerance to keep the tip within
%   param w_p: weight for position matching
%   param w_a: weight for orientation-change minimization

%   return q_delta: joint command

%   reference: ASBR W16 L1-L2

% set q_current
q_curr = q_init;

% Find tooltip position vector
% T_robot = getTransform(robot, q_curr, 'panda_link8');   %matlab
run('make_panda.m')
T_robot = FK_space(panda, q_curr,  panda.M, 1);
R = T_robot(1:3, 1:3);
t = T_robot * p_tip;
t = t(1:3);

% space jacobian
% J = geometricJacobian(robot, q_curr, 'panda_link8');
J = J_space(panda, q_curr);

J_alpha = J(1:3, :);    % rotational
J_eps = J(4:6, :);      % translation

% Matrices for Optimization
% cost function
C1 = -vec_2_skew_mat(t)*J_alpha + J_eps; % closest to the goal point
C2 = -vec_2_skew_mat(R*[0;0;1])*J_alpha; % least orientation change
d = (t - p_goal); % negative to account for ||Cx - d||

% no constraints
%         q_delta = lsqr(C, d);

% with constraint
% Implement Virtual Fixture through:
% Telerobotic Control by Virtual Fixtures for Surgical Applications
% equation 22.11
A = [];
m = 10; %number of vertices for polygon
n = 10; % number of vertices for polygon
for i=1:m
    A = [A; [cos(i*2*pi/n)*cos(i*2*pi/m) cos(i*2*pi/n)*sin(i*2*pi/m) sin(i*2*pi/n)]];
end

% d = [d;0;0;0];

b = tol * ones(m, 1) - A*d;

%         C = [sqrt(zeta)*C; sqrt(eta)*-vec_2_skew_mat(R(:,3))*J(1:3, :)];
%         d = [sqrt(zeta)*d; sqrt(eta)*zeros(3, 1)];

q_delta = lsqlin(w_p*C1+w_a*C2, -w_p*d, A*C1, b, [], [], q_lb-q_curr, q_ub-q_curr);

% i'm not sure whats causing this error??
% if size(q_delta, 1) ~= 9
%     continue
% end

q_delta;

end