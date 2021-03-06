function T = point_registration(A,B)
% input:    A  point cloud
%           B  point cloud
% output:   T transformation matrix between two point clouds
% Reference: lecture W10-L2

% Sizes of A and B are 3*N

N = size(A,2); % number of points in the point cloud

a_avg = 1/N*sum(A,2); % mean point of all the point cloud
b_avg = 1/N*sum(B,2);

A_dev = A-a_avg; % deviation from the mean
B_dev = B-b_avg;

% Using the eigenvalue decomposition method to calculate R
H = A_dev*B_dev';

delta = [H(2,3)-H(3,2) H(3,1)-H(1,3) H(1,2)-H(2,1)]';
G = [trace(H) delta';delta H+H'-trace(H)*eye(3)];

[V,D] = eig(G);

q0 = V(1,end);
q1 = V(2,end);
q2 = V(3,end);
q3 = V(4,end);

q(1) = q0;
q(2) = q1;
q(3) = q2;
q(4) = q3;

R = quaternion_2_rotation(q);

p = b_avg - R*a_avg;

T = [R p;0 0 0 1];

end
