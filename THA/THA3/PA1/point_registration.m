function T = point_registration(A,B)
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

R = [q0^2+q1^2-q2^2-q3^2 2*(q1*q2-q0*q3) 2*(q1*q3+q0*q2);
    2*(q1*q2+q0*q3) q0^2-q1^2+q2^2-q3^2 2*(q2*q3-q0*q1);
    2*(q1*q3-q0*q2) 2*(q1*q3+q0*q1) q0^2-q1^2-q2^2+q3^2];

p = b_avg - R*a_avg;

T = [R p;0 0 0 1];

end
