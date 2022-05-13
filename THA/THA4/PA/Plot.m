n_vec = [0 0 1];
wall_loc = [0;0;0.45];

figure
for i = 1:98
    plot3(tip_loc(1,i),tip_loc(2,i),tip_loc(3,i),'ro')
    hold on
end

w = null(n_vec); % Find two orthonormal vectors which are orthogonal to v
[P,Q] = meshgrid(-1.5:1.5); % Provide a gridwork (you choose the size)
X = wall_loc(1)+w(1,1)*P+w(1,2)*Q; % Compute the corresponding cartesian coordinates
Y = wall_loc(2)+w(2,1)*P+w(2,2)*Q; %   using the two vectors in w
Z = wall_loc(3)+w(3,1)*P+w(3,2)*Q;
surf(X,Y,Z)