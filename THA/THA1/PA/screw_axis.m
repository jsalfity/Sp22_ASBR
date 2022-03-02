% transform from T to the origin 
% input: configuration T
% output: screw axis s1, and theta
% output plot with fixed frame and screw axis and the point q on the screw
% axis
function [s1,theta] = screw_axis(T)

T0 = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1]; % origin
T_01 = T0 * inv(T);

R = T_01(1:3,1:3);

if R == eye(3)
    omega = [0;0;0];
    v_norm = sqrt(T_01(1,4)^2+T_01(2,4)^2+T_01(3,4)^2);
    v = T_01(1:3,4)/v_norm;
    theta = v_norm;
    
    theta_dot = v_norm;

    h = omega'*v/theta_dot;

    q = cross(omega,v)/theta_dot;
else
    [theta, omega] = rotation_2_axis_angle(R);

    w = [0 -omega(3) omega(2);omega(3) 0 -omega(1);-omega(2) omega(1) 0];

    G_inv = 1/theta*eye(3) - 1/2*w + (1/theta-1/2*cot(theta/2))*w^2;

    v = G_inv * T_01(1:3,4);

    omega_norm = (omega(1)^2 + omega(2)^2 + omega(3)^2)^(1/2);

    omega = omega/omega_norm;
    v = v/omega_norm;
    
    theta_dot = omega_norm;

    h = omega'*v/theta_dot;

    q = cross(omega,v)/theta_dot;
end

s1 = [omega;v];
theta = theta;

disp('screw axis: ')
disp(s1)
disp('theta: ')
disp(theta)

x1 = q(1);
y1 = q(2);
z1 = q(3);

if omega == [0;0;0]
    omega = v;
end

x2 = q(1) + omega(1);
y2 = q(2) + omega(2);
z2 = q(3) + omega(3);

nx = x2 - x1;
ny = y2 - y1;
nz = z2 - z1;
len = 1000;
xx = [x1 - len*nx, x2 + len*nx];
yy = [y1 - len*ny, y2 + len*ny];
zz = [z1 - len*nz, z2 + len*nz];
figure
h(1) = plot3(xx, yy, zz,'--','Color','b');
hold on

% origin
x1 = [0;0;0];
y1 = [0;0;0];
z1 = [0;0;0];

x2 = [1;0;0];
y2 = [0;1;0];
z2 = [0;0;1];

nx = x2 - x1;
ny = y2 - y1;
nz = z2 - z1;
len = 1;
xx = [x1, x2 + len*nx];
yy = [y1, y2 + len*ny];
zz = [z1, z2 + len*nz];
h(2) = plot3(xx(1,:), yy(1,:), zz(1,:),'Color','k');
hold on
h(3) = plot3(xx(2,:), yy(2,:), zz(2,:),'Color','k');
hold on
h(4) = plot3(xx(3,:), yy(3,:), zz(3,:),'Color','k');
hold on
h(5) = plot3(q(1),q(2),q(3),'o','Color','r');

xlabel('x')
ylabel('y')
zlabel('z')
grid on
xlim([-5, 5])
ylim([-5, 5])
zlim([-5, 5])
legend(h([1,2,5]),{'Screw Axis','x-y-z Coordinate','q'},'Location','northeast')

end



