function [b_post, b_tip] = pivot_cal(F)

N = size(F,1)/4;

R = zeros(3*N,6);
P = zeros(3*N,1);

for i = 1:N
    R(3*(i-1)+1:3*(i-1)+3,1:3) = F(1:3,1:3);
    R(3*(i-1)+1:3*(i-1)+3,4:6) = -eye(3);
    P(3*(i-1)+1:3*(i-1)+3,:) = -F(1:3,4);
end

output = pinv(R)*P;

b_post = output(1);
b_tip = output(2);

