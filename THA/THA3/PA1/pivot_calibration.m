function [b_post b_tip] = pivot_calibration(F)

num_frames = size(F,1)/3;

R_all = zeros(3*num_frames, 6);

for i = 1:num_frames
    R_all((i-1)*3+1:(i-1)*3+3,1:3) = F((i-1)*3+1:(i-1)*3+3,1:3);
    R_all((i-1)*3+1:(i-1)*3+3,4:6) = -eye(3);
end

P_all = -F(:,4);

output = pinv(R_all)*P_all;

b_tip = output(1:3,:);

b_post = output(4:6,:);

end

