function [S] = screw_axis_2_se3(s)
%screw_axis_2_skew_symmetric Convers screw axis vector to skew symmetric
%matrix
%   param:  1x6 screw axis vector 
%   return: 4x4 skew symmetrix matrix

%   reference: MR Ch4 Intro paragraph

    if length(s) ~= 6
        assert
    end

    omega = s(1:3);
    v = s(4:6);
    S = [   0         -omega(3)  omega(2)   v(1);
         omega(3)       0        -omega(1)  v(2);
         -omega(2)    omega(1)     0        v(3);
            0           0          0         0   ];
end