function [S] = vec_2_skew_matrix(v)
%vec_2_skew_matrix Convers screw axis vector to skew symmetric
%matrix
%   param: (v) 1x3 screw axis vector 
%   return: (S) 3x3 skew symmetrix matrix

%   reference: MR Ch4 Intro paragraph

    if length(v) ~= 3
        assert
    end

    S = [   0       -v(3)    v(2) ;
           v(3)       0     -v(1) ;
          -v(2)      v(1)     0   ];
end
