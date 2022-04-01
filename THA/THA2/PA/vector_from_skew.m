function [V] = vector_from_skew(M)
%vector_from_skew Summary of this function goes here
%   param: M (4x4 matrix)
%   return: V (6x1) 
    V =  [M(3, 2); 
          M(1, 3); 
          M(2, 1); 
          M(1, 4);
          M(2, 4);
          M(3, 4)];
end