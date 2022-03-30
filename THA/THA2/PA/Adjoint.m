function [Adjoint_T] = Adjoint(T)
%Adjoint: Calculates the Adjoint matrix
%   param:  T matrix (4x4)
%   return: Adjoint matrix (6x6)

    R = T(1:3,1:3);
    p = [T(1,4), T(2,4), T(3,4)];
    p_hat = [  0      -p(3)   p(2);...
              p(3)     0      -p(1); ...
             -p(2)     p(1)     0];
    Adjoint_T = [R, zeros(3,3);
                 p_hat*R, R];
end