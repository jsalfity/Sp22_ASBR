function output = RinSO3(R)
%RINSO3 Check R is in SO(3)
    % 3 checks
    % norm(R) = 1 (at least by some epsilon)
    % R * R' = I  (at least by some epsilon)
    % det(R) = 1  (at least by some epsilon) 

    output = (norm(R) - 1)          < getGlobaleps && ...
             norm(R*R' - eye(3))    < getGlobaleps && ...
             (det(R) - 1)           < getGlobaleps ;
      
end

