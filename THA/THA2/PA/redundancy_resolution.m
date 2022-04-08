function [q, idx, e] = redundancy_resolution(robot, Ti, Tf, q0, max_iterations)

q = q0;
T_sd = Tf;         % Desired configuration in space frame
T_bd = FK_body(robot, q, Ti, 0) \  T_sd; % Desired configuraiton in body frame

V_b_skew = logm(T_bd);
V_b = vector_from_skew(V_b_skew);
omega = V_b(1:3);
v = V_b(4:6);

idx = 0;

e = zeros(1,max_iterations+1);

J_gradient = 0;

while (norm(omega) > getGlobaleps ...
        || norm(v) > getGlobaleps)  ...
        && idx < max_iterations
    
    w_gradient = 1/2*det(J_body(q)*J_body(q)')^(-1/2)*det(J_body(q)*J_body(q)')*trace(inv(J_body(q)*J_body(q)')*J_gradient);
    
    q = q + pinv(J_body(robot, q))*V_b;
    
    T_bd = FK_body(robot, q, Ti, 0) \  T_sd;
    
    V_b_skew = logm(T_bd);
    V_b = vector_from_skew(V_b_skew);
    omega = V_b(1:3);
    v = V_b(4:6);
    
    idx = idx + 1;
    
    e(1,idx+1) = norm(v) + norm(omega);
end

end