function g = findgap_xyplate(X,d)
    %find the gap between an end effector and a plate in the x-y palte
    %position of effector in inertial coords
    % X - state of the inspector 13x1 [x;v;q;omega] q = cos first quaternion
    % d - position of effector wrt com in inertial coords
    A = quat2dcm(X(7:10)');
    d_i = A'*d+X(1:3);
    g = abs(d_i(3));
    if d_i(3) < 0
        warning('findgap_xyplate: effector is below palte');
    end
end