function q_star = quatConj( q )
%QUATCONJ finds the conjugate of a quaternion
%  INPUT - q - real first quaternion 4x1
%  OUTPUT - q* - real first quaternion 4x1 st q*q = |q|^2

q_star = [q(1) -1*q(2:4)];


end

