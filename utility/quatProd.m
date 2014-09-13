function [ r ] = quatProd( p, q)
%QUATPROD Performs oreder-preserving quaternion product
%   r = 1x4 scalar-first quaternion
if any(size(p) ~= [4 1])
    p = p';
end
if any(size(q) ~= [4 1])
    q = q';
end
if any(size(p) ~= [4 1]) || any(size(q) ~= [4 1])
    error('QUATPROD:Inputs must be 4x1 or 1x4');
end

r = [p(1)*q(1) - dot(p(2:4),q(2:4));...
    p(1)*q(2:4) + q(1)*p(2:4) - cross(p(2:4),q(2:4))];
r = r';



end

