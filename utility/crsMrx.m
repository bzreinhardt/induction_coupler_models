function [a_cross] = crsMrx(a)
%CRSMRX find the cross matrix for a series of vectors
% #math #linearalg #mrxmath
    [n,m] = size(a);
    if m ~= 3
        if n == 3
            a = a';
            [n,m] = size(a);
        else 
            error('Input needs to be a Nx3 mrx')
        end
    end
    a_cross = zeros(n*3,3);
    for i = 1:n
        a_cross(3*i-2:3*i,:) = [0 a(i,3) -a(i,2); ...
                                -a(i,3) 0 a(i,1);...
                                a(i,2) -a(i,1) 0];
    end        
end