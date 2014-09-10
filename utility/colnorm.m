function [norm] = colnorm(S, dim)
%COLNORM Finds the norm of each column or row along the given dimension
%   Detailed explanation goes here

norm = sqrt(sum(S.^2,dim));


end

