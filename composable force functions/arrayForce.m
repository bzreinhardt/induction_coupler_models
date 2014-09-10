function [ F ] = arrayForce( a,n,w,B_s, Gamma )
%ARRAYFORCE finds the force from a spinning halbach array near a surface
%   a - unit vector for array spin axis 3x1
%   n - normal vector to surface at closest point to array 3x1 
%   B_s - fourier transformed halbach source field @B_s(xi) 
%   Gamma - transmission function @Gamma(xi)
%   w - width of the array
%TODO: 
%make it work with arrays


%note: model ignores damping forces from velocity of arrays
%note: assumes array is close enough to surface that it is effectively flat
f = fourierForce(B_s,Gamma,w); %force magnitudes in tangential and parallel
%directions to the plane
dir_x = cross(a,n);
dir_y = cross(dir_x,a);
F = f(1)*dir_x + f(2)*dir_y;

end

