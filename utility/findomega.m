function [ w ] = findomega( t, r12 )
%FINDOMEGA Finds the angular velocity of a rigid body from a moving vector
%attached to that body
%   Detailed explanation goes here
%   INPUTS
%       t - Nx1 vector of timestamps
%       r12 - Nx3 matrix of row-vectors that point from point 1 to point 2
%   OUTPUTS
%       w - Nx3 matrix of angular velocities
% #rigidbody #rbmotion

[n,m] = size(r12);
if m ~= 3
    if n==3
        r12 = r12';
        warning('transposed r12 to be Nx3');
    else
        error('r12 improper size');
    end
end
theta = acos(dot(r12(2:end,:),r12(1:end-1,:),2)./...
                (colnorm(r12(2:end,:),2).*colnorm(r12(1:end-1,:),2)));
            
dt = t(2:end)-t(1:end-1);
mag = theta./dt; %magnitude of omega at each time step

dir = normr(cross(r12(1:end-1,:),r12(2:end,:),2));

w = (mag*ones(1,3)).*dir;


end

