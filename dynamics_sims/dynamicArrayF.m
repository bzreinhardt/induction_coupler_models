%control force and torque for eddy current spacecraft
function f = dynamicArrayF(omega,X, a, d, B_s, Gamma,w)
%Find the force from a single magnetic actuator
%omega - input speed to motor
%X - pose of the inspector
%a - magnet axis in body frame
%d - magnet position in body frame
%B_s - source field @B_s(xi,g)
%Gamma - transmission function @Gamma(xi, v_x, v_z, omega)
%w - width of array
%TODO make this more complicated
%find the position and orientation of each magnet
A = quat2dcm(X(7:11)')'; %rotation from body to inertial coords
a_i = A*a;
d_i = A*d;

%find the position of the plate closest to each magnet
%TODO update for curved plate for now assume plate lies in x-y plane
g = d_i(3)+X(3); %gap to plate
n = [0;0;1]; %plate normal 
%find the force for each magnet
B = @(xi)B_s(xi,g); %fourier transformed source
xy_vel = sqrt(X(4)^2+X(5)^2);
G = @(xi)Gamma(xi, xy_vel,X(6),omega);
f = arrayForce(a_i, n, w, B, G);
end