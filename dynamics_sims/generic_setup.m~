%setup for simulation of a generic induction coupler inspection vehicle

%1) initialize magnet arrays in the body frame
%array properties - size etc
r_o = 2.54*0.75*0.01; %0.75 inches converted to m
r_i = 2.54*0.5*0.01;  %0.5 inches converted to m

Br = 1.42; %T Magnet strength
ur = 1.08; %unitless magnet relative permeability
P = 1; %pole-pairs
w = 0.01; % width of the magnet
C = findC(Br,P,ur,r_o,r_i); %C property of a halbach array
omega_max = 4500; %max motor speed in rad/sec
%spin axes - each column is a unit vector pointing along axis
a1 = [0;1;0];
a2 = [0;-1;0];
a = [a1 a2];

%array locations - each column is a vector from the com to the array
d1 = [0.25;0.25;0];
d2 = [0.25;-0.25;0];
d = [d1 d2];

%2) initialize electromagnets in the body frame
%EM properties - frequency, etc.
%pointing axes - each column is a unit vector pointing along dipole
%EM locations - each column is a vector from the com to the em

%3) initialize the mass properties 
%mass
kgperlb = 2.20462262;
m = 26/kgperlb;
%inertia matrix
%assuming a spherical robot for now
r = 0.1; %radius of the sphereical robot
I = 2/5*m*r^2*eye(3);

%4) initialize the body in the inertial frame
%translational pose
g = 0.01; %gap between magnet arrays and target
x0 = [0;0;g+max(d(end,:))];
v0 = [0;0;0];
om0 = [0;0;0];
%rotation matrix
q0 = [1 0 0 0];
%5) initialize the target in the inertial frame - origin of system is at
%closest point of a plane or curve to the target
%curvature
% properties
b = 0.01; %plate thickness
mu0 = 1.256E-6; %m*kg/s^2 A^2 magnetic permeability in a vacuum 
rho = 2.83E-6; %ohm-m resistivity of the plate
sigma = 1/rho; %conductivity of the plate
n_hat = [0;0;1]; %surface normal at closest initial point
kappa = 0; %curvature of the surface

%6)initialize time or controllers
t0 = 0;
t_max = 10; 
dt = 0.01;





