%setup for simulation of a generic induction coupler inspection vehicle

%1) initialize magnet arrays in the body frame
%array properties - size etc
r_o = 2.54*0.75*0.01; %0.75 inches converted to m
r_i = 2.54*0.5*0.01;  %0.5 inches converted to m

Br = 1.42; %T Magnet strength
ur = 1.08; %unitless magnet relative permeability
P = 1; %pole-pairs
C = findC(Br,P,ur,r_o,r_i); %C property of a halbach array
%spin axes - each column is a unit vector pointing along axis
a1 = [1;0;0];
a2 = [1;0;0];
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
%inertia matrix
%4) initialize the body in the inertial frame
%translational pose
%rotation matrix from body to inertial i_A_b
%5) initialize the target in the inertial frame - origin of system is at
%closest point of a plane or curve to the target
%curvature
% properties
b = 0.01; %plate thickness
mu0 = 1.256E-6; %m*kg/s^2 A^2 magnetic permeability in a vacuum 
rho = 2.83E-6; %ohm-m resistivity of the plate
sigma = 1/rho; %conductivity of the plate



c = 5.031E-4; %N/(rad/sec)
a = [1 0 -1; 0 1 0; 0 0 0]; %each column is an a vector
n = [0 0 0;0 0 0; 1 1 1];  %plate normal vectors
l = 0.1; %lengths of the arms
d = l*a; %in the spacecraft frame
u_max = 4500; %max motor speed in rad/sec

m = 4; %kg

I = 2/5*m*l^2;

J = c*[cross(d,cross(a,n));cross(a,n)];

%define a rotation matrix between inspector reference frame and ISS plate
%frame- \frac{2\, {\left|y\right|}^2\, \left(\sqrt{\mathrm{C1}}\, \mathrm{sign}\!\left(y\right)\, K_{1}\left(\left(\sqrt{\mathrm{C1}}\, \left|y\right|\right)\right) + \frac{2\, \mathrm{sign}\!\left(y\right)\, K_{2}\left(\left(\sqrt{\mathrm{C1}}\, \left|y\right|\right)\right)}{\left|y\right|}\right)\, \mathrm{i}}{3\, \mathrm{C1}} + \frac{4\, \left|y\right|\, \mathrm{sign}\!\left(y\right)\, K_{2}\left(\left(\sqrt{\mathrm{C1}}\, \left|y\right|\right)\right)\, \mathrm{i}}{3\, \mathrm{C1}}
waypts = [[0;1;0],[pi/8;1.25;0.25],[pi/4;1.5;0.5],[3*pi/8;1.75;0.75],...
    [4*pi/10;2;1],[pi/2;2;2]];
R = 1; %radius of the circle
dt = 0.1;
t = 0;
targetPt = 1;
x = [0;0];
theta = 0;
X = zeros(6,1);%keeps track of the state
%[theta; x; y; thetadot; xdot; ydot]

 %state propigation matrix
 A = zeros(6,6);
 A(1,4) = dt;
 A(2,5) = dt;
 A(3,6) = dt;
 
 B = [0 0 0;...
     0 0 0;...
     0 0 0;...
     1/I*dt 0 0;...
     0 1/m*dt 0;...
     0 0 1/m*dt];
 inputs = [0;0;0];


