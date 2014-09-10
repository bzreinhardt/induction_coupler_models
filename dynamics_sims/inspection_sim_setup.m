%setup variables for simulation of 3 fixed coupler inspection vehicle
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


