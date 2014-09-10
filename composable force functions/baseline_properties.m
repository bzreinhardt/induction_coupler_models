%magnet properties assuming my setup is a 1 pole pair halbach array
r_o = 2.54*0.75*0.01; %0.75 inches converted to m
r_i = 2.54*0.5*0.01;  %0.5 inches converted to m

Br = 1.42; %T Magnet strength
ur = 1.08; %unitless magnet relative permeability
P = 1; %pole-pairs
b = 0.01;

g = 0.01; %air gap m
mu0 = 1.256E-6; %m*kg/s^2 A^2 magnetic permeability in a vacuum 
rho = 2.83E-6; %ohm-m resistivity of the plate
sigma = 1/rho; %conductivity of the plate

C = findC(Br,P,ur,r_o,r_i);

m = 2E-3; %Am^2
phi = pi/2; %EM pointed straight at surface
w_e = 30;
w = 0.1;

v_x = 0;
v_y = 0;
x = 0;