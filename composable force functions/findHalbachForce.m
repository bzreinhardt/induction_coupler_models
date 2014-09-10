function f = findHalbachForce(v_x, v_y, x, C, P, w_m,b,r_o,r_i,g,w)
%Finds the force in x and y direction on a spinning halbach rotor above a plate
% using a
%steady state analysis

%INPUTS

%g %air gap m between bottom of rotor and top of plate
%w width of rotor in m
%v_x %x velocity of the wheel
%v_y = 0; %y velocity of the wheel
% x %position of the rotor in the x direction
%w_m angular velocity of the wheel 
%b %plate thickness in m
%r_o outer radius of rotor
%r_i inner radius of rotor

%OUTPUTS
    % f - 2x1 vector [fx; fy]

%constants
    mu0 = 1.256E-6; %magnetic permiability in free space %change if not this universe
    rho = 2.83E-6; %ohm-m resistivity of the plate %change if not Al
    sigma = 1/rho; %conductivity of the plate %change if not Al
    
    w_e = w_m*P; %magnetic frequency that the plate sees, related to mechanical rotational frequency by the number of pole-pairs
    integrand = @(xi)forceInt(xi, v_x, v_y, x, C, mu0, sigma,P, w_e,b,r_o,r_i,g);
    im_force = w/(8*pi*mu0)*integral(integrand,-inf,inf);
    f = [imag(im_force);real(im_force)];
    
end