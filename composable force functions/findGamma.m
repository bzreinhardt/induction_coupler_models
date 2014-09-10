function Gamma = findGamma(xi, v_x, v_y, mu0, sigma, w_e,b)
%finds the big gamma transmission propigation function from the bird style
%steady-state force solution to eddy current propigation
%INPUTS
%OUTPUT

s0 = 1i*(w_e*ones(size(xi)) - xi*v_x);
lambda = v_y * mu0*sigma/2*ones(size(xi));
gamma2 = xi.^2+mu0*sigma*s0.*ones(size(xi));
beta2 = lambda.^2+gamma2;
beta = sqrt(beta2);
test1 = coth(beta*b);
test2 = 2*beta.*xi.*test1;
test3 = 2*xi.^2 + mu0*sigma*s0 + test2;
test4 = mu0*sigma*(s0 - v_y*xi);
test5 = test4./test3;

Gamma = (mu0*sigma*(s0 - v_y*xi))./(2*xi.^2+mu0*sigma*s0+2*beta.*xi.*coth(beta*b));
Gamma(xi > 20) = 0;
Gamma(xi < -20) = 0;
end