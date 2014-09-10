function Gamma = findGamma2(xi, v_x, v_y, mu0, sigma, w_e,b)
%finds the big gamma transmission propigation function from the bird style
%steady-state force solution to eddy current propigation
%INPUTS
%OUTPUT

s0 = 1i*(w_e*ones(size(xi)) - xi*v_x);
lambda = v_y * mu0*sigma/2*ones(size(xi));
gamma2 = xi.^2+mu0*sigma*s0.*ones(size(xi));
beta2 = lambda.^2+gamma2;
beta = sqrt(beta2);

Tss_top = ((lambda-(xi+beta)).*exp(beta*b)-(lambda - (xi - beta)).*exp(-beta*b));
Tss_bottom = exp(beta*b).*(lambda.^2-(xi + beta).^2)-exp(-beta*b).*(lambda.^2-(xi-beta).^2);
Tss = Tss_top./Tss_bottom;

Gamma = 2.*xi.*Tss - 1;

end

