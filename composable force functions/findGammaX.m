function Gamma_x = findGammaX(xi, v_x, v_y, mu0, sigma, w_e,b)
%find the coefficients in the integral for F_x
gamma = sqrt(xi.^2+1i*mu0*sigma*(w_e+v_x*xi));
gammar = real(gamma);
gammai = imag(gamma);
chunk1 = xi.*gammai.*gammar./abs(exp(gamma*b).*(gamma+xi).^2-exp(-gamma*b).*(gamma-xi).^2).^2;
chunk2 = xi.*cos(2*gammai*b)-xi.*cosh(2*gammar*b)-gammar.*sinh(2*gammar*b)...
    -gammai.*sin(2*gammai*b);
Gamma_x = chunk1.*chunk2;
end
