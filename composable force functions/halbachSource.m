function B_s = halbachSource(xi, C, P, r_o, g, x)

B_s = 4*pi*C*xi.^P/factorial(P).*exp(-xi.*(r_o + g +1i*x)).*(xi >= 0);
B_s(isnan(B_s))= 0;
end