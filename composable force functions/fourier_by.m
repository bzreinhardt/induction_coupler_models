function By = fourier_by(C,C4,C5,C6,xi)
%FOURIER_BY
%    BY = FOURIER_BY(C,C4,C5,C6,W)

%    This function was generated by the Symbolic Math Toolbox version 5.11.
%    14-Aug-2014 11:01:17

% t2 = abs(w);
% t3 = sqrt(C);
% t4 = t2.*t3;
% t5 = 1.0./sqrt(C);
% t6 = sign(w);
% t7 = 1.0./t6;
% By = C6.*t2.*t5.*besselkk(1,t4).*2.0+(C5.*t2.^2.*besselkk(2,t4).*(2.0./3.0))./C-C4.*t5.*t7.*w.^2.*besselkk(1,t3.*t7.*w).*6.666666666666667e-1i;
F3 = -2*xi.^2.*besselk(1,C*abs(xi))*1i./(3*C*sign(xi));
F2 = sqrt(2/pi)*1/C*abs(xi).*besselk(1,C*abs(xi));
F1 = sqrt(2/pi)*1/C*xi.^2.*besselk(2,C*abs(xi))/(3*C);
By = C4*F3+C5*F1+C6*F2;