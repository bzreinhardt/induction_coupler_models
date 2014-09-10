function B_s = emSource(w, m, phi, y, y_m, mu0)
%evaluate the fourier transform in the x direction of a 2d em
%w - spatial frequency
%m - dipole strength
%phi - angle between dipole and horizontal
%y - height of the point of interest (usually surface fo sheet)
%y_m - height of the magnet 
%mu_0 - permiability of magnetic field in free space
% see reinhardt lab notebook page 3 for diagram

C = (y - y_m);
C1 = mu0/(4*pi)*3*m*cos(phi);
C2 = mu0/(4*pi)*3*sin(phi)*(y-y_m);
C3 = mu0/(4*pi)*-m*cos(phi);
C4 = mu0/(4*pi)*3*m*cos(phi)*(y-y_m);
C5 = mu0/(4*pi)*3*m*sin(phi)*C^2;
C6 = mu0/(4*pi)*m*sin(phi);
B_s = fourier_bx(C,C1,C2,C3,w) + 1i*fourier_by(C,C4,C5,C6,w);
if sum(isnan(B_s))>0
    warning('problem!');
end
%B_s(isnan(B_s))= 0;
%B_s = ones(size(w,1),size(w,2))./(w.^2 + 1);

end