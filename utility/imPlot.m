function imPlot( t,x )
%IMPLOT plots the real and imaginary components of a complex vector x vs t 

plot(t,real(x),t,imag(x));
legend('real','imag');

end

