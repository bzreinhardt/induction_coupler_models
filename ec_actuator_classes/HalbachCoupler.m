classdef HalbachCoupler < InductionCoupler
    %HALCBACHCOUPLER is an InductionCoupler comprising a spinning halbach
    %array
    %   Detailed explanation goes here
    
    properties
   
        r_o = 2.54*0.75*0.01; %0.75 inches converted to m
        r_i = 2.54*0.5*0.01;  %0.5 inches converted to m

        Br = 1.42; %T Magnet strength
        ur = 1.08; %unitless magnet relative permeability
        P = 1; %pole-pairs
        w = 0.01; % width of the magnet
        C; %C property of a halbach array
        omega_max = 4500; %max motor speed in rad/sec
    end
    
    methods
        %% CONSTRUCTOR
        function obj = HalbachCoupler()
            obj = obj@InductionCoupler();
            obj.C = findC(obj.Br,obj.P,obj.ur,obj.r_o,obj.r_i);
            obj.b_source = @(xi,g)obj.B_s(xi,obj.C,obj.P,obj.r_o,g,0);
        end
        
    end
    methods(Static)
        %% SOURCE FIELD FOR A HALBACH ARRAY
        function B_s = halbachSource(xi, C, P, r_o, g, x)

            B_s = 4*pi*C*xi.^P/factorial(P).*exp(-xi.*(r_o + g +1i*x)).*(xi >= 0);
            B_s(isnan(B_s))= 0;
        end
    end
end

