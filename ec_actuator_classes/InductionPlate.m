classdef InductionPlate < hgsetget
    %INDUCTIONPLATE keeps track of the position and methods for a fixed
    %conductive plate for induction coupling 
    %   Detailed explanation goes here
    
    properties
        origin = [0;0;0]; %origin of the plate in inertial space
        att = [1 0 0 0]; %attitude of the plate in inertial space
        w = 2; %width of the plate
        l = 2; %length of the plate
        t = 0.01; %thickness of the plate
        sigma = 1/2.83E-6;
        rho = 2.83E-6;
        MU0 = 1.256E-6;
        kappa = 0;
        gamma; %transmission function @(xi,v_t,v_n,w_e)
        
    end
    
    properties (SetAccess = 'private')
        couplers = [];
    end
    
    methods
        %% CONSTRUCTOR
        function obj = InductionPlate()
            %INDUCTIONPLATE creates a plate object
            obj.gamma = @(xi, v_t,v_n, w_e)obj.findGamma(xi, v_t, v_n, obj.MU0, obj.sigma, w_e,obj.t);
        end
        
        %% FIND CLOSEST PLATE POINT TO POINT
        
    end
    
    methods(Static)
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
            
            Tss_top = ((lambda-(xi+beta)).*exp(beta*b)-(lambda - (xi - beta)).*exp(-beta*b));
            Tss_bottom = exp(beta*b).*(lambda.^2-(xi + beta).^2)-exp(-beta*b).*(lambda.^2-(xi-beta).^2);
            Tss = Tss_top./Tss_bottom;
            
            Gamma = 2.*xi.*Tss - 1;
            
        end
    end

    
end

