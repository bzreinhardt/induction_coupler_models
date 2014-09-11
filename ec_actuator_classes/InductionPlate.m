classdef InductionPlate < hgsetget
    %INDUCTIONPLATE keeps track of the position and methods for a fixed
    %conductive plate for induction coupling 
    %   Detailed explanation goes here
    
    properties
        origin = [0;0;0]; %origin of the plate in inertial space
        attitude = [1 0 0 0]; %attitude of the plate in inertial space
        w = 1; %width of the plate
        l = 1; %length of the plate
        t = 0.01; %thickness of the plate
        sigma = 1/2.83E-6;
        rho = 2.83E-6;
        MU0 = 1.256E-6;
        kappa = 0;
        gamma; %transmission function
        
    end
    
    properties (SetAccess = 'private')
        couplers = [];
    end
    
    methods
%% CONSTRUCTOR
function obj = InductionPlate()
    %INDUCTIONPLATE creates a plate object
    gamma = @(xi, v_t,v_n, w_e)findGamma(xi, v_t, v_n, MU0, sigma, w_e,b);
        
    end
    
end

