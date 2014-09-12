classdef InductionCoupler < hgetset
    %INDUCTIONCOUPLER acts like a joint between a conductive surface and a 
    %body
    
    properties
        type; %type of coupler - electromagnet or permanent magnet
        omega = 0; %electromagnetic frequency of the system 
        b_source; %source field
        axis = [1;0;0]; %rotation axis in body coordinates 
        pos = [0;0;0]; %position wrt com in body coordinates
        body; %the coupled body
        plate; %the coupled plate 
    end
    
    methods
        %% CONSTRUCTOR
        function obj = InductionCoupler()
            
        end
        %% GENERATE FORCE 
        function f = genForce(obj)
            %GENFORCE solves for the force on the rigid body in inertial
            %coords
            g = findgap();
            f 
        end
        
        %% FIND GAP BETWEEN THE PLATE AND THE COUPLER
        function g = findGap(obj)
        end
        
        %% FIND THE VELOCITY OF THE COUPLER IN THE PLATE FRAME
        function v = findVelPlate(obj)
        end
    end
    
    methods(Static)
        %% FIND THE FORCE USING THE BIRD TRANSMISSION FUNCTION METHOD
        function f = fourierForce(B_s, Gamma, w)
            %fourierForce finds the steady state force from a single em source with a
            %single time frequency.

            %INPUTS
            % B_s - A function handle wrt xi that is the constant component of the source field. 
            %fourier transformed in the x direction. Complex valued: B_s = B_x + i*B_y
            % Gamma - a function handle wrt xi that is a propigation function through the material
            % w - length of the problem (the plate) in the z direction
            %OUTPUT
            % f- force 2x1 fx,fz
            mu0 = 4*pi*10^-7; % mu doesn't really change
            endPt = 1E3; % used to avoid problems at infinity with EM forces

            integrand = @(xi)(abs(B_s(xi)).^2.*Gamma(xi));
            im_force = w/(8*pi*mu0)*(integral(integrand,-endPt,endPt));
            %negatives to have force on magnet rather than force on plate
            f = [-imag(im_force);-real(im_force)];

        end
    end
end

