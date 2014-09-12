classdef InductionCoupler < hgsetget
    %INDUCTIONCOUPLER acts like a joint between a conductive surface and a 
    %body
    
    properties
        type; %type of coupler - electromagnet or permanent magnet
        w_e = 0; %electromagnetic frequency of the system 
        b_source; %source field - @(xi,g)
        axis = [1;0;0]; %rotation axis in body coordinates 
        pos = [0;0;0]; %position wrt com in body coordinates
        body; %the coupled body
        plate; %the coupled plate 
        w = 0.01; % width of the magnet
    end
    
    methods
        %% CONSTRUCTOR
        function obj = InductionCoupler(varargin)
            %parse inputs
        end
        %% GENERATE FORCE and TORQUE
        function [f,tau] = genForce(obj)
            %GENFORCE solves for the force and toruqe on the rigid body 
            % in inertial coords
            g = findgap();
            [v_norm,v_tan] = findVelPlate();
            b_A_n = quat2dcm(obj.bod.att);
            axis_n = b_A_n'*obj.axis;
            pos_n = b_A_n'*obj.pos;
            
            %TODO make the plate normal dynamic
            n = [0;0;1];
            f_p = fourierForce(@(xi)obj.b_source(xi,g), ...
                @(xi)obj.plateGamma(xi,v_tan,v_norm,obj.w_e), obj.w);
            dir_x = cross(axis_n,n);
            dir_y = cross(dir_x,a);
            f = f_p(1)*dir_x + f_p(2)*dir_y;
            
            tau = cross(f,pos_n);
            
        end
        
        %% FIND GAP BETWEEN THE PLATE AND THE COUPLER
        function g = findGap(obj)
            %FINDGAP finds the minimum distance between the coupler and the
            %surface it's coupling
            %find position of the coupler in inertial space
            b_A_n = quat2dcm(obj.body.att);
            p_A_n = quat2dcm(obj.plate.att);
            x_n = b_A_n'*obj.pos + obj.body.pos;
            
            %TODO find distance for a not-flat plate
            % built in assumption that plate is lying in x-y plane
           g = x_n(3);  
            %
        end
        
        %% FIND THE VELOCITY OF THE COUPLER IN THE PLATE FRAME
        function [v_norm,v_tan] = findVelPlate(obj)
            
            %for now assume static plate
            norm_plate = [0;0;1];
            v_n = obj.body.vel + cross(obj.body.om,obj.pos); %velocity of actuator in inertial space
            v_norm = dot(norm_plate,v_n);
            v_tan = norm(v_n - v_norm*norm_plate);
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

