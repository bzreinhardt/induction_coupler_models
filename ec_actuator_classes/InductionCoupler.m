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
            %GENFORCE solves for the force and torque on the rigid body 
            % in inertial coords
            g = obj.findGap();
            [v_norm,v_tan] = obj.findVelPlate();
            b_A_n = quat2dcm(obj.body.att);
            axis_n = b_A_n'*obj.axis;
            pos_n = b_A_n'*obj.pos;
            
            %TODO make the plate normal dynamic
            n = [0;0;1];
            f_p = fourierForce(@(xi)obj.b_source(xi,g), ...
                @(xi)obj.plate.gamma(xi,v_tan,v_norm,obj.w_e), obj.w);
            dir_x = cross(axis_n,n);
            dir_y = cross(dir_x,axis_n);
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
        
        %% Draw the system
        function [b,p,c] = drawCoupledBot(obj,varargin)
            %h = object handle
            if nargin > 1
                varargin{1} = f;
                figure(f);
            end
             %draw an arm to the coupler
             b_A_n = quat2dcm(obj.body.att);
            %arm in body coordinates
            arm_width = 0.1;
            arm_theta = atan2(obj.pos(2),obj.pos(1));
            arm_arm_coords = [0 0 norm(obj.pos(1:2)) norm(obj.pos(1:2));...
                             -arm_width/2 arm_width/2 arm_width/2 -arm_width/2];
            arm_b = [cos(arm_theta) -sin(arm_theta); ...
                sin(arm_theta) cos(arm_theta)]*arm_arm_coords;
            arm_n = b_A_n'*[arm_b;zeros(1,4)] + obj.body.pos*ones(1,4);
            patch(arm_n(1,:),arm_n(2,:),[50/255, 50/255, 50/255, 50/255]);
            %draw a circle and rectange for the coupler itself
            obj.draw2Dcoupler();
            %draw a rectangle for the plate
            
            hold on;
            %draw a rectangle for the body
  
            b_x = obj.body.pos(1); b_y = obj.body.pos(2);
            b_w = obj.body.sx; b_l = obj.body.sy;
            b_X = [b_x-b_w/2 b_x-b_w/2 b_x+b_w/2 b_x+b_w/2];
            b_Y = [b_y-b_l/2 b_y+b_l/2 b_y+b_l/2 b_y-b_l/2];
            b_Z = zeros(size(b_Y));
            
            b_n = b_A_n'*[b_X;b_Y;b_Z];
            b = patch(b_n(1,:),b_n(2,:),[50/255, 50/255, 50/255, 50/255]);
            %draw the body's com
           
            hold off;
        end
        
         %% DRAW A SINGLE COUPLER
        
        function draw2Dcoupler (obj,varargin)
            %             if nargin > 1
            %             for i = 1:(length(varargin)/2)
            %                     if ~ischar(varargin{2*i-1})
            %                         error('draw2Dcoupler:input', 'Property designations must be strings.')
            %                     end
            %                     switch lower(varargin{2*i-1})
            %                         case {'time' 't'}
            %                             if ~isnumeric(varargin{2*i})
            %                                 error('BODY:body:input', 'Time must be a number.')
            %                             else
            %                                 bod.time = varargin{2*i};
            %                             end
            %                     end
            %             end
            %             end
             b_A_n = quat2dcm(obj.body.att);
            width = 0.25; height = 0.1; [midpt,~] = obj.getInertialCoords();
            arm_theta = atan2(obj.axis(2),obj.axis(1));
            arm_arm_coords = [-height/2 -height/2 height/2 height/2;...
                             -width/2 width/2 width/2 -width/2];
            arm_b = [cos(arm_theta) -sin(arm_theta); ...
                sin(arm_theta) cos(arm_theta)]*arm_arm_coords+obj.pos(1:2)...
                *ones(1,4);
            arm_n = b_A_n'*[arm_b;zeros(1,4)] + obj.body.pos*ones(1,4);
            patch(arm_n(1,:),arm_n(2,:),'k');
            
      %      rectangle('Position',[midpt(1)-width/2,midpt(2)-height/2,width,height],...
      %          'FaceColor','k');
            drawCircle(height,midpt(1:2),'w');
            
        end
        
        %% Get the Coupler Position in inertial coordinates
        function [x_n, a_n] = getInertialCoords(obj)
            b_A_n = quat2dcm(obj.body.att);
            x_n = b_A_n'*obj.pos + obj.body.pos;
            a_n = b_A_n'*obj.axis;
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

