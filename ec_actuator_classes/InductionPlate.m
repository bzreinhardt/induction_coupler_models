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
        type;
        mesh;
        norm;
        
    end
    
    properties (SetAccess = 'private')
        couplers = [];
    end
    
    methods
        %% CONSTRUCTOR
        function obj = InductionPlate(type,varargin)
            %INDUCTIONPLATE creates a plate object
            obj.gamma = @(xi, v_t,v_n, w_e)obj.findGamma(xi, v_t, v_n, obj.MU0, obj.sigma, w_e,obj.t);
            obj.type = type;
            obj.defineSurface();
        end
        
        %% FIND CLOSEST PLATE POINT TO POINT
        
        %% DRAW THE PLATE
        function p = draw(obj)
            if obj.kappa == 0;
                width = obj.w; length = obj.l;
                p = rectangle('Position',[-width/2,-length/2,width,length],'FaceColor',[204/255 204/255 204/255]);
            end
        end
        
        %% DRAW PLATE IN 2D
        function p = draw2D(obj)
            switch obj.type
                case 'flat'
                    x_min = min(obj.mesh{1}(:));
                    x_max = max(obj.mesh{1}(:));
                    y_min = min(obj.mesh{2}(:));
                    y_max = max(obj.mesh{2}(:));
                    p = patch([x_min x_min x_max x_max],...
                        [y_min y_max y_max y_min],[204/255 204/255 204/255]);
                    
                case 'curve'
                    p = plot(obj.mesh{1}(1,:),obj.mesh{2}(1,:));
                otherwise 
                     error('Surface must be plate or curve for now');
            end
        end
        
        %% PLATE SURFACE POINTS
        function defineSurface(obj)
            %DEFINESURFCE finds a mesh of points that define the surface
            %and the associated normals
            switch obj.type
                case 'flat'
                    %for now only planar plates
                    [X,Y] = meshgrid(linspace(obj.origin(1)-obj.w/2,obj.origin(1)+obj.w/2,200),...
                        linspace(obj.origin(2)-obj.l/2,obj.origin(2)+obj.l/2,200));
                    Z = obj.origin(3)*ones(size(X));
                    [Nx,Ny,Nz] = surfnorm(X,Y,Z);
                    obj.mesh = {X,Y,Z};
                    obj.norm = {Nx,Ny,Nz};
                case 'curve'
                    if obj.kappa == 0
                        obj.kappa = 1;
                    end
                    [X,Y,Z] = cylinder(1/obj.kappa*ones(1,4),300);
                    X = X+obj.origin(1); Y = Y + obj.origin(2); Z = Z+obj.origin(3);
                    [Nx,Ny,Nz] = surfnorm(X,Y,Z);
                    obj.mesh = {X,Y,Z};
                    obj.norm = {Nx,Ny,Nz};
                otherwise
                    error('Surface must be plate or curve for now');
            end
        end
        %% FIND CLOSEST POINT ON THE SURFACE
        function [g,norm,pt] = nearestPt(obj,pt)
            dist = (obj.mesh{1}-pt(1)).^2 + (obj.mesh{2}-pt(2)).^2 + ...
                (obj.mesh{3}-pt(3)).^2;
            [g2,i] = min(dist(:));
            g = sqrt(g2);
            norm = [obj.norm{1}(i); obj.norm{2}(i); obj.norm{3}(i)];
            pt = [obj.mesh{1}(i); obj.mesh{2}(i); obj.mesh{3}(i)];
        end
        
        %% SET Kappa
        function obj = set.kappa(obj,new_kappa)
            obj.kappa = new_kappa;
            obj.defineSurface();
            
        end
        %% SET w
        function obj = set.w(obj, new_w)
            obj.w = new_w;
            obj.defineSurface();
        end
         %% SET l
        function obj = set.l(obj, new_l)
            obj.l = new_l;
            obj.defineSurface();
        end
        
        
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

