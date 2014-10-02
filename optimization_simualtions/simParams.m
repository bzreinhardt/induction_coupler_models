classdef simParams < hgsetget
    %SIMPARAMS holds the parameters of a simulation
    %   Detailed explanation goes here
    
    properties
        %initial states for the simulation
        
        om0 = zeros(3,1);
        v0 = zeros(3,1);
        x0 = zeros(3,1);
        att0 = [1 0 0 0];
        
        % things in body coordinates
        point = [0;0;0]; %point to track on a body
        plate_type = 'flat';
        plate;
        a_couplers = [0 1 0; 0 -1 0]; %coupler axes body coordinates
        d_couplers = 0.25*[0 1 0; 0 -1 0]; %coupler positions in body coordinates
    end
    
    methods
        %% CONSTRUCTOR
        function obj = simParams(varargin)
            
            %parse inputs
            if nargin == 1 && isa(varargin{1}, 'simParams')
                %if the input is a simParams, return it
                obj = varargin{1};
            elseif nargin > 0
                %inputs should be string, val, string val etc
                for i = 1:2:narargin
                    switch varargin{i}
                    end
                end
            end
            %initialize a plate for the simulation
            obj.plate = InductionPlate(obj.plate_type);
        end
        %% SET ATTITUDE
         function obj = set.att0(obj, newAtt)
            
            %% --Parse inputs
            if ~isnumeric(newAtt)
                error('SIMPARAMS:set:input', 'SIMPARAMS attitude must be numeric.')
            elseif all(size(newAtt) == [4 1])
                newAtt = newAtt';
                warning('making quaternion 4x1');
            elseif numel(newAtt) ~= 4
                error('SIMPARAMS:set:input', 'SIMPARAMS attitude must be a 4-element matrix.')
            end
            
            newAtt = newAtt/norm(newAtt);
            obj.att0 = newAtt;  
         end
         
         %% SET INDUCTION COUPLER POSITIONS
         function obj = set.a_couplers(obj, new_a)
             if size(new_a,1) == 1 
                 block_a = [];
                 for i = 1:3:length(new_a)
                     block_a = [block_a;new_a(i:i+2)];
                 end
                  obj.a_couplers = block_a;
             elseif size(new_a,2) == 1
                 block_a = [];
                 for i = 1:3:length(new_a)
                     block_a = [block_a;new_a(i:i+2)'];
                 end
                  obj.a_couplers = block_a;
             else
                 obj.a_couplers = new_a;
             end
         end
         %% SET INDUCTION COUPLER AXES
    end
    
end

