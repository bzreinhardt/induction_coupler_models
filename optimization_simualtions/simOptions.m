classdef simOptions
    %SIMOPTIONS sets up the options for induction coupler simulations
    %   Detailed explanation goes here
    
    properties
        out = {'x','att','point'};
        control_function; %function handle to control function 
    end
    
    methods
        %% CONSTRUCTOR
        function obj = simOptions(varargin)
            
            %parse inputs
            if nargin == 1 && isa(varargin{1}, 'simOptions')
                %if the input is a simParams, return it
                obj = varargin{1};
            elseif nargin > 0
                %inputs should be string, val, string val etc
                for i = 1:2:narargin
                    switch varargin{i}
                    end
                end
            end
        end
            
    end
    
end

