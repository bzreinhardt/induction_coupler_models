classdef modelOptimizer < hgsetget
    %MODELOPTIMIZER Optimizes cost functions for parameterized models
    %   Detailed explanation goes here
    
    properties
        cost_fcn; %scalar cost function that takes a vector
        x0; %initial set of parameters
        constraints;
        opt_fcn = 'fminunc';
        opt_params;
    end
    
    methods
        %% CONSTRUCTOR 
        function obj = modelOptimizer(cost_fcn, x0, varargin)
            obj.cost_fcn = cost_fcn;
            obj.x0 = x0;
        end
        
        function out_params = runOptimizer(obj)
            out_params = [];
            switch obj.opt_fcn
                case 'fminunc'
                    out_params = fminunc(obj.cost_fcn,obj.x0);
            end
            obj.opt_params = out_params;
        end
    end
    
end

%% TODO - different things to implement from Matt Kelly

% Class zero:
%  - Problem can be expressed as a linear matrix inequality
%  - Stephen Boyd's CVX toolbox for matlab
% 
% Class One:
%  - nonlinear constraint / objective
%  - smooth & consistent objective
%  - smooth & consistent constraints
%  - SNOPT, FMINCON (matlab) , IPOPT
%  - --> SNOPT is generally considered to be the best available program for this, particularly if you know the gradients. MAE has a department license. It can be called from Matlab (very easy) or from C++ (not so easy, but better with linux). FMINCON is also pretty good.
% 
% Class Two:
%  - nonlinear objective
%  - problem is reasonable, but not necessarily smooth or consistent
%  - constraints are handled directly in cost function - no explicit constraints
%  - CMAES, Simulated Anealing (Easy to code up, good as an idoit test), Matlab's genetic algorithm toolbox, write your own evolutionary algorithm (don't do this)
%  - --> CMAES is generally considered the best (with a high weight on usability, implementation, and generality). Source code is available in many launguages:
% https://www.lri.fr/~hansen/cmaes_inmatlab.html    
% The matlab implementation works very well - I used to to generate the controller in my ICRA paper.
% 
% Trajectory Optimization:
%  - Your constraints and objective must be smooth or else all hope is lost.
%  - If you want to find optimal trajectories, then you will need a transcription algorithm to wrap one of the Class One optimizers. I've written several of my own, but it is super tedious. By far the best program is GPOPS2. 
%  - If your problem is simple, then it can sometimes be solved by Matlab: bvp4c and bvp5c