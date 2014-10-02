function [ cost ] = simCost( init_params, param_list, params, options, data)
%COMCOST calculates a cost function on the difference between simulated and
%experimental data
% init_params are the parameters of the simulation to optimize over
% form: [param{1};param{2}; etc..]
%param list should be array of form 'str',size
%data is data to be compared to
%   %note - options need to be some subset of the fieldnames of data

%take param_list and turn init_params into a parameter object for the
%simulation
j = 1;
for i = 1:2:length(param_list)
    param_len = param_list{i+1};
    single_parameter = init_params(j:j+param_len-1);
    j = j+param_len;
    params.(matlab.lang.makeValidName(param_list{i})) = single_parameter;
end

%this should become an anonymous function
out = runSimCoupler(data.t,params,options);

fields = fieldnames(out);
cost = 0;
%compute the cost
%note - options need to be some subset of the fieldnames of data
for i = 1:length(fields)
    diff = out.(matlab.lang.makeValidName(fields{i}))-...
        data.(matlab.lang.makeValidName(fields{i}));
    cost = cost + sum(sum(diff.^2,1),2);
end

end

