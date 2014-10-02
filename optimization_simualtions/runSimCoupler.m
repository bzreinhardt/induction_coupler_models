function [ out ] = runSimCoupler(t, params, options)
%RUNSIM runs a simulation of a spinning body with the given input parameters 

%Assumptions - all actuation is occuring around a single plate for now
% Parameters that need to be optimized over 
%INPUTS
    %t - vector of timestamps
    %om0
    %v0
    %x0
    %q0
    %tracking position
    %control_his
    % plate   - conductive plate for the simulation
    % d_couplers - coupler positions (in body frame) [Nx3]
    % a_couplers - coupler axes (in body frame) [Nx3]
%PROCESS INPUTS
a_couplers = params.a_couplers;
d_couplers = params.d_couplers;
if size(d_couplers,1) ~= size(a_couplers,1)
    error('coupler specifications dont match lengths');
end
%OUTPUTS


dt = t(2:end)-t(1:end-1);
k = length(t);

%initialize the body
bod = body(params.x0, params.att0);
bod.force = zeros(3,1);
bod.torque = zeros(3,1);
bod.sx = 0.5;bod.sy = 0.5; bod.sz= 0.5;
bod.vel = params.v0;
bod.om = params.om0;


%put the arrays on the body
n = size(a_couplers,1);
arrays = cell(n);
for i = 1:n
    ar = HalbachCoupler();
    ar.pos = d_couplers(n,:)';
    ar.axis = a_couplers(n,:)';
    set(ar, 'body', bod);
    set(ar, 'plate',params.plate);
    arrays{i} = ar;
end

%initialize history storage
x_his = zeros(k,3);
x_his(1,:) = bod.pos';

att_his = zeros(k,4);
att_his(1,:) = bod.att;

v_his = zeros(k,3);
v_his(1,:) = bod.vel';

om_his = zeros(k,3);
om_his(1,:) = bod.om';

A = quat2dcm(bod.att);
point_his = zeros(k,3); %tracks the poisition of the non - com point of interst
point_his(1,:) = A'*params.point + bod.pos;

u_his = zeros(k,n);
tau_his = zeros(k,3);
f_his = zeros(k,3);
%run the simulation
for i = 1:length(dt)
   
    %TODO actually use a useful integrator
    %% Apply conttrol laws
    u = zeros(n,1);
    %% Find resultant force/torque in world frame
    inst_force = zeros(3,n);
    inst_torque = zeros(3,n);
    for j = 1:n
        [inst_force(:,j),inst_torque(:,j)] = arrays{j}.genForce();
        bod.force = sum(inst_force,2); 
        bod.torque = sum(inst_torque,2); 
    end
    
    %apply constraints
    % things like constrain to the plane
    
    %% Update state
    A = quat2dcm(bod.att); %world to body
    point = A'*params.point + bod.pos;
    bod.pos = bod.pos + bod.vel*dt(i);
    bod.vel = bod.vel + bod.force/bod.mass*dt(i); %yay Newton
    %Apply linearized quaternion update if rotational velocity is small
    %because otherwise you run into numerical problems
     if(norm(bod.om) < 0.000001)
        bod.att = ((eye(4)+dt(i)/2*[crsMrx(bod.om) bod.om; -bod.om' 0])...
            *bod.att')';
     else
    %Normal quaternion update
     bod.att = quatmultiply(bod.att,[cos(norm(bod.om)/2*dt(i)), ...
        bod.om'/norm(bod.om)*sin(norm(bod.om)/2*dt(i))]);
     end
    %update rotational velocity
    bod.om = bod.om+inv(bod.inertia)*(quatrotate(bod.att,bod.torque')'...
        - cross(bod.om,bod.inertia*bod.om))*dt(i); %yay Euler
    
    
    %% Record states
    x_his(i+1,:) = bod.pos';
    att_his(i+1,:) =  bod.att;
    v_his(i+1,:) =  bod.vel';
    om_his(i+1,:) = bod.om';
    f_his(i+1,:) = bod.force';
    tau_his(i+1,:) = bod.torque';
    u_his(i+1,:) = u';
    point_his(i+1,:) = point';
end

%cosntruct output structure 
out = struct;
for k = 1: length(options.out)
    switch options.out{k}
        case 'x'
            out.x = x_his;
        case 'att'
            out.att = att_his;
        case 'om' 
            out.om = om_his;
        case 'point' 
            out.point = point_his;
    end
    
end

end

