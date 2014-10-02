%open loop simulation of the system
clear all;
t_max = 5;
dt = 0.1;
t = 0;
x_body = [-1.2;0;0];
att_body = [1 0 0 0];

t_his = t;
x_his = x_body';
att_his = att_body;
v_his = zeros(1,3);
om_his = zeros(1,3);
f_his = zeros(1,3);
tau_his = zeros(1,3);
u_his = zeros(1,1);
%quaternion is cosine first. 
% Angle is angle to go from inertial coordinates to body coordiantes coordinates
%matlab quat2dcm takes a quaternion described above and finds the dcm
%to take a vector from inertial to body coordinates
%run an initialization script


array1 = HalbachCoupler();


array1.pos = [0;0;0];
array1.axis = [0;0;1];
plate = InductionPlate('curve');
bod = body(x_body, att_body);
bod.force = [0;0;0];
bod.torque = [0;0;0];
set(array1,'body',bod);
set(array1,'plate',plate);

figure(1);clf; axis([-2 2 -2 2]);
p = plate.draw2D();
set(p,'HandleVisibility','off');
time_pos = [-1.2,-1.2];
f_pos = [-0.5, -1.5];
tau_pos = [0.8, -1.5];
h = text(time_pos(1),time_pos(2), strcat('time = ',num2str(t)));
h2 = text(f_pos(1),f_pos(2),strcat('force = ',num2str(bod.force)));
h3 = text(tau_pos(1),tau_pos(2),strcat('torque = ',num2str(bod.torque)));
while t < t_max
    %update inputs
    array1.w_e = 100;
    
    u = [array1.w_e];
    %test with just constant input, should go in straight line
    
    %find net force/torque
    [f1 t1] = array1.genForce();
   
    bod.force = f1;
    bod.torque = t1;
    %update state
    t = t + dt; %increment time
    A = quat2dcm(bod.att);
    bod.pos = bod.pos + bod.vel*dt;
    f_his = [f_his;bod.force'];
    tau_his = [tau_his;bod.torque'];
    bod.force = [bod.force(1:2);0]; %constrain to the plane
    bod.torque = [0;0;bod.torque(3)];
    
    bod.vel = bod.vel + bod.force/bod.mass*dt; %yay Newton
    
    %propigation equations from 
    %http://www-users.cs.umn.edu/~trawny/Publications/Quaternions_3D.pdf
    if(norm(bod.om) < 0.000001)
        bod.att = ((eye(4)+dt/2*[crsMrx(bod.om) bod.om; -bod.om' 0])...
            *bod.att')';
    else
    
     bod.att = quatmultiply(bod.att,[cos(norm(bod.om)/2*dt), ...
        bod.om'/norm(bod.om)*sin(norm(bod.om)/2*dt)]);
    end
    bod.om = bod.om+inv(bod.inertia)*(quatrotate(bod.att,bod.torque')'...
        - cross(bod.om,bod.inertia*bod.om))*dt; %yay Euler
    
    t_his = [t_his; t'];
    x_his = [x_his; bod.pos'];
    att_his = [att_his; bod.att];
    v_his = [v_his; bod.vel'];
    om_his = [om_his;bod.om'];
    f_his = [f_his;bod.force'];
    tau_his = [tau_his;bod.torque'];
    u_his = [u_his;u'];
     cla;
    array1.drawCoupledBot();hold on;
   
   
%     delete(h);
%     delete(h2);
%     delete(h3);
    h = text(time_pos(1),time_pos(2), strcat('time = ',num2str(t)));
    h2 = text(f_pos(1),f_pos(2),strcat('force = ',num2str(bod.force)));
    h3 = text(tau_pos(1),tau_pos(2),strcat('torque = ',num2str(bod.torque)));
    drawnow;
end
figure(2);
subplot(211); plot(x_his(:,1),x_his(:,2));
subplot(212); plot(t_his, v_his); legend('v_x','v_y','v_z');
