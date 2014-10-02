%open loop simulation of the system
clear all;
load('plate_body_system_with_arm.mat')
load('overhead_with_arm.mat')

t = 0;
x_body = [0;0;0.01];
att_body = [1 0 0 0];

t_his = t;
x_his = x_body';
att_his = att_body;
v_his = zeros(1,3);
om_his = zeros(1,3);
f_his = zeros(1,3);
tau_his = zeros(1,3);

%quaternion is cosine first. 
% Angle is angle to go from inertial coordinates to body coordiantes coordinates
%matlab quat2dcm takes a quaternion described above and finds the dcm
%to take a vector from inertial to body coordinates
%run an initialization script

arrays = {array1, array2};

u_his = zeros(size(arrays));


plate = InductionPlate('flat');

bod.force = [0;0;0];
bod.torque = [0;0;0];

array1.pos = [X_m1;0];
array2.pos = [X_m2;0];
array1.axis = [array1.axis;0];
array2.axis = [array2.axis;0];
set(array1,'plate',plate);
set(array2,'plate',plate);

dar = [0.13;0.05;0];
ar_b = [-X_ar(1);X_ar(2);0] + dar;

figure(1);clf; 
p = plate.draw2D();
axis([bod.pos(1)-1.3 bod.pos(1)+1.3 bod.pos(2)-1.3 bod.pos(2)+1.3]);
set(p,'HandleVisibility','off');

time_pos = [bod.pos(1)-1.2,bod.pos(2)-0.75];
f_pos = [bod.pos(1)-0.75,bod.pos(2)-0.75];
tau_pos = [bod.pos(1)-0.1,bod.pos(2)-0.75];

h = text(time_pos(1),time_pos(2), strcat('time = ',num2str(t)));
h2 = text(f_pos(1),f_pos(2),strcat('force = ',num2str(bod.force)));
h3 = text(tau_pos(1),tau_pos(2),strcat('torque = ',num2str(bod.torque)));

t_max = 10;
dt = 0.05;
while t < t_max
    %update inputs
    if t < t_max/2;
    array1.w_e = 31.7; %303 rpm/60s/m*2*pi rad/r
    array2.w_e = -31.7;
    else
        array1.w_e = -31.7; %303 rpm/60s/m*2*pi rad/r
        array2.w_e = 31.7;
    end
    u = zeros(length(arrays),1);
    for i = 1:length(arrays)
        u(i) = arrays{i}.w_e;
    end
    %test with just constant input, should go in straight line
    
    %find net force/torque
    [f1, t1] = array1.genForce();
    [f2, t2] = array2.genForce();
    
    bod.force = f1+f2;
    bod.torque = t1+t2;
    
    %update state
    t = t + dt; %increment time
    A = quat2dcm(bod.att);
    bod.pos = bod.pos + bod.vel*dt;
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
     for i = 1:length(arrays)
         arrays{i}.drawCoupledBot();hold on;
     end

%     delete(h);
%     delete(h2);
%     delete(h3);
    h = text(time_pos(1),time_pos(2), strcat('time = ',num2str(t)));
    h2 = text(f_pos(1),f_pos(2),strcat('force = ',num2str(bod.force)));
    h3 = text(tau_pos(1),tau_pos(2),strcat('torque = ',num2str(bod.torque)));
    drawnow;
end
figure(2);
ar_his = quatrotate(att_his,ones(size(att_his,1),1)*ar_b')+x_his;
subplot(211); plot(ar_his(:,1),ar_his(:,2));
subplot(212); plot(t_his, v_his); legend('v_x','v_y','v_z');
xlabel('time(s)'); ylabel('velocity');
figure(3); clf;

[haxes,hline1,hline2] = plotyy([t_his t_his t_his],[f_his(:,1), f_his(:,2), tau_his(:,3)], t_his,u_his);

