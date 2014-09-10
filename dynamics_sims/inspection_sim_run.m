%inspection sim feedback
%fd = -k_x*xerr + kv*xerrdot 
%tau = -skew(KR * del_R)(inversecarrot) - KW(omega - deltR'*omega_nom)

% function skewA = skew(A)
% skewA = A - A';
% end
% 
% function DA = deltA(A_nom,A)
% DA = A_nom'*A;
% end
inspection_sim_setup;

k_x = 3;
k_v = 2.5;
k_thet = 0.05;
k_omeg = 0.01;
figure(1);clf;
while targetPt < (size(waypts,2)+1) && max(t)<60
    %find the error
    xErr = X(2:3,end)-waypts(2:3,targetPt);
    dxErr = X(5:6,end);
    thetaErr = X(1,end)-waypts(1,targetPt);
    omegaErr = X(4,end);
    % find the command forces
    f_d = -k_x*xErr - k_v*dxErr;
    tau_d = -k_thet*thetaErr - k_omeg*omegaErr;
    c_A_t = rotz(X(1,end)); %find c_A_t rotation matrix from 
    %target (inertial) frame to chaser frame
    % find the resultant motor speeds
    
    u = pinv(J(3:5,:))*[tau_d;c_A_t(2:2,2:2)*f_d];
    u(abs(u)>u_max) = sign(u(abs(u)>u_max))*u_max; 
    
    %propigate forces through the system
    X_new = X(:,end)+A*X(:,end)+B*[tau_d;f_d];
     
    X_new(1) = wrapToPi(X_new(1));
    %check if you're close to a waypoint.
    if (norm(X_new(2:3)-waypts(2:3,targetPt)) < 0.05 && ...
            norm(X_new(1)-waypts(1,targetPt)) < pi/16 &&...
            norm(X_new(4:6)) <0.1 )
        %if you're near a waypoint, go towards the next waypoint
        targetPt = targetPt + 1;
    end
    %update everything
    X = [X,X_new];
    inputs = [inputs, u];
    t = [t,t(end)+dt];
    subplot(311);
    hold on;
    plot(X_new(2),X_new(3));
    hold off;
    subplot(312);
    hold on;
    plot(t(end),X_new(1));
    hold off;
    subplot(313);
    hold on;
    plot(t(end),u);drawnow;
    hold off
end

figure(2); clf;
plot(X(2,:),X(3,:));
hold on
plot(waypts(2,:),waypts(3,:),'*','MarkerSize',20);
xlim([-1,2.5]);ylim([-1,2.5]);
xlabel('x position (m)');
ylabel('y position (m)');
legend('Inspector Trajectory','Waypoints');


figure(3);clf;
subplot(211);

plot(t,X(1,:));
xlabel('time (s)');
ylabel('Heading (radians from X axis)');
subplot(212);
plot(t,inputs);
xlabel('time (s)');
ylabel('Array Speeds (rad/sec)');
legend('Array 1','Array 2','Array 3');
