%open loop simulation of the system
t = t0;
X = [x0;v0;q0';om0];
t_his = t;
x_his = X';
f_his = zeros(1,3);
tau_his = zeros(1,3);
u_his = zeros(1,2);
%quaternion is cosine first. 
% Angle is angle to go from body coordinates to inertial coordinates
%matlab quat2dcm takes a quaternion described above and finds the dcm
%to take a vector from inertial to body coordinates
%run an initialization script

while t < t_max
    %update inputs
    %test with just constant input, should go in straight line
    u = [1;-1];
    %find net force
    f_net = 0;
    tau_net = 0;
    A = quat2dcm(X(7:10)');
    for i = 1:size(a,2) %loop over all end effectors
        g = findgap_xyplate(X,d(:,i));%gap between effector and plate 
        v_t = sqrt(X(4)^2 + X(5)^2); 
        switch magType(i)
            case 'p'
                f = dynamicArrayF(u(i),X,a(:,i),d(:,i),B_array,gamma_v,w);
            case 'e'
            otherwise 
                error('not a designated magnet type');
        end
    
        
        f_net = f_net + f; %force in inertial frame
        
        %find net torque in inertial frame
        tau = cross(f,A'*d(:,i));
        tau_net = tau_net + tau;
    end
    %update state
    t = t + dt; %increment time
    X(1:3) = X(1:3) + X(4:6)*dt;
    X(4:6) = X(4:6) + f_net/m*dt; %yay Newton
    X(7:10) = X(7:10) + 0.5*quatmultiply(X(7:10)',[0 X(11:13)'])'*dt;
    X(11:13) = X(11:13)+inv(I)*(A*tau_net - cross(X(11:13),I*X(11:13)))*dt; %yay Euler
    t_his = [t_his; t'];
    x_his = [x_his; X'];
    f_his = [f_his; f'];
    tau_his = [tau_his; tau'];
    u_his = [u_his;u'];
    
    
end

figure(1);
subplot(211); plot(x_his(:,1),x_his(:,2));
subplot(212); plot(t_his, x_his(:,4:6)); legend('v_x','v_y','v_z');
