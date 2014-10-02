%Quaternion reminder script
q = [cos(pi/8) 0 0 sin(pi/8)]; %angle is amount that BODY FRAME has rotated wrt world frame 
    A = quat2dcm(q); %A rotates from world to body
    v = [1;0;0]; %vector in global frame
    v_rot = A*v; %vector in body frame