% v = [[0;0;0],[0;1;0],[1;1;0],[1;0;0]];
% mesh = meshgrid(linspace(0,1),linspace(0,1));
% figure(1);clf;
% pts = [];
% for i = linspace(0,1,50)
%     for j = linspace(0,1,50)
%         m = midpoint(v(:,1),v(:,2),v(:,4), i,j);
%         pts = [pts,m];
%         patch(v(1,:),v(2,:),v(3,:),'g'); hold on;
%        % plot3(m(1),m(2),m(3),'*');
%        % hold on;
%     end
% end
%     
% n = cross(v(:,2)-v(:,1),v(:,4)-v(:,1));
% n = n/norm(n);
% center = 0.5*(v(:,2)-v(:,1)+v(:,4)-v(:,1));
% %quiver3(pts(1,:),pts(2,:),pts(3,:),...
% %    n(1)*ones(1,size(pts,2)),n(2)*ones(1,size(pts,2)),n(3)*ones(1,size(pts,2)));
% 
% pt = [0.5;0.3;0.1];
% 
% [d, i] = min(colnorm(pts-pt*ones(1,size(pts,2)),1));
% plot3(pt(1),pt(2),pt(3),'x');
% plot3(pts(1,i),pts(2,i),pts(3,i),'o');
% perp = dot(pt,pts(:,i))/(norm(pt)*norm(pts(:,i)));

[X,Y,Z] = cylinder([1 1 1],50);
[nx,ny,nz] = surfnorm(X,Y,Z);
figure(2);
surf(X,Y,Z);
%hold on;
%quiver3(X,Y,Z,nx,ny,nz);