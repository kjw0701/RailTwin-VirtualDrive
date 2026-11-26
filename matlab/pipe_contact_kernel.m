function force=pipe_contact_kernel(u)
% Nearest point on the three finite tread surfaces, including end corners.
best=inf; dx=0; dz=0;
for j=1:3
    edge=min(max(u(1),u(11+2*j)),u(12+2*j));
    rx=u(1)-edge; rz=u(3)-(u(12)+(j==2)*u(7));
    dist=hypot(rx,rz);
    if dist<best
        best=dist; dx=rx; dz=rz;
    end
end
force=zeros(3,1);
if best<u(9) && best>0
    normal=[dx;0;dz]/best;
    force=normal*max(0,u(10)*(u(9)-best)-u(11)*(normal(1)*u(4)+normal(3)*u(6)));
end
end
