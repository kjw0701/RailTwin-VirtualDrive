function force=rail_contact_kernel(u)
% Circular wheel normal contact with an analytical rail profile.
% u=[px py pz vx vy vz step bend radius k damping rail_top].
% Each rail is the union of two smooth half-profiles, meeting at x=0.
% Select the nearest surface instead of summing duplicate seam contacts.
% No tread/flange, lateral, friction, or flexible-body contact is modeled.
x=u(1); z=u(3); vx=u(4); vz=u(6);
step=u(7); bend=u(8); radius=u(9); stiffness=u(10); damping=u(11); top=u(12);
best=1e30; nx=0; nz=1;
for side=1:2
    if side==1, lo=-10; hi=0; rise=0; else, lo=0; hi=10; rise=step; end
    s=min(max(x,lo),hi);
    for it=1:12
        [h,dh,ddh]=profile(s,bend,top+rise);
        denom=1+dh*dh+(h-z)*ddh;
        delta=((s-x)+(h-z)*dh)/max(denom,.2);
        delta=min(max(delta,-radius/2),radius/2);
        s=min(max(s-delta,lo),hi);
    end
    [h,dh,~]=profile(s,bend,top+rise);
    dx=x-s; dz=z-h; dist=sqrt(dx*dx+dz*dz);
    if dist<best
        best=dist;
        if dist>1e-12, nx=dx/dist; nz=dz/dist;
        else, nx=-dh/sqrt(1+dh*dh); nz=1/sqrt(1+dh*dh); end
    end
end
penetration=radius-best;
force=zeros(3,1);
if penetration>0
    normal_force=max(0,stiffness*penetration-damping*(nx*vx+nz*vz));
    force(1)=normal_force*nx; force(3)=normal_force*nz;
end
end

function [h,dh,ddh]=profile(x,amplitude,top)
sigma=.07; center=.25; a=(x-center)/sigma;
bump=amplitude*exp(-.5*a*a);
h=top+bump; dh=-a/sigma*bump; ddh=(a*a-1)/(sigma*sigma)*bump;
end
