function distance=rail_ray_distance(origin,direction,step,bend,top)
% Ray against both smooth half-rails and the vertical face of an upward step.
best=inf;
for side=1:2
    rise=(side-1)*step;
    lambda=(top+rise-origin(3))/direction(3);
    for k=1:12
        x=origin(1)+lambda*direction(1);
        a=(x-.25)/.07; bump=bend*exp(-.5*a*a);
        h=top+rise+bump; dh=-a/.07*bump;
        denom=direction(3)-dh*direction(1);
        if abs(denom)<1e-9, break; end
        lambda=lambda-(origin(3)+lambda*direction(3)-h)/denom;
    end
    x=origin(1)+lambda*direction(1);
    if lambda>=0 && ((side==1 && x<=0)||(side==2 && x>=0))
        best=min(best,lambda);
    end
end
if abs(direction(1))>1e-9 && step>0
    lambda=-origin(1)/direction(1);
    height=origin(3)+lambda*direction(3);
    base=top+bend*exp(-.5*(.25/.07)^2);
    if lambda>=0 && height>=base && height<=base+step, best=min(best,lambda); end
end
if isinf(best), distance=NaN; else, distance=best; end
end
