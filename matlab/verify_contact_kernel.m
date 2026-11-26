function verify_contact_kernel
r=.0235; top=.0225; k=1e5; c=100;
for x=[-.01 -1e-8 0 1e-8 .01]
    f=rail_contact_kernel([x 0 top+r-1e-4 0 0 0 0 0 r k c top]);
    assert(abs(f(3)-10)<1e-6 && abs(f(1))<1e-6,'Flat rail seam artifact');
end
f=rail_contact_kernel([-.1 0 top+r+.001 0 0 0 0 0 r k c top]);
assert(norm(f)==0,'Airborne wheel must have no tensile contact force');
h=.002; xc=-sqrt(2*r*h-h*h);
f=rail_contact_kernel([xc 0 top+r 0 0 0 h 0 r k c top]);
assert(norm(f)<1e-6,'First corner touch must have zero elastic penetration');
f=rail_contact_kernel([xc+.0001 0 top+r 0 0 0 h 0 r k c top]);
assert(f(1)<0 && f(3)>0,'Step normal must push upward and backward');
fprintf('Contact kernel checks passed: flat seam, airborne, corner geometry.\n');
end
