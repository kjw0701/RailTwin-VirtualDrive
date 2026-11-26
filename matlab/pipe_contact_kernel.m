function force=pipe_contact_kernel(input)
% Fixed-size Simulink port: position(3), velocity(3), contact parameters(3),
% followed by one [startX, endX, topZ] triple for each pipe section.
position=input(1:3);
velocity=input(4:6);
radius=input(7);
stiffness=input(8);
damping=input(9);
segments=reshape(input(10:end),3,[]);
nearestDistance=inf;
nearestRadial=zeros(3,1);
for segment=1:size(segments,2)
    edgeX=min(max(position(1),segments(1,segment)),segments(2,segment));
    dx=position(1)-edgeX;
    dz=position(3)-segments(3,segment);
    distance=hypot(dx,dz);
    if distance<nearestDistance
        nearestDistance=distance;
        nearestRadial=[dx;0;dz];
    end
end
force=zeros(3,1);
if nearestDistance>0 && nearestDistance<radius
    normal=nearestRadial/nearestDistance;
    closingVelocity=normal(1)*velocity(1)+normal(3)*velocity(3);
    magnitude=stiffness*(radius-nearestDistance)-damping*closingVelocity;
    force=normal*max(0,magnitude);
end
end
