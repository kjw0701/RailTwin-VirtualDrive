function filename=make_rail_visual(c,side)
% Visual mesh only. Physical normal contact uses rail_contact_kernel.
here=fileparts(mfilename('fullpath')); filename=fullfile(here,'cad',['rail_' side '.stl']);
if isfield(c,'pipe_bounds')
    filename=fullfile(here,'cad',['paper_pipe_' side '.stl']);
    source=stlread(fullfile(here,'cad',c.pipe_file)); p=source.Points(:,[3 1 2])/1000;
    assert(max(abs((max(p)-min(p))-[.300 .025 .025]))<1e-6);
    V=[]; F=[]; yc=.193*(2*strcmp(side,'left')-1);
    for j=1:3
        bounds=c.pipe_bounds(j,:); verts=p;
        verts(:,1)=bounds(1)+p(:,1)*diff(bounds)/.300;
        verts(:,2)=yc+p(:,2);
        verts(:,3)=p(:,3)-max(p(:,3))+c.rail_top_m+(j==2)*c.step_m;
        F=[F;source.ConnectivityList+size(V,1)]; V=[V;verts];
    end
    stlwrite(triangulation(F,V),filename); return
end
x=[-.9:.005:-.005, -1e-9, 0, .005:.005:1.6]';
active=strcmp(side,c.defect_side) || strcmp(c.defect_side,'both');
h=c.rail_top_m+active*(c.step_m*(x>=0)+c.bend_m*exp(-.5*((x-.25)/.07).^2));
if strcmp(side,'left'), yc=.193; else, yc=-.193; end
n=numel(x); y1=yc-c.rail_width_m/2; y2=yc+c.rail_width_m/2;
V=[x,repmat(y1,n,1),h; x,repmat(y2,n,1),h; ...
   x,repmat(y1,n,1),repmat(c.rail_top_m-.025,n,1); ...
   x,repmat(y2,n,1),repmat(c.rail_top_m-.025,n,1)];
F=zeros((n-1)*8+4,3); j=0;
for i=1:n-1
    quads=[i i+1 n+i+1 n+i; 2*n+i 3*n+i 3*n+i+1 2*n+i+1; ...
        i 2*n+i 2*n+i+1 i+1; n+i n+i+1 3*n+i+1 3*n+i];
    for q=1:4
        a=quads(q,:); F(j+1,:) = a([1 2 3]); F(j+2,:)=a([1 3 4]); j=j+2;
    end
end
F(j+1:end,:)=[1 n+1 3*n+1;1 3*n+1 2*n+1;n 3*n 4*n;n 4*n 2*n];
stlwrite(triangulation(F,V),filename);
end
