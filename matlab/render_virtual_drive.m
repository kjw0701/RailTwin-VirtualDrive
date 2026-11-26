function render_virtual_drive(run_name)
% Render CAD against the computed physical trajectory; no invented motion.
here=fileparts(mfilename('fullpath')); folder=fullfile(here,'output',run_name);
d=load(fullfile(folder,'trajectory.mat')); c=d.c;
manifest=jsondecode(fileread(fullfile(here,'cad','manifest.json')));
fig=figure('Visible','off','Color','w','Position',[30 30 1100 850]);
ax=axes(fig,'Position',[.08 .43 .88 .50]); hold(ax,'on');
T=readtable(fullfile(folder,'paired_sensors.csv'));
da=axes(fig,'Position',[.08 .24 .88 .12]);
plot(da,T.time_s,[T.distance_left_mm T.distance_right_mm]);
ylabel(da,'Distance (mm)'); grid(da,'on'); xlim(da,[0 c.stop_time_s]);
legend(da,{'Left','Right'},'Location','eastoutside');
dc=xline(da,0,'k--','HandleVisibility','off');
aa=axes(fig,'Position',[.08 .07 .88 .12]);
plot(aa,T.time_s,[T.L_az_filtered_mps2 T.R_az_filtered_mps2]-9.80665);
ylabel(aa,'Vertical force - g'); xlabel(aa,'Time (s)'); grid(aa,'on'); xlim(aa,[0 c.stop_time_s]);
steady=T.time_s>=2;
peak=max(abs([T.L_az_filtered_mps2(steady);T.R_az_filtered_mps2(steady)]-9.80665));
if ~isempty(peak), ylim(aa,[-1 1]*max(.05,peak*1.2)); end
ac=xline(aa,0,'k--');
tr=stlread(fullfile(here,'cad','chassis.stl')); bodyV=tr.Points;
body=patch(ax,'Faces',tr.ConnectivityList,'Vertices',bodyV,'FaceColor',[.6 .65 .7],'EdgeColor','none');
wheel=cell(4,1); wheelV=cell(4,1);
for j=1:4
    tr=stlread(fullfile(here,'cad',[manifest.parts(j).name '.stl'])); wheelV{j}=tr.Points;
    wheel{j}=patch(ax,'Faces',tr.ConnectivityList,'Vertices',tr.Points,'FaceColor',[.13 .16 .19],'EdgeColor','none');
end
for side={'left','right'}
    tr=stlread(make_rail_visual(c,side{1}));
    patch(ax,'Faces',tr.ConnectivityList,'Vertices',tr.Points,'FaceColor',[.35 .42 .46],'EdgeColor','none');
end
axis(ax,'equal'); grid(ax,'on'); view(ax,32,22);
xlim(ax,[-.9 1.3]); ylim(ax,[-.35 .35]); zlim(ax,[-.02 .3]);
if isfield(c,'pipe_bounds'), xlim(ax,[-.05 c.pipe_bounds(end)+.05]); end
xlabel(ax,'Forward X (m)'); ylabel(ax,'Lateral Y (m)'); zlabel(ax,'Up Z (m)');
camlight(ax,'headlight'); lighting(ax,'gouraud');
indices=round(linspace(1,numel(d.t),60)); gif=fullfile(folder,'virtual_drive.gif');
for k=1:numel(indices)
    n=indices(k); R=reshape(d.rot(n,:),3,3); pos=d.p(n,:);
    dc.Value=d.t(n); ac.Value=d.t(n);
    set(body,'Vertices',bodyV*R'+pos);
    angle=(d.p(n,1)-d.p(1,1))/c.wheel_radius_m;
    Q=[cos(angle) 0 sin(angle);0 1 0;-sin(angle) 0 cos(angle)];
    for j=1:4
        local=wheelV{j}*Q'+manifest.parts(j).center_m(:)';
        set(wheel{j},'Vertices',local*R'+pos);
    end
    title(ax,{sprintf('RailTwin CAD virtual drive | t = %.2f s',d.t(n)), ...
        sprintf('Step %.1f mm + vertical bend %.1f mm | uncalibrated prototype',1000*c.step_m,1000*c.bend_m)});
    drawnow;
    png=fullfile(folder,'drive_frame.png'); exportgraphics(fig,png,'Resolution',100);
    [im,map]=rgb2ind(imread(png),256);
    if k==1, imwrite(im,map,gif,'gif','LoopCount',inf,'DelayTime',.1);
    else, imwrite(im,map,gif,'gif','WriteMode','append','DelayTime',.1); end
end
close(fig); fprintf('Saved %s\n',gif);
end
