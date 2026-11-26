function preview_cart_cad
% Preview the actual repository CAD-derived mesh, not a substitute box model.
here=fileparts(mfilename('fullpath')); cad=fullfile(here,'cad');
out=fullfile(here,'output'); if ~exist(out,'dir'), mkdir(out); end
manifest=jsondecode(fileread(fullfile(cad,'manifest.json')));
fig=figure('Visible','off','Color','w','Position',[50 50 1100 720]);
ax=axes(fig); hold(ax,'on');
tr=stlread(fullfile(cad,'chassis.stl'));
patch(ax,'Faces',tr.ConnectivityList,'Vertices',tr.Points, ...
    'FaceColor',[.6 .65 .7],'EdgeColor','none');
for k=1:numel(manifest.parts)
    part=manifest.parts(k); tr=stlread(fullfile(cad,[part.name '.stl']));
    patch(ax,'Faces',tr.ConnectivityList,'Vertices',tr.Points+part.center_m(:)', ...
        'FaceColor',[.15 .18 .22],'EdgeColor','none');
end
for yy=[-.193 .193]
    [xx,y]=meshgrid([-.5 .6],yy+[-.0125 .0125]);
    surf(ax,xx,y,ones(2)*.0225,'FaceColor',[.35 .4 .45],'EdgeColor','none','FaceAlpha',.7);
end
axis(ax,'equal'); grid(ax,'on'); view(ax,35,22);
xlabel(ax,'Forward X (m)'); ylabel(ax,'CAD lateral Y (m)'); zlabel(ax,'Up Z (m)');
camlight(ax,'headlight'); lighting(ax,'gouraud');
title(ax,{'RailTwin repository CAD: chassis + 4 separate wheels', ...
    'Geometry import preview | physical mass/contact calibration pending'});
exportgraphics(fig,fullfile(out,'cart_cad_preview.png'),'Resolution',150);
savefig(fig,fullfile(out,'cart_cad_preview.fig')); close(fig);
fprintf('CAD preview saved. %d triangles.\n',manifest.total_triangles);
end
