function plot_virtual_drive_comparison
here=fileparts(mfilename('fullpath')); out=fullfile(here,'output');
names={'normal','step_1mm','bend_1mm','combined_1mm'};
fig=figure('Visible','off','Color','w','Position',[40 40 1200 1000]);
tl=tiledlayout(fig,4,2,'TileSpacing','compact');
for i=1:4
    T=readtable(fullfile(out,names{i},'paired_sensors.csv')); sel=T.time_s>=2;
    nexttile; plot(T.time_s(sel),T{sel,{'distance_left_mm','distance_right_mm'}});
    grid on; ylabel('Distance (mm)'); title(strrep(names{i},'_',' '));
    if i==1, legend('CAD left','CAD right'); ylim([3 5]); end
    nexttile; plot(T.time_s(sel),T{sel,{'L_az_filtered_mps2','R_az_filtered_mps2'}}-9.80665);
    grid on; ylabel('Vertical specific - g (m/s^2)'); title('Same-run IMU');
    if i==1, ylim([-.01 .01]); end
end
xlabel(tl,'Time (s); settling and acceleration excluded');
title(tl,'Rigid CAD cart: paired synthetic sensors | uncalibrated');
exportgraphics(fig,fullfile(out,'virtual_drive_comparison.png'),'Resolution',130);
close(fig);
end
