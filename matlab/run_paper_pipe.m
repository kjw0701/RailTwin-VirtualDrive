function result=run_paper_pipe(match_csv)
% Three nominal 600 mm sections, with 3 mm clear gaps between the pieces.
c=virtual_drive_config;
c.pipe_bounds=[0 .600; .603 1.203; 1.206 1.806];
c.pipe_file='pipe_L300.stl';
c.support_model='Fixed filament supports represented by segment elevations; no exposed metal in joint gaps';
c.speed_mps=.052; c.start_x_m=.28; c.stop_time_s=25;
c.defect_side='both'; c.step_m=.002; c.bend_m=0;
name='paper_pipe_step_2mm'; animate=true;
if nargin>0 && match_csv
    % Median of per-run medians, normal runs 31-33 only (left, right).
    c.tip_lift_m=([6.4395 6.436]-3.5)/1000;
    c.tip_calibration='Effective distance ray origins fitted to normal CSV runs 31-33; physical mount unverified';
    name=[name '_csvfit']; animate=false;
end
result=run_virtual_drive(c,name,animate);
end
