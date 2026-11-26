function result=run_paper_pipe(match_csv)
% Three nominal 600 mm sections, with 3 mm clear gaps between the pieces.
c=virtual_drive_config;
segmentStarts=(0:2)'*(PaperRail.SegmentLengthM+PaperRail.JointGapM);
c.pipe_bounds=[segmentStarts segmentStarts+PaperRail.SegmentLengthM];
c.pipe_file='pipe_L300.stl';
c.support_model='Fixed filament supports represented by segment elevations; no exposed metal in joint gaps';
c.speed_mps=PaperRail.PaperSpeedMps;
c.start_x_m=.28; % Rear axle starts 10 mm inside the track.
c.stop_time_s=25; % Finish with the front axle still on the final pipe.
c.defect_side='both';
c.step_m=PaperRail.PaperStepM;
c.bend_m=0;
name='paper_pipe_step_2mm'; animate=true;
if nargin>0 && match_csv
    % Median of per-run medians, normal runs 31-33 only (left, right).
    observedBaselineMm=[6.4395 6.436];
    c.tip_lift_m=observedBaselineMm/1000-c.nominal_settled_gap_m;
    c.tip_calibration='Effective distance ray origins fitted to normal CSV runs 31-33; physical mount unverified';
    name=[name '_csvfit']; animate=false;
end
result=run_virtual_drive(c,name,animate);
end
