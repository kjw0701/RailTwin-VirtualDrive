function result=export_virtual_drive(simout,c,run_name)
% Synchronized observations from ONE simulated chassis trajectory.
here=fileparts(mfilename('fullpath')); out=fullfile(here,'output',run_name);
if ~exist(out,'dir'), mkdir(out); end
dt=c.max_step_s; t=(0:dt:c.stop_time_s)'; n=numel(t);
p=sample(simout.body_p,t,3); a=sample(simout.body_a,t,3);
w=sample(simout.body_w,t,3); alpha=sample(simout.body_alpha,t,3);
rot=sample(simout.body_R,t,9);
dist=zeros(n,2); specific=zeros(n,6); linear=zeros(n,6); gyro=zeros(n,6);
tipZ=repmat(c.distance_tip_z_m,1,2);
if isfield(c,'tip_lift_m'), tipZ=tipZ+c.tip_lift_m; end
sideSigns=[1 -1];
for side=1:2
    sensors(side)=ChassisIMU(c,sideSigns(side));
    if isfield(c,'pipe_bounds')
        rails(side)=PaperRail(c,sideSigns(side));
    end
end
for k=1:n
    R=reshape(rot(k,:),3,3);
    for s=1:2
        signy=sideSigns(s);
        cols=(s-1)*3+(1:3);
        [specific(k,cols),linear(k,cols),gyro(k,cols)]=sensors(s).observe( ...
            R,a(k,:)',w(k,:)',alpha(k,:)');
        tip=p(k,:)'+R*[c.sensor_x_m;signy*c.sensor_y_m;tipZ(s)];
        active=strcmp(c.defect_side,'both')||(s==1&&strcmp(c.defect_side,'left'))||(s==2&&strcmp(c.defect_side,'right'));
        if isfield(c,'pipe_bounds')
            dist(k,s)=rails(s).rayDistance(tip,-R(:,3));
        else
            dist(k,s)=rail_ray_distance(tip,-R(:,3),active*c.step_m,active*c.bend_m,c.rail_top_m);
        end
    end
end
assert(all(isfinite(p),'all')&&all(isfinite(specific),'all'),'Nonfinite simulation');
% Example analog-like sensor response: four cascaded 20Hz RC sections.
% Each section is initialized at its first input to avoid startup filter spikes.
filtered=specific;
coef=exp(-2*pi*c.sensor_lowpass_hz*dt);
for stage=1:4
    for ch=1:6
        filtered(:,ch)=filter(1-coef,[1 -coef],filtered(:,ch),coef*filtered(1,ch));
    end
end
idx=1:round(1/c.output_hz/dt):n;
valid=dist>=c.distance_min_m & dist<=c.distance_max_m;
T=table(t(idx),p(idx,1),dist(idx,1)*1000,dist(idx,2)*1000,valid(idx,1),valid(idx,2), ...
    'VariableNames',{'time_s','chassis_x_m','distance_left_mm','distance_right_mm','distance_left_valid','distance_right_valid'});
names={'L_ax','L_ay','L_az','R_ax','R_ay','R_az'};
T.steady_drive=t(idx)>=2; % Exclude settling and commanded acceleration from defect training.
for ch=1:6
    T.([names{ch} '_specific_mps2'])=specific(idx,ch);
    T.([names{ch} '_filtered_mps2'])=filtered(idx,ch);
    T.([names{ch} '_linear_mps2'])=linear(idx,ch);
end
writetable(T,fullfile(out,'paired_sensors.csv'));
meta=c; meta.run_name=run_name; meta.synthetic=true; meta.validated_against_real=false;
meta.filter='four cascaded RC sections, each 20Hz; illustrative, not MPU6050 calibration';
meta.acceleration_channels='left XYZ + right XYZ, sensor frame, specific force includes gravity';
meta.motion='Prescribed travel, lateral position and yaw guide; computed vertical/roll/pitch response';
meta.training_note='Exclude steady_drive=false. Run-level defect parameters are not per-window labels. Split by rail/run before windowing.';
fid=fopen(fullfile(out,'metadata.json'),'w'); fprintf(fid,'%s',jsonencode(meta,PrettyPrint=true)); fclose(fid);
save(fullfile(out,'trajectory.mat'),'t','p','rot','a','w','alpha','dist','specific','gyro','c','-v7');
fig=figure('Visible','off','Color','w','Position',[50 50 1100 750]);
tl=tiledlayout(fig,3,1);
nexttile; plot(t(idx),dist(idx,:)*1000); ylabel('Distance (mm)'); legend('CAD left','CAD right'); grid on;
nexttile; plot(t(idx),filtered(idx,[3 6])-9.80665); ylabel('Vertical specific force - g'); grid on;
nexttile; plot(t(idx),filtered(idx,[1 2 4 5])); ylabel('Horizontal specific force'); xlabel('Time (s)'); grid on;
title(tl,[strrep(run_name,'_',' ') ' | synthetic, uncalibrated']);
exportgraphics(fig,fullfile(out,'paired_signals.png'),'Resolution',140); close(fig);
result=struct('output_folder',out,'samples',height(T),'valid_distance_fraction',mean(valid(idx,:),'all'));
disp(result);
end

function x=sample(ts,t,width)
tt=ts.Time(:); d=ts.Data;
% Simulink timeseries vectors are time-first; matrices may be time-last.
if size(d,1)==numel(tt)
    d=reshape(d,numel(tt),width);
else
    order=[ndims(d),1:ndims(d)-1]; d=reshape(permute(d,order),numel(tt),width);
end
[tt,keep]=unique(tt,'last'); d=d(keep,:);
x=interp1(tt,d,t,'linear');
end
