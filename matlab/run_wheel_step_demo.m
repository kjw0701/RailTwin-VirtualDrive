function run_wheel_step_demo
% Single-wheel contact feasibility demo; NOT calibrated training data.
% Run: matlab -batch "addpath('simulation/matlab'); run_wheel_step_demo"
out = fullfile(fileparts(mfilename('fullpath')), 'output');
if ~exist(out,'dir'), mkdir(out); end
p = struct('radius_m',0.0235,'mass_kg',1,'speed_mps',0.15, ...
    'stiffness_Npm',1e5,'damping_Nspm',100,'g',9.80665, ...
    'x_start_m',-0.06,'duration_s',1,'output_hz',100,'lowpass_hz',20);
% Radius from drawing, speed from design; all other dynamics uncalibrated.
heights = [0 .0005 .001 .002];
dt = 1e-4;
rows = zeros(4,5); runs = cell(4,1);
for j=1:4
    h=heights(j);
    a=simulate(p,h,dt); b=simulate(p,h,dt/2);
    err=max(abs(a.filtered-b.filtered(1:2:end)));
    scale=max(max(abs(b.filtered)),0.01);
    rows(j,:)=[h*1000,max(abs(a.filtered)),sqrt(mean(a.filtered.^2)),err,err/scale];
    assert(all(isfinite(a.accel)),'Nonfinite dynamics');
    assert(err/scale < .05,'Time-step convergence failed');
    if h==0, assert(max(abs(a.accel))<1e-6,'Flat rail generated vibration'); end
    % Uniform 100 Hz export following a defined illustrative low-pass.
    idx=1:round(1/p.output_hz/dt):numel(a.t);
    T=table(a.t(idx),a.x(idx),a.z(idx),a.gap(idx)*1000, ...
        a.filtered(idx),a.filtered(idx)+p.g, ...
        'VariableNames',{'time_s','position_m','hub_height_m', ...
        'vertical_gap_proxy_mm','vertical_linear_accel_filtered_mps2', ...
        'vertical_specific_force_filtered_mps2'});
    writetable(T,fullfile(out,sprintf('step_%0.1fmm.csv',h*1000)));
    a.height=h; runs{j}=a;
end
summary=array2table(rows,'VariableNames',{'step_mm','filtered_peak_mps2', ...
    'filtered_rms_mps2','dt_halving_max_error_mps2','relative_error'});
writetable(summary,fullfile(out,'convergence.csv')); disp(summary);
fid=fopen(fullfile(out,'parameters.json'),'w'); fprintf(fid,'%s',jsonencode(p,PrettyPrint=true)); fclose(fid);
save(fullfile(out,'wheel_step_runs.mat'),'runs','p','summary');
fig=figure('Visible','off','Color','w','Position',[50 50 1100 760]);
tl=tiledlayout(fig,3,1,'TileSpacing','compact');
ax1=nexttile(tl); hold(ax1,'on'); ax2=nexttile(tl); hold(ax2,'on'); ax3=nexttile(tl); hold(ax3,'on');
for j=1:4
    a=runs{j}; label=sprintf('%.1f mm',a.height*1000);
    plot(ax1,a.t,1000*(a.z-a.z(1)),'DisplayName',label);
    plot(ax2,a.t,a.accel,'DisplayName',label);
    plot(ax3,a.t,a.filtered,'DisplayName',label);
end
ylabel(ax1,'Hub rise (mm)'); ylabel(ax2,'Raw a_z (m/s^2)');
ylabel(ax3,'Filtered a_z (m/s^2)'); xlabel(ax3,'Time (s)');
legend(ax1,'Location','northwest');
title(tl,'Single-wheel step test | illustrative parameters, not validated');
for ax=[ax1 ax2 ax3], grid(ax,'on'); xlim(ax,[.28 .75]); end
exportgraphics(fig,fullfile(out,'comparison.png'),'Resolution',150); close(fig);
animate(runs{4},p,out);
fprintf('Outputs: %s\n',out);
end

function a=simulate(p,h,dt)
t=(0:dt:p.duration_s)';
opts=odeset('RelTol',1e-8,'AbsTol',1e-10,'MaxStep',dt);
z0=p.radius_m-p.mass_kg*p.g/p.stiffness_Npm;
[~,state]=ode15s(@rhs,t,[z0;0],opts);
x=p.x_start_m+p.speed_mps*t;
acc=zeros(size(t));
for n=1:numel(t), d=rhs(t(n),state(n,:)'); acc(n)=d(2); end
% First-order illustrative measurement filter, not an MPU6050 model.
alpha=exp(-2*pi*p.lowpass_hz*dt);
filtered=filter(1-alpha,[1 -alpha],acc);
a=struct('t',t,'x',x,'z',state(:,1),'accel',acc,'filtered',filtered, ...
    'gap',state(:,1)-h*(x>=0));
    function dy=rhs(time,y)
        xx=p.x_start_m+p.speed_mps*time;
        if h==0
            normal=[0;1]; distance=y(1);
        else
            % Closest boundary of a raised half-plane and lower half-plane.
            % Center remains above the small step; corner contact is explicit.
            lower=[xx-min(xx,0);y(1)];
            upper=[xx-max(xx,0);y(1)-h];
            if norm(lower)<norm(upper), r=lower; else, r=upper; end
            distance=norm(r); normal=r/max(distance,eps);
        end
        overlap=p.radius_m-distance;
        force=0;
        if overlap>0
            vn=normal(1)*p.speed_mps+normal(2)*y(2);
            force=max(0,p.stiffness_Npm*overlap-p.damping_Nspm*vn);
        end
        dy=[y(2);force*normal(2)/p.mass_kg-p.g];
    end
end

function animate(a,p,out)
fig=figure('Visible','off','Color','w','Position',[50 50 900 600]);
ang=linspace(0,2*pi,160);
frames=round(linspace(find(a.t>=.28,1),find(a.t>=.65,1),55));
gif=fullfile(out,'wheel_step_2mm.gif');
for k=1:numel(frames)
    n=frames(k); clf(fig);
    subplot(2,1,1); hold on;
    plot([-50 0 0 70],[0 0 2 2],'k','LineWidth',3);
    plot(1000*(a.x(n)+p.radius_m*cos(ang)),1000*(a.z(n)+p.radius_m*sin(ang)),'b','LineWidth',2);
    plot(a.x(n)*1000,a.z(n)*1000,'ro','MarkerFaceColor','r');
    axis equal; xlim([-50 70]); ylim([-3 55]); grid on;
    ylabel('Height (mm)'); xlabel('Position (mm)');
    title('2 mm step | prescribed forward speed | uncalibrated single-wheel model');
    subplot(2,1,2); plot(a.t,a.filtered,'b'); hold on;
    plot(a.t(n),a.filtered(n),'ro','MarkerFaceColor','r');
    xlim([.28 .65]); ylim([min(a.filtered)-.1 max(a.filtered)+.1]); grid on;
    xlabel('Time (s)'); ylabel('Filtered vertical acceleration (m/s^2)');
    drawnow;
    tmp=fullfile(out,'animation_frame.png'); exportgraphics(fig,tmp,'Resolution',100);
    [indexed,map]=rgb2ind(imread(tmp),256);
    if k==1, imwrite(indexed,map,gif,'gif','LoopCount',inf,'DelayTime',.06);
    else, imwrite(indexed,map,gif,'gif','WriteMode','append','DelayTime',.06); end
end
close(fig);
end
