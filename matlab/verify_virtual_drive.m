function report=verify_virtual_drive
% End-to-end checks, including a full-cart time-step refinement comparison.
here=fileparts(mfilename('fullpath')); base=fullfile(here,'output');
names={'normal','step_1mm','bend_1mm','combined_1mm','combined_1mm_fine'};
distance_span=zeros(5,1); peak_vertical=zeros(5,1);
for i=1:5
    folder=fullfile(base,names{i}); d=load(fullfile(folder,'trajectory.mat'));
    T=readtable(fullfile(folder,'paired_sensors.csv')); steady=T.time_s>=2;
    assert(height(T)==1001 && all(diff(T.time_s)>0));
    assert(all(isfinite(T{:,3:4}),'all'));
    assert(all(T.distance_left_valid & T.distance_right_valid));
    assert(max(abs(d.p(:,2)))<1e-7 && max(abs(d.p(:,3)))<.02);
    assert(abs(d.p(end,1)-.8125)<1e-6);
    for k=1:1000:numel(d.t)
        R=reshape(d.rot(k,:),3,3);
        assert(norm(R'*R-eye(3),'fro')<1e-4 && det(R)>.9999);
        assert(abs(atan2(R(2,1),R(1,1)))<.001,'Cart must stay aligned to guided rails');
    end
    gaps=T{steady,3:4}; distance_span(i)=max(max(gaps,[],1)-min(gaps,[],1));
    peak_vertical(i)=max(abs(T{steady,{'L_az_filtered_mps2','R_az_filtered_mps2'}}-9.80665),[],'all');
end
assert(peak_vertical(1)<.01,'Flat rail should settle to near-static vertical force');
assert(distance_span(1)<.01,'Flat rail distance must be stable');
assert(all(distance_span(2:4)>.05),'Defects must affect distance');
assert(all(peak_vertical(2:4)>peak_vertical(1)+.005),'Defects must affect IMU');
A=readtable(fullfile(base,names{4},'paired_sensors.csv'));
B=readtable(fullfile(base,names{5},'paired_sensors.csv'));
sel=A.time_s>=2; cols=contains(A.Properties.VariableNames,'_filtered_mps2');
signal=B{sel,cols}; signal=signal-mean(signal,1);
error=max(abs(A{sel,cols}-B{sel,cols}),[],'all');
relative_error=error/max(abs(signal),[],'all');
assert(relative_error<.05,'Full-cart refinement error exceeds 5%%');
report=table(string(names(:)),distance_span,peak_vertical, ...
    'VariableNames',{'run','distance_span_mm','peak_vertical_specific_minus_g_mps2'});
writetable(report,fullfile(base,'virtual_drive_validation.csv'));
fid=fopen(fullfile(base,'virtual_drive_validation.txt'),'w');
fprintf(fid,'Full-cart max filtered IMU refinement difference: %.9g m/s^2\nRelative to fine-run dynamic peak: %.4f%%\nAll numerical checks passed; NOT real-world calibration.\n',error,100*relative_error);
fclose(fid); disp(report); fprintf('Refinement: %.4f%%\n',100*relative_error);
end
