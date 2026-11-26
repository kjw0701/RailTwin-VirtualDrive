function result=run_virtual_drive(c,run_name,make_animation)
% One command: build CAD dynamics, simulate, export synchronized observations.
if nargin<1, c=virtual_drive_config; end
if nargin<2, run_name='combined_1mm'; end
if nargin<3, make_animation=true; end
assert(ischar(run_name) && ~isempty(regexp(run_name,'^[A-Za-z0-9_-]+$','once')), ...
    'Use a simple run name with letters, numbers, underscores or hyphens.');
assert(c.step_m>=0 && c.step_m<c.wheel_radius_m/2,'Only small upward steps supported');
assert(c.bend_m>=0 && c.bend_m<=.005,'Prototype vertical bend range: 0 to 5mm');
assert(c.speed_mps>0 && c.max_step_s>0 && c.stop_time_s>1.5);
assert(ismember(c.defect_side,{'left','right','both'}));
assert(c.chassis_mass_kg==3 && c.wheel_mass_kg==.1, ...
    'Mass is fixed in the imported CAD URDF; change and re-import URDF first.');
assert(c.bend_center_m==.25 && c.bend_sigma_m==.07, ...
    'Prototype profile center and width are fixed in the contact and ray kernels.');
assert(abs(round(1/c.output_hz/c.max_step_s)-1/c.output_hz/c.max_step_s)<1e-8, ...
    'Output period must be an integer multiple of the internal sample period.');
model=build_virtual_drive(c);
simout=sim(model);
result=export_virtual_drive(simout,c,run_name);
if make_animation, render_virtual_drive(run_name); end
save_system(model);
fprintf('Completed %s: %s\n',run_name,result.output_folder);
end
