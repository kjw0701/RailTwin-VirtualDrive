function run_virtual_drive_examples
% Four paired scenarios plus refined combined run. No CNN training performed.
c=virtual_drive_config;
names={'normal','step_1mm','bend_1mm','combined_1mm'};
defects=[0 0;.001 0;0 .001;.001 .001];
for i=1:4
    c.step_m=defects(i,1); c.bend_m=defects(i,2);
    run_virtual_drive(c,names{i},false);
end
c.max_step_s=c.max_step_s/2;
run_virtual_drive(c,'combined_1mm_fine',false);
verify_virtual_drive;
plot_virtual_drive_comparison;
% Leave the editable model at its normal timestep and combined scenario.
c.max_step_s=c.max_step_s*2; build_virtual_drive(c);
render_virtual_drive('combined_1mm');
end
