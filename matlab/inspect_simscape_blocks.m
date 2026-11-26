function inspect_simscape_blocks
% Discover the installed release's actual block parameters, never guess them.
here=fileparts(mfilename('fullpath')); out=fullfile(here,'output');
load_system('sm_lib'); load_system('fl_lib'); load_system('nesl_utility');
m='RailTwin_BlockInspect'; if bdIsLoaded(m), close_system(m,0); end
new_system(m);
sources={'sm_lib/Joints/6-DOF Joint','sm_lib/Frames and Transforms/Rigid Transform', ...
    'sm_lib/Frames and Transforms/Transform Sensor', ...
    'sm_lib/Body Elements/Spherical Solid','sm_lib/Body Elements/Brick Solid', ...
    'sm_lib/Body Elements/Extruded Solid','sm_lib/Body Elements/File Solid', ...
    'sm_lib/Forces and Torques/Spatial Contact Force', ...
    'sm_lib/Utilities/Mechanism Configuration', ...
    'sm_lib/Forces and Torques/External Force and Torque', ...
    'sm_lib/Joints/Revolute Joint', ...
    'nesl_utility/Simulink-PS Converter','nesl_utility/PS-Simulink Converter'};
fid=fopen(fullfile(out,'block_parameters.txt'),'w'); cleaner=onCleanup(@()fclose(fid));
for i=1:numel(sources)
    try
        b=add_block(sources{i},[m '/B' num2str(i)]);
        fprintf(fid,'\nSOURCE %s\n',sources{i});
        names=get_param(b,'DialogParameters'); keys=fieldnames(names);
        for j=1:numel(keys)
            value=get_param(b,keys{j});
            fprintf(fid,'%s = %s',keys{j},string(value));
            if isfield(names.(keys{j}),'Enum'), fprintf(fid,' | %s',strjoin(string(names.(keys{j}).Enum),',')); end
            fprintf(fid,'\n');
        end
    catch e
        fprintf(fid,'ERROR %s: %s\n',sources{i},e.message);
    end
end
close_system(m,0); fprintf('Block parameters written.\n');
end
