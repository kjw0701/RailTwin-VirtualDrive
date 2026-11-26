function model=import_cart_simscape
% Import CAD-derived geometry plus explicit wheel joints into Simscape.
% This builds the assembly only; contact and excitation are added by the runner.
assert(~isempty(which('smimport')), ...
    'Install Simulink, Simscape, and Simscape Multibody, then restart MATLAB.');
here=fileparts(mfilename('fullpath')); cad=fullfile(here,'cad');
out=fullfile(here,'output'); if ~exist(out,'dir'), mkdir(out); end
addpath(cad);
old=pwd; restore=onCleanup(@()cd(old)); cd(out);
model='RailTwin_CAD_Assembly';
if bdIsLoaded(model), close_system(model,0); end
smimport(fullfile(cad,'RailTwin_CAD.urdf'),'ModelName',model);
set_param(model,'StopTime','0','Solver','ode15s','MaxStep','0.0001');
save_system(model,fullfile(out,[model '.slx']));
fprintf('Imported CAD assembly: %s\n',model);
end
