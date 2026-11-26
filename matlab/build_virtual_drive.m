function model=build_virtual_drive(c)
% CAD rigid-body dynamics in Simscape, with analytic wheel/rail normal contact.
if nargin==0, c=virtual_drive_config; end
here=fileparts(mfilename('fullpath')); out=fullfile(here,'output'); cad=fullfile(here,'cad');
addpath(here,cad); load_system('sm_lib'); load_system('nesl_utility');
template=fullfile(out,'RailTwin_CAD_Assembly.slx');
if ~exist(template,'file'), import_cart_simscape; end
model='RailTwin_VirtualDrive';
if bdIsLoaded(model), close_system(model,0); end
load_system(template); save_system('RailTwin_CAD_Assembly',fullfile(out,[model '.slx']));
if bdIsLoaded('RailTwin_CAD_Assembly'), close_system('RailTwin_CAD_Assembly',0); end
load_system(fullfile(out,[model '.slx']));
mw=get_param(model,'ModelWorkspace'); assignin(mw,'drive_config',c);
set_param(model,'Solver','ode15s','MaxStep',num2str(c.max_step_s), ...
    'RelTol','1e-5','AbsTol','1e-7','StopTime',num2str(c.stop_time_s), ...
    'ReturnWorkspaceOutputs','on','SaveTime','on','SimscapeLogType','none');
set_param([model '/MechanismConfiguration'],'GravityVector','[0 0 -9.80665]');
% Ideal rail guide: prescribed X, lateral position, and yaw. Z/roll/pitch dynamic.
% Without a yaw guide, unilateral wheel forces can turn the cart off the rails.
replace_block(model,'SearchDepth',1,'Name','cart_free', ...
    'sm_lib/Joints/Bushing Joint','noprompt');
j=[model '/cart_free'];
set_param(j,'PxMotionActuationMode','InputMotion','PxTorqueActuationMode','ComputedTorque', ...
    'PyMotionActuationMode','InputMotion','PyTorqueActuationMode','ComputedTorque', ...
    'RzMotionActuationMode','InputMotion','RzTorqueActuationMode','ComputedTorque');
world=ports([model '/World'],'RConn',1); chassis=ports([model '/chassis'],'LConn',1);
% Smooth start after one second of settling; position, velocity, acceleration.
clock=add_block('simulink/Sources/Clock',[model '/Time']);
motion=add_block('simulink/User-Defined Functions/MATLAB Function',[model '/Travel']);
chart=find(sfroot,'-isa','Stateflow.EMChart','Path',[model '/Travel']);
chart.Script=sprintf(['function [x,v,a]=travel(t)\n' ...
'u=t-1; T=.5; speed=%.17g; x0=%.17g;\n' ...
'if u<=0, x=x0;v=0;a=0; elseif u<T, s=u/T; x=x0+speed*T*(s^3-.5*s^4); v=speed*(3*s^2-2*s^3); a=speed/T*(6*s-6*s^2); else, x=x0+speed*(u-.5*T);v=speed;a=0;end\nend'],c.speed_mps,c.start_x_m);
wire(ports(clock,'Outport',1),ports(motion,'Inport',1));
conv=add_block('nesl_utility/Simulink-PS Converter',[model '/TravelInput']);
set_param(conv,'Unit','m','FilteringAndDerivatives','provide','UdotUserProvided','2');
for k=1:3, wire(ports(motion,'Outport',k),ports(conv,'Inport',k)); end
wire(ports(conv,'RConn',1),ports(j,'LConn',2));
zero=add_block('simulink/Sources/Constant',[model '/LateralZero'],'Value','0');
cy=add_block('nesl_utility/Simulink-PS Converter',[model '/LateralInput']);
set_param(cy,'Unit','m','FilteringAndDerivatives','zero');
wire(ports(zero,'Outport',1),ports(cy,'Inport',1)); wire(ports(cy,'RConn',1),ports(j,'LConn',3));
cz=add_block('nesl_utility/Simulink-PS Converter',[model '/YawGuide']);
set_param(cz,'Unit','rad','FilteringAndDerivatives','zero');
yawzero=add_block('simulink/Sources/Constant',[model '/YawZero'],'Value','0');
wire(ports(yawzero,'Outport',1),ports(cz,'Inport',1)); wire(ports(cz,'RConn',1),ports(j,'LConn',4));
manifest=jsondecode(fileread(fullfile(cad,'manifest.json')));
for k=1:4
    name=manifest.parts(k).name; frame=ports([model '/' name],'LConn',1);
    % Wheel spin is prescribed rolling for display. Normal contact is radial.
    joint=[model '/' name '_axle'];
    set_param(joint,'MotionActuationMode','InputMotion','TorqueActuationMode','ComputedTorque');
    wheelInput=add_block('nesl_utility/Simulink-PS Converter',[model '/' name '_SpinInput']);
    set_param(wheelInput,'Unit','rad','FilteringAndDerivatives','provide','UdotUserProvided','2');
    for r=1:3
        gain=add_block('simulink/Math Operations/Gain',[model '/' name '_SpinGain' num2str(r)],'Gain',num2str(1/c.wheel_radius_m));
        wire(ports(motion,'Outport',r),ports(gain,'Inport',1)); wire(ports(gain,'Outport',1),ports(wheelInput,'Inport',r));
    end
    wire(ports(wheelInput,'RConn',1),ports(joint,'LConn',2));
    ps=sense(frame,[name '_Position'],'SenseXYZ','m');
    vs=sense(frame,[name '_Velocity'],'SenseXYZDot','m/s');
    mux=add_block('simulink/Signal Routing/Mux',[model '/' name '_ContactInput'],'Inputs','3');
    wire(ps,ports(mux,'Inport',1)); wire(vs,ports(mux,'Inport',2));
    left=manifest.parts(k).center_m(2)>0;
    active=strcmp(c.defect_side,'both')||(left&&strcmp(c.defect_side,'left'))||(~left&&strcmp(c.defect_side,'right'));
    cp=[active*c.step_m active*c.bend_m c.wheel_radius_m c.contact_stiffness_Npm c.contact_damping_Nspm c.rail_top_m];
    cb=add_block('simulink/Sources/Constant',[model '/' name '_ContactParameters'],'Value',mat2str(cp,17));
    wire(ports(cb,'Outport',1),ports(mux,'Inport',3));
    kernel=add_block('simulink/User-Defined Functions/MATLAB Function',[model '/' name '_ContactLaw']);
    chart=find(sfroot,'-isa','Stateflow.EMChart','Path',[model '/' name '_ContactLaw']);
    law='rail_contact_kernel.m';
    if isfield(c,'pipe_bounds')
        rail=PaperRail(c,sign(manifest.parts(k).center_m(2)));
        % The compiled block receives position, velocity, then these constants.
        segments=[rail.Bounds rail.Heights];
        contact=[c.wheel_radius_m c.contact_stiffness_Npm c.contact_damping_Nspm];
        set_param(cb,'Value',mat2str([contact reshape(segments.',1,[])],17));
        law='pipe_contact_kernel.m';
    end
    chart.Script=fileread(fullfile(here,law));
    wire(ports(mux,'Outport',1),ports(kernel,'Inport',1));
    fc=add_block('nesl_utility/Simulink-PS Converter',[model '/' name '_ForceInput']);
    set_param(fc,'Unit','N','FilteringAndDerivatives','zero');
    force=add_block('sm_lib/Forces and Torques/External Force and Torque',[model '/' name '_ContactForce']);
    set_param(force,'EnableForce','on','ForceResolutionFrame','World');
    wire(ports(kernel,'Outport',1),ports(fc,'Inport',1));
    wire(ports(fc,'RConn',1),ports(force,'LConn',1)); wire(frame,ports(force,'RConn',1));
end
signals={'body_p','SenseXYZ','m';'body_a','SenseXYZDDot','m/s^2'; ...
    'body_w','SenseOmega','rad/s';'body_alpha','SenseAlpha','rad/s^2';'body_R','SenseR','1'};
for i=1:size(signals,1)
    port=sense(chassis,signals{i,1},signals{i,2},signals{i,3});
    sink=add_block('simulink/Sinks/To Workspace',[model '/' signals{i,1} '_Log'], ...
        'VariableName',signals{i,1},'SaveFormat','Timeseries','MaxDataPoints','inf');
    wire(port,ports(sink,'Inport',1));
end
for side={'left','right'}
    filename=make_rail_visual(c,side{1});
    b=add_block('sm_lib/Body Elements/File Solid',[model '/Rail_' side{1}]);
    set_param(b,'ExtGeomFileName',filename,'UnitType','Custom','ExtGeomFileUnits','m', ...
        'InertiaType','PointMass','Mass','0','GraphicDiffuseColor','[.35 .42 .46]');
    wire(world,ports(b,'RConn',1));
end
annotation=Simulink.Annotation(model, sprintf(['CAD rigid-body virtual driving prototype\n' ...
    'Normal contact only; prescribed travel; mass/contact/sensor parameters uncalibrated\n' ...
    'Distance + left/right XYZ acceleration share one computed trajectory']));
annotation.Position=[20 20];
Simulink.BlockDiagram.arrangeSystem(model);
save_system(model,fullfile(out,[model '.slx']));
fprintf('Built %s\n',model);

    function wire(a,b)
        add_line(model,a,b,'autorouting','on');
    end
    function result=sense(frame,name,flag,unit)
        sensor=add_block('sm_lib/Frames and Transforms/Transform Sensor',[model '/' name '_Sensor']);
        set_param(sensor,flag,'on','MeasurementFrame','World');
        wire(world,ports(sensor,'LConn',1)); wire(frame,ports(sensor,'RConn',1));
        cv=add_block('nesl_utility/PS-Simulink Converter',[model '/' name '_Output']);
        set_param(cv,'Unit',unit);
        wire(ports(sensor,'RConn',2),ports(cv,'LConn',1));
        result=ports(cv,'Outport',1);
    end
end

function h=ports(block,kind,index)
p=get_param(block,'PortHandles'); h=p.(kind)(index);
end
