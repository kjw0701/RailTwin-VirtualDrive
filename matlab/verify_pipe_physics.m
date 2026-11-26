function verify_pipe_physics
% Analytical checks independent of field accelerometer recordings.
config=virtual_drive_config;
sensor=ChassisIMU(config,1);
stationary=zeros(3,1);
[reading,linear]=sensor.observe(eye(3),stationary,stationary,stationary);
assert(norm(reading+sensor.GravityWorldMps2')<1e-12 && norm(linear)==0);
reading=sensor.observe(eye(3),sensor.GravityWorldMps2,stationary,stationary);
assert(norm(reading)<1e-12,'Free fall should produce zero specific force');
% Rotation around Z at constant speed produces inward acceleration at the IMU.
omega=[0;0;2];
[~,linear]=sensor.observe(eye(3),stationary,omega,stationary);
expected=-omega(3)^2*[sensor.OffsetBodyM(1:2);0];
assert(norm(linear'-expected)<1e-12);

% One wheel centered on a level tread with a known penetration.
segments=[0 .6 0; .603 1.203 .002; 1.206 1.806 0];
radius=config.wheel_radius_m;
penetration=.0001;
position=[.3;0;radius-penetration];
contact=[radius;config.contact_stiffness_Npm;config.contact_damping_Nspm];
input=[position;stationary;contact;reshape(segments.',[],1)];
force=pipe_contact_kernel(input);
assert(norm(force-[0;0;penetration*config.contact_stiffness_Npm])<1e-9);
input(3)=radius+penetration;
assert(norm(pipe_contact_kernel(input))==0,'Airborne wheel must exert no force');
disp('Pipe contact and IMU checks passed');
end
