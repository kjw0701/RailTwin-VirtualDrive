classdef ChassisIMU
    % Ideal three-axis accelerometer and gyro, rigidly attached to the chassis.
    properties (Constant)
        GravityWorldMps2 = [0;0;-9.80665]
    end
    properties (SetAccess=private)
        OffsetBodyM
    end
    methods
        function obj=ChassisIMU(config,sideSign)
            obj.OffsetBodyM=[config.sensor_x_m; sideSign*config.sensor_y_m; config.imu_z_m];
        end
        function [specific,linear,gyro]=observe(obj,rotation,originAccel,omega,alpha)
            arm=rotation*obj.OffsetBodyM;
            sensorAccel=originAccel+cross(alpha,arm)+cross(omega,cross(omega,arm));
            specific=(rotation'*(sensorAccel-obj.GravityWorldMps2))';
            linear=(rotation'*sensorAccel)';
            gyro=(rotation'*omega)';
        end
    end
end
