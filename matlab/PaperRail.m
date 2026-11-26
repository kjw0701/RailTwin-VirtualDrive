classdef PaperRail
    % Geometry shared by contact, distance rays and the visible pipe.
    properties (Constant)
        SegmentLengthM = 0.600
        JointGapM = 0.003
        GaugeM = 0.386
        SourcePipeLengthM = 0.300
        SourcePipeWidthM = 0.025
        MiddleSegment = 2
        PaperStepM = 0.002
        PaperSpeedMps = 0.052
    end
    properties (SetAccess=private)
        Bounds
        Heights
        CenterY
        Width
    end
    methods
        function obj=PaperRail(config,sideSign)
            obj.Bounds=config.pipe_bounds;
            obj.Heights=repmat(config.rail_top_m,size(obj.Bounds,1),1);
            isRaised=strcmp(config.defect_side,'both') || ...
                (sideSign>0 && strcmp(config.defect_side,'left')) || ...
                (sideSign<0 && strcmp(config.defect_side,'right'));
            if isRaised
                obj.Heights(obj.MiddleSegment)=config.rail_top_m+config.step_m;
            end
            obj.CenterY=sideSign*obj.GaugeM/2;
            obj.Width=config.rail_width_m;
        end
        function distance=rayDistance(obj,origin,direction)
            distance=NaN;
            for segment=1:size(obj.Bounds,1)
                travel=(obj.Heights(segment)-origin(3))/direction(3);
                hit=origin+travel*direction;
                onSegment=hit(1)>=obj.Bounds(segment,1) && hit(1)<=obj.Bounds(segment,2);
                onTread=abs(hit(2)-obj.CenterY)<=obj.Width/2;
                if travel>=0 && onSegment && onTread && (isnan(distance) || travel<distance)
                    distance=travel;
                end
            end
        end
    end
end
