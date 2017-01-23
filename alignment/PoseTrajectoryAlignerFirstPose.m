classdef PoseTrajectoryAlignerFirstPose < PoseTrajectoryAlignerBase
    %POSITIONTRAJECTORYALIGNERYAWONLY Contains functionality relating to a dense mapping system
   
    properties
    end
    
    methods
        
        % Constructor
        function obj = PoseTrajectoryAlignerFirstPose()
            % Calling the superclass constructor
            obj = obj@PoseTrajectoryAlignerBase();
        end

    end
    
    methods(Static)
        
        function T_alignment = calculateAlignmentTransform(aligned_to_trajectory,...
                                                           aligned_from_trajectory,...
                                                           orientation_scaling,...
                                                           sigma_init)
            % Extracting the first poses
            aligned_to_pose = aligned_to_trajectory.getTransformation(1);
            aligned_from_pose = aligned_from_trajectory.getTransformation(1);
            % Calculating the alignment based on this
            T_alignment = aligned_to_pose * aligned_from_pose.inverse();
        end

    end
    
end