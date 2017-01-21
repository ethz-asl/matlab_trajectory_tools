classdef OrientationTrajectory3D < handle & matlab.mixin.Copyable
    %TRAJECTORY Encapsulates trajectory functionality
    %   Detailed explanation goes here
    
    properties
        data
        length
    end
    
    methods
        
        % Constructor
        % Initializes the trajectory from a timeseries 
        function obj = OrientationTrajectory3D(data)
            obj.data = data;
            obj.length = size(data.Time, 1);
        end
        
        
        % Transforms a trajectory of vectors
        function rotateTrajectory(obj, trajectory)
            % Checking the lengths
            assert(obj.length == trajectory.length());
            % Getting the orientations as rotation matrices
            R = obj.getRotationMatrixTrajectory();
            % Looping over and rotating vectory
            % TODO(alexmillane): Make this matrix based for speed.
            v_rotated = zeros(trajectory.length(), 3);
            for i = 1:obj.length
                v_rotated(i,:) = squeeze(R(i,:,:)) * trajectory.data().Data(i,:)';
            end
            % Making a timeseries
            v_rotated_timeseries = timeseries(v_rotated, trajectory.data().Time());
            % Writing the data
            trajectory.setData(v_rotated_timeseries);
        end
        
        % Gets a trajectory of rotation matrices
        function R = getRotationMatrixTrajectory(obj)
            % Sending the trajectory data to the generic function
            R = obj.quat2rot(obj.data.Data);
        end
        
    end
    
    methods(Static)
        
        % Converts an array of quaternions to an array of rotation matrices
        function R = quat2rot(Q)
            % Credits to: Mike Bosse
            Q = permute(Q, [2 3 1]);
            R = [
               Q(1,1,:).^2+Q(2,1,:).^2-Q(3,1,:).^2-Q(4,1,:).^2 2.0.*(Q(2,1,:).*Q(3,1,:)-Q(1,1,:).*Q(4,1,:))   2.0.*(Q(2,1,:).*Q(4,1,:)+Q(1,1,:).*Q(3,1,:))
               2.0.*(Q(2,1,:).*Q(3,1,:)+Q(1,1,:).*Q(4,1,:))   Q(1,1,:).^2-Q(2,1,:).^2+Q(3,1,:).^2-Q(4,1,:).^2 2.0.*(Q(3,1,:).*Q(4,1,:)-Q(1,1,:).*Q(2,1,:))
               2.0.*(Q(2,1,:).*Q(4,1,:)-Q(1,1,:).*Q(3,1,:))   2.0.*(Q(3,1,:).*Q(4,1,:)+Q(1,1,:).*Q(2,1,:))   Q(1,1,:).^2-Q(2,1,:).^2-Q(3,1,:).^2+Q(4,1,:).^2];
           R = permute(R, [3,1,2]);
        end
                        
    end
    
end

