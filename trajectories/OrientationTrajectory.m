classdef OrientationTrajectory < handle & matlab.mixin.Copyable
    %TRAJECTORY Encapsulates trajectory functionality
    %   Detailed explanation goes here
    
    properties
        % Raw Data
        orientations
        times
        % Properties
        length
    end
    
    methods
        
        % Constructor
        % Initializes the trajectory from a timeseries 
        function obj = OrientationTrajectory(orientations, times)
            obj.setData(orientations, times)
        end
        
        % Sets the data
        function setData(obj, orientations, times)
            % Checks
            assert(size(orientations,1) == size(times,1), 'Must be the same number of orientations as times.');
            % Storing the raw data
            obj.orientations = orientations;
            obj.times = times;
            % Properties
            obj.length = size(times, 1);
        end
        
        % Returns an indexed orientation quaternion
        function q = getOrientationQuaternion(obj, index)
            q = OrientationQuaternion(obj.orientations(index, :));
        end
        
        % Returns the trajectory with inverted rotation matrices
        function inverse_trajectory = inverse(obj)
            inverse_trajectory = OrientationTrajectory(k_quat_inv(obj.orientations), obj.times);
        end
        
        % Compose this trajectory with another
        function transformed_trajectory = compose(obj, trajectory_other)
            q = k_quat_mult(obj.orientations, trajectory_other.orientations());
            transformed_trajectory = OrientationTrajectory(q, obj.times);
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
                v_rotated(i,:) = squeeze(R(i,:,:)) * trajectory.positions(i,:)';
            end
            % Writing the data
            trajectory.setData(v_rotated, trajectory.times());
        end
        
        % Gets a trajectory of rotation matrices
        function R = getRotationMatrixTrajectory(obj)
            % Sending the trajectory data to the generic function
            R = quat2rot(obj.orientations);
        end
        
    end
    
end

