classdef OrientationQuaternion < handle & matlab.mixin.Copyable
    %TRAJECTORY Encapsulates trajectory functionality
    %   Detailed explanation goes here
    
    properties
        % Raw data
        quat
    end
    
    methods
        
        % Constructor
        function obj = OrientationQuaternion(quat)
            % If no arguments initialize identity transform
            if nargin == 0
                quat = [1 0 0 0];
            end
            % Setting the data
            obj.setData(quat);
        end
        
        % Sets the data
        function setData(obj, quat)
            obj.quat = quat;
        end
        
        % Initialize from euler angles
        function initializeFromYPR(obj, y, p, r)
            q = ypr2quat(y,p,r)';
            obj.setData(q);
        end
        
        % Get rotation matrix
        function R = getRotationMatrix(obj)
            R = squeeze(quat2rot(obj.quat));
        end
        
        % Rotates a vector
        function rv = rotateVector(obj, v)
            rv = k_quat_rotate(obj.quat, v);
        end
        
    end
    
end