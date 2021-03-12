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
            assert(size(quat,1) == 1)
            assert(size(quat,2) == 4)
            obj.setData(quat);
        end
        
        % Sets the data
        function setData(obj, quat)
            obj.quat = quat;
        end
        
        % Get rotation matrix
        function R = getRotationMatrix(obj)
            R = squeeze(quat2rot(obj.quat));
        end
        
        % Rotates a vector
        function rv = rotateVectors(obj, v)
            rv = k_quat_rotate(obj.quat, v);
        end
        
        % Returns an object representing the inverse quaternion
        function q_inv = inverse(obj)
            q_inv = OrientationQuaternion([obj.quat(1) -obj.quat(2:4)]);
        end
        
        function q = compose(obj, other)
            q = OrientationQuaternion(k_quat_mult(obj.quat, other.quat));
        end
        
        function q = diff(obj, other)
             q = obj.compose(other.inverse);
        end
        
        function [axis, angle] = axisAngle(obj)
            vec_mag = norm(obj.quat(2:4));
            if vec_mag > 0
                angle = 2 * acos(obj.quat(1));
                axis = obj.quat(2:4) / norm(obj.quat(2:4));
            else
                angle = 0;
                axis = [1 0 0];
            end
        end
        
        % Plots this transform as a set of axis
        function plot(obj, length)
            if nargin < 2
                length = 1;
            end
            holdstate = ishold;
            hold on
            % Generating the unit vectors for plotting
            unit_x = [1 0 0]';
            unit_y = [0 1 0]';
            unit_z = [0 0 1]';
            % Rotating the axis
            R = obj.getRotationMatrix();
            trans_x = R * unit_x;
            trans_y = R * unit_y;
            trans_z = R * unit_z;
            % Plotting
            quiver3(0, 0, 0, trans_x(1), trans_x(2), trans_x(3), length, 'r');
            quiver3(0, 0, 0, trans_y(1), trans_y(2), trans_y(3), length, 'g');
            quiver3(0, 0, 0, trans_z(1), trans_z(2), trans_z(3), length, 'b');
            if ~holdstate
              hold off
            end
        end
        
    end
    
    methods(Static)
        
        function q = fromYPR(y, p, r)
            q = OrientationQuaternion(ypr2quat(y,p,r)');
        end
        
        function q = fromRotationMatrix(R)
            q = OrientationQuaternion(rot2quat(R)');
        end
        
    end
    
end