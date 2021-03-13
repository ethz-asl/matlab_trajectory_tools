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
            assert(size(quat,1) == 1, 'Must be size 1x4')
            assert(size(quat,2) == 4, 'Must be size 1x4')
            obj.setData(quat);
        end
        
        % Sets the data
        function setData(obj, quat)
            obj.quat = quat;
        end
        
        % NOTE(alexmillane): Deprecated this to use the static functions.
        %                    Will leave this in incase we need it later.
%         % Initialize from euler angles
%         function initializeFromYPR(obj, y, p, r)
%             q = ypr2quat(y,p,r)';
%             obj.setData(q);
%         end
                
        % Operator times
        function q = mtimes(obj_1, obj_2)
            q = OrientationQuaternion(k_quat_mult(obj_1.quat, obj_2.quat));
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
        
        function [angle, axis] = toAngleAxis(obj)
            angle = 2 * acos(obj.quat(1));
            den = sqrt(1 - obj.quat(1) * obj.quat(1));
            axis = [obj.quat(2) obj.quat(3) obj.quat(4)] ./ den;
        end
        
        % Plots this transform as a set of axis
        function plot(obj, length, varargin)
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
            quiver3(0, 0, 0, trans_x(1), trans_x(2), trans_x(3), length, 'r', varargin{:});
            quiver3(0, 0, 0, trans_y(1), trans_y(2), trans_y(3), length, 'g', varargin{:});
            quiver3(0, 0, 0, trans_z(1), trans_z(2), trans_z(3), length, 'b', varargin{:});
            if ~holdstate
              hold off
            end
        end
        
    end
    
    methods(Static)
        
        function orientation_quaternion = initializeFromYPR(y, p, r)
            q = ypr2quat(y,p,r);
            orientation_quaternion = OrientationQuaternion(q');
        end
        
        function orientation_quaternion = fromRot(R)
            q = rot2quat(R);
            orientation_quaternion = OrientationQuaternion(q');
        end
                
    end
    
end