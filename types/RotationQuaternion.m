classdef RotationQuaternion < handle & matlab.mixin.Copyable
    %TRAJECTORY Encapsulates trajectory functionality
    %   Detailed explanation goes here
    
    properties
        % Raw data
        quat
    end
    
    methods
        
        % Constructor
        function obj = RotationQuaternion(quat)
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
            % TODO(alexmillane): Do this is one step for more efficiency.
            R = obj.yprToRotationMatrix(y, p, r);
            q = obj.rot2quat(R);
            obj.setData(q);
        end
        
        % Get rotation matrix
        function R = getRotationMatrix(obj)
            R = obj.quat2rot(obj.quat);
        end
        
    end
    
    methods(Static)

        % Converts an array of quaternions to an array of rotation matrices
        function R = quat2rot(q)
            % Credits MBosse
            R = [   q(1).^2+q(2).^2-q(3).^2-q(4).^2 2.0.*(q(2).*q(3)-q(1).*q(4))    2.0.*(q(2).*q(4)+q(1).*q(3))
                    2.0.*(q(2).*q(3)+q(1).*q(4))    q(1).^2-q(2).^2+q(3).^2-q(4).^2 2.0.*(q(3).*q(4)-q(1).*q(2))
                    2.0.*(q(2).*q(4)-q(1).*q(3))    2.0.*(q(3).*q(4)+q(1).*q(2))    q(1).^2-q(2).^2-q(3).^2+q(4).^2];
        end
        
        % Quaternion to rotation matrix
        function q = rot2quat(R)
            % Credits MBosse
            Rv = reshape(R,9,1);
            % The cubed root determinate of R is the scaling factor
            detR = sum(Rv([1 4 7]).*Rv([5 8 2]).*Rv([9 3 6]))- ...
                   sum(Rv([7 1 4]).*Rv([5 8 2]).*Rv([3 6 9]));
            Q2 = detR.^(1/3);
            % Unnormalized unsigned quaternion
            q = sqrt(max(0,[(Q2+Rv(1)+Rv(5)+Rv(9))
                            (Q2+Rv(1)-Rv(5)-Rv(9))
                            (Q2-Rv(1)+Rv(5)-Rv(9))
                            (Q2-Rv(1)-Rv(5)+Rv(9))]))/2;
            % Now copy signs
            g = find(Rv(6)<Rv(8)); q(2,g) = -q(2,g);
            g = find(Rv(7)<Rv(3)); q(3,g) = -q(3,g);
            g = find(Rv(2)<Rv(4)); q(4,g) = -q(4,g);
            Q2 = sum(q.^2);
            % Normalize
            D = 0.5*(1-Q2);
            q = q + q.*D([1 1 1 1],:);
            % Make orientation correct
            q = q';
        end
        
        % Converts euler angles to a rotation matrix
        function R = yprToRotationMatrix(w_z, w_y, w_x)
            % Forming the rotation matrix
            R = [ 1         0           0;
                  0         cos(w_x)    sin(w_x);
                  0         -sin(w_x)   cos(w_x);] * ...
                [ cos(w_y)  0           -sin(w_y);
                  0         1           0;
                  sin(w_y)  0           cos(w_y);] * ...
                [ cos(w_z)  sin(w_z)    0       ;
                  -sin(w_z) cos(w_z)    0       ;
                  0           0         1       ;];
        end
        
    end
    
end