classdef Transformation < handle & matlab.mixin.Copyable
    %TRAJECTORY Encapsulates trajectory functionality
    %   Detailed explanation goes here
    
    properties
        % Raw data
        orientation_quat
        position
        % Dependant data
        transformation_matrix
    end
    
    methods
        
        % Constructor
        function obj = Transformation(orientation_quat, position)
            % If no arguments initialize identity transform
            if nargin == 0
                orientation_quat = [1 0 0 0];
                position = [0 0 0];
            end
            % Setting the data
            obj.setData(orientation_quat, position)
        end
                
        % Sets the data
        function setData(obj, orientation_quat, position)
            % Checks
            assert(length(orientation_quat) == 4, 'Quaternion incorrectly sized');
            assert(length(position) == 3, 'Position incorrectly sized');
            eps = 1e-6;
            assert(abs(norm(orientation_quat)-1) < eps, 'Quaternion does not have unit norm');
            % Saving the data
            obj.orientation_quat = orientation_quat;
            obj.position = position;
            % Calculating transformation matrix
            % NOTE(alexmillane): This might slow things down if you make
            % many transformation matrices
            obj.transformation_matrix = obj.calculateTransformationMatrix();
        end
        
        % Initialize from a transformation matrix
        function initializeFromMatrix(obj, T)
            [R, t] = obj.transformationMatrix2Parts(T);
            q = obj.rot2quat(R);
            obj.setData(q, t);
        end
        
        % Operator times
        function r = mtimes(obj_1, obj_2)
            % Composition through matrix multiplication
            T = obj_1.transformation_matrix() * obj_2.transformation_matrix();
            % Extracting the parts and forming a transform object
            R = T(1:3,1:3);
            t = T(1:3,4)';
            q = obj_1.rot2quat(R);
            % Forming the transformation object
            r = Transformation(q, t);
            % TODO(alexmillane): Probably quicker to operate with quaternions directly
            %                    as in minkindr.
%             r_q = obj_1.quatmult(obj_1.orientation_quat(), obj_2.orientation_quat())
        end
        
        % Returns the 4x4 transformation 
        function T = getTransformationMatrix(obj)
            T = obj.transformation_matrix;
        end
        
        % Returns the 4x4 transformation 
        function T = calculateTransformationMatrix(obj)
            R = obj.quat2rot(obj.orientation_quat);
            p = obj.position';
            T = [R p ; 0 0 0 1];
        end
        
        % Transforms a set of vectors
        function vecs_transformed = transformVectorsLHS(obj, vecs)
            % Checks
            assert(size(vecs,2) == 3, 'Needs to be nx3 matrix');
            % Getting the transformed vectors
            vecs_hom = [vecs'; zeros(1, size(vecs,1))];
            vecs_transformed_hom = obj.transformation_matrix * vecs_hom;
            vecs_transformed = vecs_transformed_hom(1:3,:)';
        end
        
        % Returns the inverse transformation
        function transform_inv = inverse(obj)
            % TODO(alexmillane): This is an extremely inefficient way of
            %                    doing this. Correct in the future (when needed).
            T = obj.getTransformationMatrix();
            T_inv = inv(T);
            R_inv = T_inv(1:3,1:3);
            t_inv = T_inv(1:3,4)';
            q_inv = obj.rot2quat(R_inv);
            transform_inv = Transformation(q_inv, t_inv);
        end
        
        % Gets the rotation matrix
        function R = getRotationMatrix(obj)
            R = obj.transformation_matrix(1:3,1:3);
        end
        
        % Gets the translation vector
        function t = getTranslationVector(obj)
            t = obj.transformation_matrix(1:3,4);
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
            t = obj.getTranslationVector();
            trans_x = R * unit_x;
            trans_y = R * unit_y;
            trans_z = R * unit_z;
            % Plotting
            quiver3(t(1), t(2), t(3), trans_x(1), trans_x(2), trans_x(3), length, 'r');
            quiver3(t(1), t(2), t(3), trans_y(1), trans_y(2), trans_y(3), length, 'g');
            quiver3(t(1), t(2), t(3), trans_z(1), trans_z(2), trans_z(3), length, 'b');
            if ~holdstate
              hold off
            end
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
        
        % Multiplies two quaternions
        function r = quatmult(p,q)
            % Credits MBosse
            r = [   p(1)*q(1) - sum(p(2:4).*q(2:4)), ...
                    p([1 1 1]).*q(2:4) +     ...
                    q([1 1 1]).*p(2:4) +     ...
                    p([3 4 2]).*q([4 2 3]) - ...
                    p([4 2 3]).*q([3 4 2])      ];
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
        
        % Returns the parts of a transformation matrix
        function [R, t] = transformationMatrix2Parts(T)
            R = T(1:3,1:3);
            t = T(1:3,4)';
        end
        
    end
    
end

