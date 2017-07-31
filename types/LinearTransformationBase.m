classdef LinearTransformationBase < handle & matlab.mixin.Copyable
    %TRAJECTORY Encapsulates trajectory functionality
    %   Detailed explanation goes here
    
    properties
        % The matrix form of the transformation
        % Dependant on the other data members and updated through
        % updateTransformationMatrix(obj)
        transformation_matrix
    end
    
    methods
        
        % Constructor
        function obj = LinearTransformationBase()
            % Do nothing (yet...)
        end
        
        % Returns the 4x4 transformation 
        function T = getTransformationMatrix(obj)
            T = obj.transformation_matrix;
        end
        
    end
    
    % Compulsory methods for transformation types to implement
    methods(Abstract)
        
        % Updates the transformation matrix member from the other members
        updateTransformationMatrix(obj)
        
        % Initialize from a transformation matrix
        initializeFromMatrix(obj, T)
            
        % Returns the inverse transformation
        transform_inv = inverse(obj)
        
        % Transforms a set of vectors
        vecs_transformed = transformVectorsLHS(obj, vecs)
        
        % The times operator
        r = mtimes(obj_1, obj_2)
        
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
        
    end
    
end

