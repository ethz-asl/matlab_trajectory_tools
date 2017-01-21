classdef TransformationTrajectory < handle & matlab.mixin.Copyable
    %TRANSFORMATIONTRAJECTORY Encapsulates trajectory functionality
    %   Detailed explanation goes here
    
    properties
        % Raw timeseries data
        positions
        orientations
        % Properites
        length
    end
    
    methods
        
        % Constructor
        % Initializes the trajectory from a timeseries 
        function obj = TransformationTrajectory(orientations, positions)
            % Setting the data
            obj.setData(orientations, positions);
        end
        
        % Sets the data from two timeseries
        function setData(obj, orientations, positions)
            % Checks
            assert( size(positions.Time, 1) == size(orientations.Time, 1))
            % Saving the data
            obj.positions = positions;
            obj.orientations = orientations;
            obj.length = size(positions.Time, 1);
        end
        
        % Returns a transformation by index
        function T = getTransformation(obj, index)
            T = Transformation(obj.orientations.Data(index,:), obj.positions.Data(index,:));
        end
        
        % Returns a trajectory which is the inverse of the 
        function trajectory_inverse = inverse(obj)
            % Combined vector form
            trajectory_vec = [obj.positions.Data obj.orientations.Data];
            % Getting inverse
            trajectory_inverse_vec = k_tf_inv(trajectory_vec);
            % Converting to object
            q = timeseries(trajectory_inverse_vec(:,4:7), obj.positions.Time);
            t = timeseries(trajectory_inverse_vec(:,1:3), obj.positions.Time);
            trajectory_inverse = TransformationTrajectory(q, t);
        end
        
        % Applies a transformation to the trajectory
        function transformed_trajectory = applyStaticTransformLHS(obj, T_static)
            % Combined vector form
            T_vec_obj = [obj.positions.Data obj.orientations.Data];
            T_vec_static = [T_static.position T_static.orientation_quat];
            %   Doing composition
            T_vec_transformed = k_tf_mult(T_vec_static, T_vec_obj);
            % Converting to object
            q = timeseries(T_vec_transformed(:,4:7), obj.positions.Time);
            t = timeseries(T_vec_transformed(:,1:3), obj.positions.Time);
            transformed_trajectory = TransformationTrajectory(q, t);
        end
        
        % Applies a transformation to the trajectory
        function transformed_trajectory = applyStaticTransformRHS(obj, T_static)
            % Combined vector form
            T_vec_obj = [obj.positions.Data obj.orientations.Data];
            T_vec_static = [T_static.position T_static.orientation_quat];
            %   Doing composition
            T_vec_transformed = k_tf_mult(T_vec_obj, T_vec_static);
            % Converting to object
            q = timeseries(T_vec_transformed(:,4:7), obj.positions.Time);
            t = timeseries(T_vec_transformed(:,1:3), obj.positions.Time);
            transformed_trajectory = TransformationTrajectory(q, t);
        end
              
        % Returns the position trajectory
        function position_trajectory = getPositionTrajectory(obj)
            position_trajectory = PositionTrajectory3D(obj.positions);
        end
        
        % Composes this trajectory with another
        function transformed_trajectory = compose(obj, trajectory_other)
            % Combined vector form
            T_vec_obj = [obj.positions.Data obj.orientations.Data];
            T_vec_other = [trajectory_other.positions.Data trajectory_other.orientations.Data];
            %   Doing composition
            T_vec_transformed = k_tf_mult(T_vec_obj, T_vec_other);
            % Converting to object
            q = timeseries(T_vec_transformed(:,4:7), obj.positions.Time);
            t = timeseries(T_vec_transformed(:,1:3), obj.positions.Time);
            transformed_trajectory = TransformationTrajectory(q, t);
        end
        
        % Gets a windowed portion of this trajectory
        function windowed_trajectory = getWindowedTrajectory(obj, start_index, end_index)
            % Checks
            assert(start_index >= 1, 'Start index should be greater than 1.');
            assert(end_index <= obj.length, 'End index should be less than length.');
            % Creating the timeseries
            windowed_times = obj.positions.Time(start_index:end_index);
            windowed_positions = timeseries(obj.positions.Data(start_index:end_index,:), windowed_times);
            windowed_orientations = timeseries(obj.orientations.Data(start_index:end_index,:), windowed_times);
            % Creating the trajectory object
            windowed_trajectory = TransformationTrajectory(windowed_orientations, windowed_positions);
        end
                
        % Plots the trajectory
        function h = plot(obj, step, length, symbol)
            if nargin < 4
                symbol = '';
            end
            holdstate = ishold;
            hold on
            % Plotting the position trajectory
            h_temp = obj.getPositionTrajectory().plot(symbol);
            % Plotting the transformation axis
            for index = 1:step:obj.length()
                obj.getTransformation(index).plot(length);
            end
            if nargout > 0 
                h = h_temp;
            end
            if ~holdstate
              hold off
            end
        end
        
    end
    
    methods(Static)
       
        % Converts a vector of rotation matrices to a vector of quaternions
        function q = rot2quat(R)
            % Credits MBosse
            % Alex -> Mbosse
            Rb = permute(R, [2,3,1]);
            % Reshaping from 3x3xn to 9xn
            Rv = reshape(Rb,9,[]);
            % the cubed root determinate of R is the scaling factor
            detR = sum(Rv([1 4 7],:).*Rv([5 8 2],:).*Rv([9 3 6],:))- ...
                   sum(Rv([7 1 4],:).*Rv([5 8 2],:).*Rv([3 6 9],:));
            Q2 = detR.^(1/3);
            % Forming the quaternions
            qb = sqrt(max(0,[(Q2+Rv(1,:)+Rv(5,:)+Rv(9,:))
                            (Q2+Rv(1,:)-Rv(5,:)-Rv(9,:))
                            (Q2-Rv(1,:)+Rv(5,:)-Rv(9,:))
                            (Q2-Rv(1,:)-Rv(5,:)+Rv(9,:))]))/2;
            % now copy signs
            g = find(Rv(6,:)<Rv(8,:)); qb(2,g) = -qb(2,g);
            g = find(Rv(7,:)<Rv(3,:)); qb(3,g) = -qb(3,g);
            g = find(Rv(2,:)<Rv(4,:)); qb(4,g) = -qb(4,g);

            Q2 = sum(qb.^2);
            % normalize
            D = 0.5*(1-Q2);
            qb = qb + qb.*D([1 1 1 1],:);
            % Mbosse -> Alex
            q = permute(qb,[2,1]);
        end
        
    end
    
end

