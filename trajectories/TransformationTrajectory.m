classdef TransformationTrajectory < handle & matlab.mixin.Copyable
    %TRANSFORMATIONTRAJECTORY Encapsulates trajectory functionality
    %   Detailed explanation goes here
    
    properties
        % Raw data
        times
        positions
        orientations
        % Properites
        length
    end
    
    methods
        
        % Constructor
        % Initializes the trajectory from a timeseries 
        function obj = TransformationTrajectory(orientations, positions, times)
            % Setting the data
            obj.setData(orientations, positions, times);
        end
        
        % Sets the data
        function setData(obj, orientations, positions, times)
            % Checks
            assert( size(times, 1) == size(orientations, 1), 'Number of orientations must be the same as the number of times')
            assert( size(times, 1) == size(positions, 1), 'Number of positions must be the same as the number of times')
            % Saving the data
            obj.times = times;
            obj.positions = positions;
            obj.orientations = orientations;
            % Properties
            obj.length = size(times, 1);
        end
        
        % Returns a transformation by index
        function T = getTransformation(obj, index)
            T = Transformation(obj.orientations(index,:), obj.positions(index,:));
        end
        
        % Returns a trajectory which is the inverse of the 
        function trajectory_inverse = inverse(obj)
            % Combined vector form
            trajectory_vec = [obj.positions obj.orientations];
            % Getting inverse
            trajectory_inverse_vec = k_tf_inv(trajectory_vec);
            % Converting to object
            q = trajectory_inverse_vec(:,4:7);
            t = trajectory_inverse_vec(:,1:3);
            trajectory_inverse = TransformationTrajectory(q, t, obj.times);
        end
        
        % Truncates the trajectory to indicies
        function truncateToIndicies(obj, start_index, end_index)
            % Rewriting the data
            obj.setData(obj.orientations(start_index:end_index, :),...
                        obj.positions(start_index:end_index, :),...
                        obj.times(start_index:end_index));
        end
        
        % Resamples the trajectory at the passed times
        function resample(obj, times)
            % Creating time series
            orientation_timeseries = timeseries(obj.orientations, obj.times);
            position_timeseries = timeseries(obj.positions, obj.times);
            % Resampling
            orientation_timeseries_resampled = obj.quaternionResample(orientation_timeseries, times);
            position_timeseries_resampled = position_timeseries.resample(times);
            % Updating the object
            obj.setData(orientation_timeseries_resampled.Data(),...
                        position_timeseries_resampled.Data(),...
                        position_timeseries_resampled.Time());
        end
        
        % Applies a transformation to the trajectory
        function transformed_trajectory = applyStaticTransformLHS(obj, T_static)
            % Combined vector form
            T_vec_obj = [obj.positions obj.orientations];
            T_vec_static = [T_static.position T_static.orientation_quat];
            %   Doing composition
            T_vec_transformed = k_tf_mult(T_vec_static, T_vec_obj);
            % Converting to object
            q = T_vec_transformed(:,4:7);
            t = T_vec_transformed(:,1:3);
            transformed_trajectory = TransformationTrajectory(q, t, obj.times);
        end
        
        % Applies a transformation to the trajectory
        function transformed_trajectory = applyStaticTransformRHS(obj, T_static)
            % Combined vector form
            T_vec_obj = [obj.positions obj.orientations];
            T_vec_static = [T_static.position T_static.orientation_quat];
            %   Doing composition
            T_vec_transformed = k_tf_mult(T_vec_obj, T_vec_static);
            % Converting to object
            q = T_vec_transformed(:,4:7);
            t = T_vec_transformed(:,1:3);
            transformed_trajectory = TransformationTrajectory(q, t, obj.times);
        end
        
        % Applies a Similarity transformation to the trajectory
        function transformed_trajectory = applyStaticSimTransformLHS(obj, T_static)
            % Combined vector form
            T_vec_obj = [obj.positions obj.orientations];
            T_vec_static = [T_static.position T_static.orientation_quat];
            %   Doing composition (without scale)
            T_vec_transformed = k_tf_mult(T_vec_static, T_vec_obj);
            % Converting to object (and scaling)
            q = T_vec_transformed(:,4:7);
            t = T_vec_transformed(:,1:3) * T_static.scale;
            transformed_trajectory = TransformationTrajectory(q, t, obj.times);
        end
              
        % Returns the position trajectory
        function position_trajectory = getPositionTrajectory(obj)
            position_trajectory = PositionTrajectory(obj.positions, obj.times);
        end
        
        function orientation_trajectory = getOrientationTrajectory(obj)
            orientation_trajectory = OrientationTrajectory(obj.orientations, obj.times);
        end
        
        % Composes this trajectory with another
        function transformed_trajectory = compose(obj, trajectory_other)
            % Combined vector form
            T_vec_obj = [obj.positions obj.orientations];
            T_vec_other = [trajectory_other.positions trajectory_other.orientations];
            %   Doing composition
            T_vec_transformed = k_tf_mult(T_vec_obj, T_vec_other);
            % Converting to object
            q = T_vec_transformed(:,4:7);
            t = T_vec_transformed(:,1:3);
            transformed_trajectory = TransformationTrajectory(q, t, obj.times);
        end
        
        % Gets a windowed portion of this trajectory
        function windowed_trajectory = getWindowedTrajectory(obj, start_index, end_index)
            % Checks
            assert(start_index >= 1, 'Start index should be greater than 1.');
            assert(end_index <= obj.length, 'End index should be less than length.');
            % Creating the timeseries
            windowed_times = obj.times(start_index:end_index);
            windowed_positions = obj.positions(start_index:end_index,:);
            windowed_orientations = obj.orientations(start_index:end_index,:);
            % Creating the trajectory object
            windowed_trajectory = TransformationTrajectory(windowed_orientations, windowed_positions, windowed_times);
        end
           
        % Plots the trajectory
        function h = plot(obj, cs_axis_length, symbol)
            if nargin < 3
                symbol = '';
            end
            if nargin < 2
                cs_axis_length=0.1;
            end
            holdstate = ishold;
            hold on
            % Plotting the position trajectory
            h_temp = obj.getPositionTrajectory().plot(symbol);
            % Plotting the transformation axis
            for index = 1:obj.length
                obj.getTransformation(index).plot(cs_axis_length);
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

        % Resamples a quaternion time series using linear interpolation
        function quats_resampled = quaternionResample(quats, times)
            % Checks
            assert(times(1) > quats.Time(1), 'Sampling times need to be in between data.')
            assert(times(end) < quats.Time(end), 'Sampling times need to be in between data.')
            % Getting the interpolation amounts
            dummy_timeseries = timeseries((1:quats.Length)', quats.Time);
            interpolation_factors = dummy_timeseries.resample(times).Data;
            % Looping over the times and sampling
            slerp_quats_low = zeros(length(times),4);
            slerp_quats_high = zeros(length(times),4);
            slerp_coeffs = zeros(length(times),1);
            for index = 1:length(times)
                % Getting the interpolation end points and coeff
                interpolation_factor = interpolation_factors(index);
                low_index = floor(interpolation_factor);
                high_index = ceil(interpolation_factor);
                interpolation_coeff = interpolation_factor - low_index;
                % Slerping
                slerp_quats_low(index,:) = quats.Data(low_index,:);
                slerp_quats_high(index,:) = quats.Data(high_index,:);
                slerp_coeffs(index) = interpolation_coeff;
            end
            % Slerping 2
            quat_resampled_array = quatinterp(slerp_quats_low, slerp_quats_high, slerp_coeffs);
            % Storing as timeseries for output
            quats_resampled = timeseries(quat_resampled_array, times);
        end
                     
    end
    
end

