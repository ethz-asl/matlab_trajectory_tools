classdef PoseTrajectoryAlignerBase
    %POSETRAJECTORYALIGNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods
        
        % Constructor
        function obj = PoseTrajectoryAlignerBase()

        end
        
    end
    
    methods(Static, Abstract)
        % Calculates the alignment transform from the trajectories passed
        T_alignment = calculateAlignmentTransform(aligned_to_trajectory,...
                                                  aligned_from_trajectory,...
                                                  orientation_scaling,...
                                                  sigma_init)        
    end
    
    methods(Static)
       
        % Truncated and resamples data streams such that they contain
        % measurements which correspond in time.
        function [fast_data_resampled, slow_data_resampled] = truncateAndResampleDatastreams(...
                                                                fast_data, slow_data)
            % Extracting position and orientation time series
%             fast_positions = fast_data.positions();
%             slow_positions = slow_data.positions();
%             fast_orientations = fast_data.orientations();
%             slow_orientations = slow_data.orientations();
            % Truncating the sample times stream such that it lies inside data
            fast_data_start_time = fast_data.times(1);
            fast_data_end_time = fast_data.times(end);
            slow_data_start_index = find(slow_data.times() > fast_data_start_time, 1, 'first');
            slow_data_end_index = find(slow_data.times() < fast_data_end_time, 1, 'last');
            % Constructing time series from trancated data
            slow_positions_truncated_timeseries = timeseries(slow_data.positions(slow_data_start_index:slow_data_end_index,:),...
                                                             slow_data.times(slow_data_start_index:slow_data_end_index));
            fast_positions_truncated_timeseries = timeseries(fast_data.positions(), fast_data.times());
            slow_orientations_truncated_timeseries = timeseries(slow_data.orientations(slow_data_start_index:slow_data_end_index,:),...
                                                                slow_data.times(slow_data_start_index:slow_data_end_index));
            fast_orientations_truncated_timeseries = timeseries(fast_data.orientations(), fast_data.times());
            % Resampling the data
            slow_positions_resampled_timeseries = slow_positions_truncated_timeseries;
            fast_positions_resampled_timeseries = resample(fast_positions_truncated_timeseries,...
                                                           slow_positions_truncated_timeseries.Time);                                                              
            slow_orientations_resampled_timeseries = slow_orientations_truncated_timeseries;
            fast_orientations_resampled_timeseries = PoseTrajectoryAlignerBase.quaternionResample(...
                                                fast_orientations_truncated_timeseries,...
                                                slow_orientations_truncated_timeseries.Time);
            % Output objects
            fast_data_resampled = TransformationTrajectory(fast_orientations_resampled_timeseries.Data,...
                                                           fast_positions_resampled_timeseries.Data,...
                                                           fast_positions_resampled_timeseries.Time);
            slow_data_resampled = TransformationTrajectory(slow_orientations_resampled_timeseries.Data,...
                                                           slow_positions_resampled_timeseries.Data,...
                                                           slow_positions_resampled_timeseries.Time);                                                           
        end
        
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
        
%         % Apply an alignment transform to a trajectory
%         function position_trajectory_aligned = alignTrajectory(position_trajectory, T_alignment)
%             % Performing alignment
%             trajectory_length = size(position_trajectory.Data,1);
%             position_trajectory_aligned_data = T_alignment * [position_trajectory.Data'; ones(1, trajectory_length)];
%             position_trajectory_aligned_data = position_trajectory_aligned_data(1:3,:)';
%             % Writing to timeseries
%             position_trajectory_aligned = timeseries(position_trajectory_aligned_data, position_trajectory.Time);
%         end
        
%         % Calculate the RMS of the error vector between two trajectories.
%         function rms_position_error = calculateRmsPositionError(...
%                 to_trajectory, from_trajectory, T_alignment)
%             % Checks
%             if (size(to_trajectory.Data,1) ~= size(from_trajectory.Data,1))
%                 error('Vectors to be aligned must have the same size.')
%             end
%             % Align from_trajectory.
%             aligned_from_trajectory = ...
%                 PositionTrajectoryAlignerBase.alignTrajectory(from_trajectory, T_alignment);
%             % Calculate the errors in x y z.
%             error_x_y_z = aligned_from_trajectory.Data - to_trajectory.Data;
%             % Calculate the residuals.
%             error_norm = sqrt(sum(abs(error_x_y_z).^2,2));
%             rms_position_error = rms(error_norm);
%         end
        
%         % Calculate the trajectory length.
%         function trajectory_length = calculateTrajectoryLength(trajectory)
%             N = length(trajectory.Time);
%             trajectory_length = 0.0;
%             % Sum of all data point connections.
%             for i = 1:N-1
%                 delta_s = norm(trajectory.Data(i+1,:) - trajectory.Data(i,:));
%                 trajectory_length = trajectory_length + delta_s;
%             end
%         end
        
    end
    
end
