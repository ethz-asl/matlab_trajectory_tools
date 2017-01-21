classdef RosDataHelper < handle
    %ROSDATAHELPER Helps with interpretting ros bags
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods
        
        % Constructor
        function obj = RosDataHelper()

        end
        
    end
    
    methods(Static)
       
        % Converts a set of position stamped messages to arrays
        function [position_data] = convertPositionStampedToTimeseries(position_stamped_messages)
            % Initializing
            message_num = size(position_stamped_messages,1);
            position_data_time = zeros(message_num,1);
            position_data_data = zeros(message_num,3);
            % Looping over messages and extracting the data
            for message_index = 1:message_num
                message = position_stamped_messages{message_index};
                position_data_time(message_index) = seconds(message.Header.Stamp);
                position_data_data(message_index,:) = [message.Point.X, message.Point.Y, message.Point.Z];
            end
            % Creating the time series for output
            position_data = timeseries(position_data_data, position_data_time);
        end
        
        % Converts a set of position stamped messages to arrays
        function [position_data] = convertPoseStampedWithCovarianceToPostionTimeseries(pose_messages)
            % Initializing
            message_num = size(pose_messages,1);
            position_data_time = zeros(message_num,1);
            position_data_data = zeros(message_num,3);
            % Looping over messages and extracting the data
            for message_index = 1:message_num
                message = pose_messages{message_index};
                position_data_time(message_index) = seconds(message.Header.Stamp);
                position_data_data(message_index,:) = [message.Pose.Pose.Position.X,...
                                                       message.Pose.Pose.Position.Y,...
                                                       message.Pose.Pose.Position.Z];
            end
            % Creating the time series for output
            position_data = timeseries(position_data_data, position_data_time);
        end
                
        function odometry = convertOdometryToTimeSeries(odometry_messages)
            % Initializing
            message_num = size(odometry_messages,1);
            time = zeros(message_num,1);
            position = zeros(message_num,3);
            orientation = zeros(message_num,4);
            pose_covariance = zeros(message_num,6,6);
            linear_velocity = zeros(message_num,3);
            rotational_velocity = zeros(message_num,3);
            velocity_covariance = zeros(message_num,6,6);
            % Looping over messages and extracting the data
            for message_index = 1:message_num
                message = odometry_messages{message_index};
                time(message_index) = seconds(message.Header.Stamp);
                position(message_index,:) = [message.Pose.Pose.Position.X,...
                                             message.Pose.Pose.Position.Y,...
                                             message.Pose.Pose.Position.Z];
                orientation(message_index,:) = [message.Pose.Pose.Orientation.W,...
                                                message.Pose.Pose.Orientation.X,...
                                                message.Pose.Pose.Orientation.Y,...
                                                message.Pose.Pose.Orientation.Z];
                pose_covariance(message_index,:,:) = reshape(message.Pose.Covariance, 6, 6);
                linear_velocity(message_index,:) = [message.Twist.Twist.Linear.X,...
                                                    message.Twist.Twist.Linear.Y,...
                                                    message.Twist.Twist.Linear.Z];
                rotational_velocity(message_index,:) = [message.Twist.Twist.Angular.X,...
                                                        message.Twist.Twist.Angular.Y,...
                                                        message.Twist.Twist.Angular.Z];
                velocity_covariance(message_index,:,:) = reshape(message.Twist.Covariance, 6, 6);
            end
            % Creating the time series for output
            odometry.position = timeseries(position, time);
            odometry.orientation = timeseries(orientation, time);
            odometry.pose_covariance = pose_covariance;
            odometry.linear_velocity = timeseries(linear_velocity, time);
            odometry.rotational_velocity = timeseries(rotational_velocity, time);
            odometry.velocity_covariance = velocity_covariance;
        end
        
        % Converts a set of transform stamped messages to position arrays
        function [transform] = convertTransformStampedToTimeseries(transform_stamped_messages)
            % Initializing
            message_num = size(transform_stamped_messages,1);
            time = zeros(message_num,1);
            position = zeros(message_num,3);
            orientation = zeros(message_num,4);
            % Looping over messages and extracting the data
            for message_index = 1:message_num
                message = transform_stamped_messages{message_index};
                time(message_index) = seconds(message.Header.Stamp);
                position(message_index,:) = [ message.Transform.Translation.X,...
                                              message.Transform.Translation.Y,...
                                              message.Transform.Translation.Z];
                orientation(message_index,:) = [message.Transform.Rotation.W,...
                                                message.Transform.Rotation.X,...
                                                message.Transform.Rotation.Y,...
                                                message.Transform.Rotation.Z];
            end
            % Creating the time series for output
            transform.position = timeseries(position, time);
            transform.orientation = timeseries(orientation, time);
        end
        
        % Converts a range message stream to a timeseries
        % Converts a set of position stamped messages to arrays
        function [range] = convertRangeToTimeseries(range_messages)
            % Initializing
            message_num = size(range_messages,1);
            range_time = zeros(message_num,1);
            range_data = zeros(message_num,1);
            % Looping over messages and extracting the data
            for message_index = 1:message_num
                message = range_messages{message_index};
                range_time(message_index) = seconds(message.Header.Stamp);
                range_data(message_index,:) = message.Range_;
            end
            % Creating the time series for output
            range = timeseries(range_data, range_time);
        end
        
    end
    
end
