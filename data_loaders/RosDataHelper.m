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
        function positions_stamped = convertPositionStampedMessages(position_stamped_messages)
            % Initializing
            message_num = size(position_stamped_messages,1);
            times = zeros(message_num,1);
            positions = zeros(message_num,3);
            % Looping over messages and extracting the data
            for message_index = 1:message_num
                message = position_stamped_messages{message_index};
                times(message_index) = seconds(message.Header.Stamp);
                positions(message_index,:) = [message.Point.X, message.Point.Y, message.Point.Z];
            end
            % Creating the time series for output
            positions_stamped.times = times;
            positions_stamped.positions = positions;
        end
        
        % TODO(millane) Convert the whole message not just part of it.
%         % Converts a set of position stamped messages to arrays
%         function [position_data] = convertPoseStampedWithCovarianceToPostionTimeseries(pose_messages)
%             % Initializing
%             message_num = size(pose_messages,1);
%             position_data_time = zeros(message_num,1);
%             position_data_data = zeros(message_num,3);
%             % Looping over messages and extracting the data
%             for message_index = 1:message_num
%                 message = pose_messages{message_index};
%                 position_data_time(message_index) = seconds(message.Header.Stamp);
%                 position_data_data(message_index,:) = [message.Pose.Pose.Position.X,...
%                                                        message.Pose.Pose.Position.Y,...
%                                                        message.Pose.Pose.Position.Z];
%             end
%             % Creating the time series for output
%             position_data = timeseries(position_data_data, position_data_time);
%         end
                
        function odometry = convertOdometryMessages(odometry_messages)
            % Initializing
            message_num = size(odometry_messages,1);
            times = zeros(message_num,1);
            positions = zeros(message_num,3);
            orientations = zeros(message_num,4);
            pose_covariances = zeros(message_num,6,6);
            linear_velocities = zeros(message_num,3);
            rotational_velocities = zeros(message_num,3);
            velocity_covariances = zeros(message_num,6,6);
            % Looping over messages and extracting the data
            for message_index = 1:message_num
                message = odometry_messages{message_index};
                times(message_index) = seconds(message.Header.Stamp);
                positions(message_index,:) = [message.Pose.Pose.Position.X,...
                                             message.Pose.Pose.Position.Y,...
                                             message.Pose.Pose.Position.Z];
                orientations(message_index,:) = [message.Pose.Pose.Orientation.W,...
                                                message.Pose.Pose.Orientation.X,...
                                                message.Pose.Pose.Orientation.Y,...
                                                message.Pose.Pose.Orientation.Z];
                pose_covariances(message_index,:,:) = reshape(message.Pose.Covariance, 6, 6);
                linear_velocities(message_index,:) = [message.Twist.Twist.Linear.X,...
                                                    message.Twist.Twist.Linear.Y,...
                                                    message.Twist.Twist.Linear.Z];
                rotational_velocities(message_index,:) = [message.Twist.Twist.Angular.X,...
                                                        message.Twist.Twist.Angular.Y,...
                                                        message.Twist.Twist.Angular.Z];
                velocity_covariances(message_index,:,:) = reshape(message.Twist.Covariance, 6, 6);
            end
            % Creating the time series for output
            odometry.times = times;
            odometry.positions = positions;
            odometry.orientation = orientations;
            odometry.pose_covariances = pose_covariances;
            odometry.linear_velocities = linear_velocities;
            odometry.rotational_velocities = rotational_velocities;
            odometry.velocity_covariances = velocity_covariances;
        end
        
        % Converts a set of transform stamped messages to position arrays
        function transforms = convertTransformStampedMessages(transform_stamped_messages)
            % Initializing
            message_num = size(transform_stamped_messages,1);
            times = zeros(message_num,1);
            positions = zeros(message_num,3);
            orientations = zeros(message_num,4);
            % Looping over messages and extracting the data
            for message_index = 1:message_num
                message = transform_stamped_messages{message_index};
                times(message_index) = seconds(message.Header.Stamp);
                positions(message_index,:) = [ message.Transform.Translation.X,...
                                              message.Transform.Translation.Y,...
                                              message.Transform.Translation.Z];
                orientations(message_index,:) = [message.Transform.Rotation.W,...
                                                message.Transform.Rotation.X,...
                                                message.Transform.Rotation.Y,...
                                                message.Transform.Rotation.Z];
            end
            % Creating the time series for output
            transforms.times = times;
            transforms.positions = positions;
            transforms.orientations = orientations;
        end
        
        % Converts a range message stream to a timeseries
        % Converts a set of position stamped messages to arrays
        function ranges_stamped = convertRangeStampedMessages(range_messages)
            % Initializing
            message_num = size(range_messages,1);
            times = zeros(message_num,1);
            ranges = zeros(message_num,1);
            % Looping over messages and extracting the data
            for message_index = 1:message_num
                message = range_messages{message_index};
                times(message_index) = seconds(message.Header.Stamp);
                ranges(message_index,:) = message.Range_;
            end
            % Creating the time series for output
            ranges_stamped.times = times;
            ranges_stamped.range = ranges;
        end
        
    end
    
end
