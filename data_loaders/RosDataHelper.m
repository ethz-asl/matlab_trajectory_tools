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
        % Converts a set of pose stamped messages to arrays
        function [positions_stamped] = convertPoseStampedWithCovariance(pose_messages)
            % Initializing
            message_num = size(pose_messages,1);
            times = zeros(message_num,1);
            positions = zeros(message_num,3);
            % Looping over messages and extracting the data
            for message_index = 1:message_num
                message = pose_messages{message_index};
                times(message_index) = seconds(message.Header.Stamp);
                positions(message_index,:) = [message.Pose.Pose.Position.X,...
                                              message.Pose.Pose.Position.Y,...
                                              message.Pose.Pose.Position.Z];
            end
            % Creating the time series for output
            positions_stamped.times = times;
            positions_stamped.positions = positions;
        end
        
        function [positions_stamped] = convertPoseStampedToPositions(pose_messages)
            % Initializing
            message_num = size(pose_messages,1);
            times = zeros(message_num,1);
            positions = zeros(message_num,3);
            % Looping over messages and extracting the data
            for message_index = 1:message_num
                message = pose_messages{message_index};
                times(message_index) = seconds(message.Header.Stamp);
                positions(message_index,:) = [message.Pose.Position.X,...
                                              message.Pose.Position.Y,...
                                              message.Pose.Position.Z];
            end
            % Creating the time series for output
            positions_stamped.times = times;
            positions_stamped.positions = positions;
        end

                
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
        
        function pose_arrays = convertPoseArrayMessages(pose_array_messages)
            % Looping over the messages and converting
            message_num = size(pose_array_messages,1);
            pose_arrays = cell(message_num,1);
            for message_index = 1:message_num
                % Extracting this pose array
                pose_array = pose_array_messages{message_index};
                % Extracting the data in this pose array
                pose_num = size(pose_array.Poses, 1);
                positions = zeros(pose_num,3);
                orientations = zeros(pose_num,4);
                for pose_index = 1:pose_num
                    pose = pose_array.Poses(pose_index);
                    positions(pose_index,:) = [ pose.Position.X,...
                                                pose.Position.Y,...
                                                pose.Position.Z];
                    orientations(pose_index,:) = [  pose.Orientation.W,...
                                                    pose.Orientation.X,...
                                                    pose.Orientation.Y,...
                                                    pose.Orientation.Z];
                end
                % Output
                pose_arrays{message_index}.positions = positions;
                pose_arrays{message_index}.orientations = orientations;
            end
            
        end
        
        function pose_arrays = convertTransformStampedArrayMessages(transform_stamped_array_messages)
            % Looping over the messages and converting
            message_num = size(transform_stamped_array_messages,1);
            pose_arrays = cell(message_num,1);
            for message_index = 1:message_num
                % Extracting this pose array
                transform_stamped_array = transform_stamped_array_messages{message_index};
                % Extracting the data in this pose array
                transform_num = size(transform_stamped_array.Transforms, 1);
                times = zeros(transform_num,1);
                translations = zeros(transform_num,3);
                rotations = zeros(transform_num,4);
                for transform_index = 1:transform_num
                    times(transform_index) = seconds(transform_stamped_array.Transforms(transform_index).Header.Stamp);
                    transform = transform_stamped_array.Transforms(transform_index).Transform;
                    translations(transform_index,:) = [ transform.Translation.X,...
                                                        transform.Translation.Y,...
                                                        transform.Translation.Z];
                    rotations(transform_index,:) = [  transform.Rotation.W,...
                                                      transform.Rotation.X,...
                                                      transform.Rotation.Y,...
                                                      transform.Rotation.Z];
                end
                % Output
                pose_arrays{message_index}.times = times;
                pose_arrays{message_index}.translations = translations;
                pose_arrays{message_index}.rotations = rotations;
            end
            
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
        
        % Converts a set of position stamped messages to arrays
        function vectors_stamped = convertVector3StampedMessages(vector3_stamped_messages)
            % Initializing
            message_num = size(vector3_stamped_messages,1);
            times = zeros(message_num,1);
            vectors = zeros(message_num,3);
            % Looping over messages and extracting the data
            for message_index = 1:message_num
                message = vector3_stamped_messages{message_index};
                times(message_index) = seconds(message.Header.Stamp);
                vectors(message_index,:) = [message.Vector.X, message.Vector.Y, message.Vector.Z];
            end
            % Creating the time series for output
            vectors_stamped.times = times;
            vectors_stamped.vectors = vectors;
        end
        
        % Converts a set of position stamped messages to arrays
        function points_stamped = convertPointStampedMessages(point_stamped_messages)
            % Initializing
            message_num = size(point_stamped_messages,1);
            times = zeros(message_num,1);
            points = zeros(message_num,3);
            % Looping over messages and extracting the data
            for message_index = 1:message_num
                message = point_stamped_messages{message_index};
                times(message_index) = seconds(message.Header.Stamp);
                points(message_index,:) = [message.Point.X, message.Point.Y, message.Point.Z];
            end
            % Creating the time series for output
            points_stamped.times = times;
            points_stamped.points = points;
        end

        
    end
    
end
