function [T_align, r_align, q_align, rms_average, odom_poses_aligned, vicon_poses_resampled, q_errs] = align_trajectory_positions3d(bagfile, odom_topic, gt_topic)
%% align_trajectories
%
% Aligns rtk and lieca trajectories for MBZIRC.
%
% --
%
% (c)   November 2015 by
%       Alexander Millane
%       ETH Zurich, ASL
%       millanea(at)ethz.ch
%
% Revision History:
% [17.11.16, AM] file created
%
%% handle bag topic selection
baginfo = rosbag('info', bagfile);
bag_select = rosbag(bagfile);
topiclist = {baginfo.Topics(:).Topic};
%check for topic existence
if ~any(strcmp(topiclist,gt_topic))
    for i=1:length(topiclist)
        topic=topiclist{i};
        if contains(topic, 'leica')
            fprintf('Could not find ground_truth topic. Setting it to:\n');            
            gt_topic=topic           
            break;
        else
            if contains(topic, 'vrpn_client')
                fprintf('Could not find ground_truth topic. Setting it to:\n');            
                gt_topic=topic
                break;
            end
        end
    end
end
if ~any(strcmp(topiclist,odom_topic))
    for i=1:length(topiclist)
        topic=topiclist{i};
        if contains(topic, 'odom')
            if ~strcmp(topic,gt_topic)
                fprintf('Could not find odometry topic. Setting it to:\n');                        
                odom_topic = topic
                break;
            end
        end
    end
end
odom_poses_select = select(bag_select, 'Topic', odom_topic);
vicon_poses_select = select(bag_select, 'Topic', gt_topic);

%% Extracting the data
fprintf('Extracting messages...\n')
% Creating a helper to help with data extraction
ros_data_helper = RosDataHelper();

% Reading the messages
odom_pose_messages = odom_poses_select.readMessages;
vicon_pose_messages = vicon_poses_select.readMessages;

% determine topic type
gt_type = char(vicon_poses_select.AvailableTopics{1,2});
odom_type = char(odom_poses_select.AvailableTopics{1,2});

%currently only Odometry, PoseStamped and PointStamped are supported
if strcmp(odom_type,'nav_msgs/Odometry')
    odom_poses = ros_data_helper.convertOdometryMessages(odom_pose_messages);
else
    if strcmp(odom_type,'geometry_msgs/PointStamped')
        odom_poses = ros_data_helper.convertPositionStampedMessages(odom_pose_messages);
    else
        if strcmp(odom_type,'geometry_msgs/PoseStamped')
            odom_poses = ros_data_helper.convertPoseStampedMessages(odom_pose_messages);
        end
    end
end

if strcmp(gt_type,'nav_msgs/Odometry')
    vicon_poses = ros_data_helper.convertOdometryMessages(vicon_pose_messages);
else
    if strcmp(gt_type,'geometry_msgs/PointStamped')
        vicon_poses = ros_data_helper.convertPositionStampedMessages(vicon_pose_messages);
    else
        if strcmp(odom_type,'geometry_msgs/PoseStamped')
            vicon_poses = ros_data_helper.convertPoseStampedMessages(vicon_pose_messages);
        end
    end
end

%% Calculating the alignment transform
fprintf('Calculating the alignment transform...\n')
% NOTE: Here I've decided to resample the faster datastream, the vicon
%       Vicon - fast
%       Odometry   - slow

% Creating an aligner
position_trajectory_aligner = PositionTrajectoryAligner6Dof();
% Resampling the data and maintaining the orientation information
pose_trajectory_for_resampling = PoseTrajectoryAligner6Dof();
[vicon_poses_resampled, odom_poses_resampled] =...
    pose_trajectory_for_resampling.truncateAndResampleDatastreams(vicon_poses,...
                                                               odom_poses);
% Calculating the alignment transform
T_alignment = position_trajectory_aligner.calculateAlignmentTransform(vicon_poses_resampled, odom_poses_resampled);
%r_align = regexprep(num2str(T_alignment.position),'\s+',',');
%q_align = regexprep(num2str(T_alignment.orientation_quat),'\s+',',');
r_align = T_alignment.position;
q_align = T_alignment.orientation_quat;
T_align = T_alignment.transformation_matrix;

% Transforming the trajectory by the alignment transform
odom_poses_aligned = odom_poses_resampled.applyStaticTransformLHS(T_alignment);

%% Compute the errors
% calculate and display the rms in x, y, z direction
rms_average=mean(odom_poses_aligned.getPositionTrajectory().rmsErrorTo(vicon_poses_resampled.getPositionTrajectory()));
q_errs = zeros(length(odom_poses_aligned.orientations),3);
for i=1:length(odom_poses_aligned.orientations)
    o1 = odom_poses_aligned.orientations(i,:);
    o2 = vicon_poses_resampled.orientations(i,:);
    q_errs(i,1) = vicon_poses_resampled.times(i);
    q_errs(i,2) = 1 - abs(dot(o1,o2));
    r1 = quatmultiply(quatconj(o1),o2);
    angle=abs(2*acos(r1(1)));
    if angle > pi
        angle = 2*pi - angle;
    end
    q_errs(i,3) = rad2deg(angle);
end

