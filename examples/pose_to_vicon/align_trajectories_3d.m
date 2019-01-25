function [r_align, q_align, rms_average] = align_trajectories_3d(bagfile, odom_topic, gt_topic)
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
%% Loading the data
fprintf('Loading the bagfile...\n')
bag_select = rosbag(bagfile);

% Selecting the topics
odom_poses_select = select(bag_select, 'Topic', odom_topic);
vicon_poses_select = select(bag_select, 'Topic', gt_topic);

% Reading the messages
odom_pose_messages = odom_poses_select.readMessages;
vicon_pose_messages = vicon_poses_select.readMessages;

%% Extracting the data
fprintf('Extracting messages...\n')
% Creating a helper to help with data extraction
ros_data_helper = RosDataHelper();

% Extracting the data to arrays
odom_poses = ros_data_helper.convertOdometryMessages(odom_pose_messages);
vicon_poses = ros_data_helper.convertOdometryMessages(vicon_pose_messages);

%% Converting to trajectory objects

% Shifting the times such that they start at zero
start_time = vicon_poses.times(1);

% Constructing the transformation trajectory objects
odom_trajectory = TransformationTrajectory(odom_poses.orientations, odom_poses.positions,...
                                    odom_poses.times - start_time);
vicon_trajectory = TransformationTrajectory(vicon_poses.orientations, vicon_poses.positions,...
                                      vicon_poses.times - start_time);

%% Calculating the alignment transform 
fprintf('Calculating the alignment transform...\n')
% NOTE: Here I've decided to resample the faster datastream, the vicon
%       Vicon - fast
%       Odometry   - slow

% Creating an aligner
position_trajectory_aligner = PoseTrajectoryAligner6Dof();
% Resampling the data
[vicon_poses_resampled, odom_poses_resampled] =...
    position_trajectory_aligner.truncateAndResampleDatastreams(vicon_poses,...
                                                               odom_poses);
% Calculating the alignment transform
orientation_scaling=1.0;
T_alignment = position_trajectory_aligner.calculateAlignmentTransform(vicon_poses_resampled,...
                                                                      odom_poses_resampled, orientation_scaling);
r_align = regexprep(num2str(T_alignment.position),'\s+',',');
q_align = regexprep(num2str(T_alignment.orientation_quat),'\s+',',');

%% Applying the alignment transform and compute the error

% Transforming the trajectory by the alignment transform
odom_poses_aligned = odom_poses_resampled.applyStaticTransformLHS(T_alignment);

% calculate and display the rms in x, y, z direction
rms_average=mean(odom_poses_aligned.getPositionTrajectory().rmsErrorTo(vicon_poses_resampled.getPositionTrajectory()));
fprintf('\n\nAlignment finished.\n')
