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

