%% align_trajectories (script)
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
%% Initialization

clc ;
clear ;

%% Loading the data

% Getting the datapath (relative load)
bag_select = rosbag('/home/nico/2018-12-03/easy-2018-12-03-17-49-09_rovio_fused_reordered.bag');

% Selecting the topics
odom_poses_select = select(bag_select, 'Topic', '/rovio/odometry');
vicon_poses_select = select(bag_select, 'Topic', '/lidar_stick/vrpn_client/estimated_odometry');

% Reading the messages
odom_pose_messages = odom_poses_select.readMessages;
vicon_pose_messages = vicon_poses_select.readMessages;

%% Extracting the data

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
alignment_offset = regexprep(num2str(T_alignment.position),'\s+',',')
alignment_orientation_q = regexprep(num2str(T_alignment.orientation_quat),'\s+',',')

%% Applying the alignment transform

% Transforming the trajectory by the alignment transform
odom_poses_aligned = odom_poses_resampled.applyStaticTransformLHS(T_alignment);

% calculate and display the rms in x, y, z direction
position_rms=mean(odom_poses_aligned.getPositionTrajectory().rmsErrorTo(vicon_poses_resampled.getPositionTrajectory()))
%% Plotting

% 3D Positions Pre-Alignment
figure()
ax1=subplot(1,2,1);
ax1.ColorOrderIndex=3;
odom_trajectory.getPositionTrajectory().plot()
hold on
ax1.ColorOrderIndex=5;
vicon_trajectory.getPositionTrajectory().plot()
hold off
axis equal
grid on
xlabel('x'); ylabel('y'); zlabel('z');
title('Trajectories Pre-Alignment')
legend('Odom Position', 'Start Points', 'End Points', 'Vicon Positions')
pbaspect([1 1 1])
view(3)
% 3D Positions Post-Alignment
ax2=subplot(1,2,2);
ax2.ColorOrderIndex=3;
odom_poses_aligned.getPositionTrajectory().plot()
hold on
ax2.ColorOrderIndex=5;
vicon_trajectory.getPositionTrajectory().plot()
hold off
axis equal
grid on
xlabel('x'); ylabel('y'); zlabel('z');
title('Trajectories Post-Alignment')
legend('Odom Position', 'Start Points', 'End Points', 'Vicon Positions')
pbaspect([1 1 1])
view(3)

%% Positions post-alignment on individual axes
figure('Name', strcat('ROVIO on the easy dataset - RMS:', num2str(mean(position_rms),3)));
opa=odom_poses_aligned.getPositionTrajectory();
vpr=vicon_poses_resampled.getPositionTrajectory();

ax11=subplot(3,1,1);
ax11.ColorOrderIndex=3;
plot(opa.times,opa.positions(:,1));
hold on
ax11.ColorOrderIndex=5;
plot(vpr.times,vpr.positions(:,1));
hold off
xlabel('ROS Time'); ylabel('x');
title('X')
legend('Odom Positions', 'Vicon Positions')

ax12=subplot(3,1,2);
ax12.ColorOrderIndex=4;
plot(opa.times,opa.positions(:,2));
hold on
ax12.ColorOrderIndex=5;
plot(vpr.times,vpr.positions(:,2));
hold off
xlabel('ROS Time'); ylabel('y');
title('Y');
legend('Odom Positions', 'Vicon Positions');

ax13=subplot(3,1,3);
ax13.ColorOrderIndex=4;
plot(opa.times,opa.positions(:,3));
hold on
ax13.ColorOrderIndex=5;
plot(vpr.times,vpr.positions(:,3));
hold off
xlabel('ROS Time'); ylabel('z');
title('Z')
legend('Odom Positions', 'Vicon Positions')
