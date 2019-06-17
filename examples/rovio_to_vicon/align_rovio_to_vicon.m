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
[script_path, ~, ~] = fileparts(mfilename('fullpath'));
rosbag_path = strcat(script_path, '/2019-06-12-11-10-21.bag');
bag_select = rosbag(rosbag_path);

% Selecting the topics
rovio_position_select = select(bag_select, 'Topic', '/loon/rovio/transform');
vicon_position_select = select(bag_select, 'Topic', '/loon/loon/vrpn_client/estimated_transform');

% Reading the messages
rovio_position_messages = rovio_position_select.readMessages;
vicon_position_messages = vicon_position_select.readMessages;

%% Extracting the data

% Creating a helper to help with data extraction
ros_data_helper = RosDataHelper();

% Extracting the data to arrays
rovio_position = ros_data_helper.convertTransformStampedMessages(rovio_position_messages);
vicon_position = ros_data_helper.convertTransformStampedMessages(vicon_position_messages);

%% Converting to trajectory objects

% Shifting the times such that they start at zero
start_time = rovio_position.times(1);

% Constructing the transformation trajectory objects
gps_trajectory = PositionTrajectory(rovio_position.positions,...
                                    rovio_position.times - start_time);
leica_trajectory = PositionTrajectory(vicon_position.positions,...
                                      vicon_position.times - start_time);

%% Calculating the alignment transform 

% NOTE: Here I've decided to resample the fastest datastream, the leica
%       Leica - fast
%       GPS   - slow

% Creating an aligner
position_trajectory_aligner = PositionTrajectoryAligner4Dof();

% Resampling the data
[leica_position_resampled, gps_position_resampled] =...
    position_trajectory_aligner.truncateAndResampleDatastreams(vicon_position,...
                                                               rovio_position);
% Calculating the alignment transform
T_alignment = position_trajectory_aligner.calculateAlignmentTransform(leica_position_resampled,...
                                                                      gps_position_resampled);

%% Applying the alignment transform

% Transforming the trajectory by the alignment transform
gps_position_aligned = gps_trajectory.applyStaticTransformLHS(T_alignment);

%% Plotting

% 3D Positions Pre-Alignment
figure()
subplot(1,2,1)
gps_trajectory.plot()
hold on
leica_trajectory.plot();
hold off
axis equal
grid on
xlabel('x'); ylabel('y'); zlabel('z');
title('Trajectories Pre-Alignment')
legend('GPS Position', 'Leica Position')
% 3D Positions Post-Alignment
subplot(1,2,2)
gps_position_aligned.plot()
hold on
leica_trajectory.plot()
hold off
axis equal
grid on
xlabel('x'); ylabel('y'); zlabel('z');
title('Trajectories Post-Alignment')
legend('GPS Position', 'Leica Position')
