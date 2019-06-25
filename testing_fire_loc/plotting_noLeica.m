%% Initialization

clc ;
clear ;

%% Loading the data

% Getting the datapath (UAV paths)
[script_path, ~, ~] = fileparts(mfilename('fullpath'));
rosbag_path = strcat(script_path, '/tagLanding1Result.bag');
bag_trajectories = rosbag(rosbag_path);

% Getting the datapath (fire estimates)
[script_path, ~, ~] = fileparts(mfilename('fullpath'));
rosbag_path = strcat(script_path, '/tagLanding1Result.bag');
bag_estimates = rosbag(rosbag_path);

% Selecting the topics
fire_estimate = select(bag_estimates, 'Topic', '/fire_points_debug');
rovio_position_select = select(bag_trajectories, 'Topic', '/loon/rovio/transform');

% Reading the messages
fire_estimate_msgs = fire_estimate.readMessages;
rovio_transform_messages = rovio_position_select.readMessages;

%% Extracting the data

% Creating a helper to help with data extraction
ros_data_helper = RosDataHelper();

% Extracting the data to arrays
fire_est_array = ros_data_helper.convertPositionStampedMessages(fire_estimate_msgs);
rovio_transform = ros_data_helper.convertTransformStampedMessages(rovio_transform_messages);
fire_est_array = fire_est_array.positions;
fire_est_array = [fire_est_array, ones(size(fire_est_array, 1), 1)]';

%% Converting to trajectory objects

% Shifting the times such that they start at zero
start_time = rovio_transform.times(1);

% Constructing the transformation trajectory objects
rovio_trajectory = PositionTrajectory(rovio_transform.positions,...
                                    rovio_transform.times - start_time);
% get covariance matrix
cov_mat = cov(fire_est_array(1,:), fire_est_array(2,:));

% get RMSE between estimates and estimate mean
numofestimates = size(fire_est_array);
mean_vector = zeros(numofestimates(2),2);
mean_vector(:,1) = mean(fire_est_array(1,:));
mean_vector(:,2) = mean(fire_est_array(2,:));
squared_error = (fire_est_array(1:2,:)'-mean_vector).^2;
squared_error = squared_error(:,1) + squared_error(:,2);
RMSE_to_mean = sqrt(mean(squared_error));

%% Plotting

% 3D Positions Pre-Alignment
figure()
scatter3(fire_est_array(1,:)', fire_est_array(2,:)', fire_est_array(3,:)',...
    15, [1,0,0], 'filled');
hold on
plot3(mean(fire_est_array(1,:)), mean(fire_est_array(2,:)), mean(fire_est_array(3,:)), 'bx', ...
    'MarkerSize',20,'LineWidth',4);

[V D] = eig(cov_mat);     %#' cov(X0)
[D order] = sort(diag(D), 'descend');
D = diag(D);
V = V(:, order);
t = linspace(0,2*pi,100);
e = [cos(t) ; sin(t)];        %# unit circle
VV = V*sqrt(D);               %# scale eigenvectors
e = bsxfun(@plus, VV*e, [mean(fire_est_array(1,:)) mean(fire_est_array(2,:))]'); %#' project circle back to orig space
%plot(e(1,:), e(2,:), 'Color','b','LineWidth',3);


% xticks([-9 -5 -2 0 2 5 9]) % set grid
% xticklabels({'-3\pi','-2\pi','-\pi','0','\pi','2\pi','3\pi'}) % name grid
% yticks([-5 -2 -1 0 1 2 5])

rovio_trajectory.plot('r');

std_dev_x = num2str(RMSE_to_mean, 2);
RMSE_to_mean_str = num2str(RMSE_to_mean, 2);
RMSE_to_mean_str =  "RMSE (estimates to mean estimates):        " + RMSE_to_mean_str + " m";
text = RMSE_to_mean_str;

%a = annotation('textbox', [0.04, 0.15, 0, 0], 'string', text, 'FitBoxToText','on', 'LineStyle', 'none');
%a.FontSize = 17;

hold off
axis equal
% xlim([-3 5]) %set axis limits
% ylim([-10 -4]) 
% zlim([0 2]) 
grid on
xlabel('x [m]', 'FontSize',15); ylabel('y [m]', 'FontSize',15); zlabel('z [m]', 'FontSize',15);
title('Fire Estimation Flight', 'FontSize',22)
legend('Fire Estimates', 'Fire Estimates Mean',...
    'Rovio Trajectory', 'Start Position', 'End Position', ...
    'FontSize',17);

