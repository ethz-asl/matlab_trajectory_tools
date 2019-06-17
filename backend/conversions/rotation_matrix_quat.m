%% depth_cam_calibration (script)
%
% Script for generating depth camera calibration quaternion.
%
% --
%
% (c)   May 2019 by
%       Alexander Millane
%       ETH Zurich, ASL
%       alexander.millane(at)mavt.ethz.ch
%     
%% Initialization

clc ;
clear ;
close all;

% Adding sdf pathz
addpath(genpath('/home/millanea/trunk/matlab_trajectory_tools/'))

%% Doing stuff

% The Matrix from Kalibr T_imu_depth
T_matrix = [0.9995665581797498, -0.029402508151407658, -0.0014792846517102623, 0.017974474303098634;
            0.02939908084366632, 0.9995651359237321, -0.002287595091834257, 2.1608286808458033e-05;
            0.001545902397291447, 0.0022431139433872482, 0.9999962893059157, 0.0022638130496824235;
            0.0, 0.0, 0.0, 1.0];

% Creating the Transform
T = Transformation();
T.initializeFromMatrix(T_matrix)

%t = T.position
%q = T.orientation_quat

% The inverse transform
T_inv = T.inverse();
t = T_inv.position
q = T_inv.orientation_quat
