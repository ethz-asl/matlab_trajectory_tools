function [alignment_data,gt_resampled,odom_aligned_resampled,q_errs] = align_trajectories_wrapper(bagfile, odom_topic, gt_topic)
%ALIGN_TRAJECTORIES_WRAPPER Summary of this function goes here
%   Detailed explanation goes here
[T,r,q,rms,odom,gt,q_errs]=align_trajectories_3d(bagfile,odom_topic,gt_topic);
alignment_data = { T; r; q; rms;};
odom_aligned_resampled = [odom.times odom.positions odom.orientations];
gt_resampled = [gt.times gt.positions gt.orientations];
end
