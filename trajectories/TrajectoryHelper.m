classdef TrajectoryHelper < handle & matlab.mixin.Copyable
    %TRAJECTORY Encapsulates trajectory functionality
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        
        % Constructor
        % Initializes the trajectory from a timeseries 
        function obj = TrajectoryHelper()
            % Empty
        end
        
    end
    
    methods(Static)

        function truncateToMinTimes(trajectory_1, trajectory_2)
            % Getting the times
            start_time = max( trajectory_1.data().Time(1), trajectory_2.data().Time(1));
            end_time = min( trajectory_1.data().Time(end), trajectory_2.data().Time(end));
            % Performing truncation
            trajectory_1.truncate(start_time, end_time);
            trajectory_2.truncate(start_time, end_time);
        end
        
        function truncateToMinTimesAndResample(trajectory_1, trajectory_2)
            % Getting the times
            start_index = find(trajectory_1.data().Time() > trajectory_2.data().Time(1), 1, 'first');
            end_index = find(trajectory_1.data().Time() < trajectory_2.data().Time(end), 1, 'last');
            % Truncate to indicies
            trajectory_1.truncateToIndicies(start_index, end_index);
            % Resampling the second trajectory
            trajectory_2.resample(trajectory_1.data().Time());
        end
                        
    end
    
end

