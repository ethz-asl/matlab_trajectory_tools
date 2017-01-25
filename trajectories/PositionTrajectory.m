classdef PositionTrajectory < handle & matlab.mixin.Copyable
    %TRAJECTORY Encapsulates trajectory functionality
    %   Detailed explanation goes here
    
    properties
        % Raw Data
        positions
        times
        % Properties
        length
    end
    
    methods
        
        % Constructor
        % Initializes the trajectory from a timeseries 
        function obj = PositionTrajectory(positions, times)
            % Setting data
            obj.setData(positions, times)
        end
        
        % Sets the data
        function setData(obj, positions, times)
            % Checks
            assert(size(positions,1) == size(times,1), 'Times and positions must have the same length');
            % Storing the data
            obj.positions = positions;
            obj.times = times;
            % Properties
            obj.length = size(times, 1);
        end
        
        % Gets the position at a time index
        function position = getPositionAtIndex(obj, index)
            position = obj.positions(index,:);
        end
        
        % Truncates the trajectory based on passed times
        function truncateToTimes(obj, start_time, end_time)
            % Finding the index bounds for truncation
            start_index = find(obj.times >= start_time, 1, 'first');
            end_index = find(obj.times <= end_time, 1, 'last');
            % Rewriting the data
            obj.truncateToIndicies(start_index, end_index);
        end
        
        % Truncates the trajectory to indicies
        function truncateToIndicies(obj, start_index, end_index)
            % Rewriting the data
            obj.setData(obj.positions(start_index:end_index, :),...
                        obj.times(start_index:end_index));
        end
        
        % Resamples the trajectory at the passed times
        function resample(obj, times)
            % Creating time series
            data_timeseries = timeseries(obj.positions, obj.times);
            % Resampling
            data_timeseries_resampled = data_timeseries.resample(times);
            % Updating the object
            obj.setData(data_timeseries_resampled.Data(), data_timeseries_resampled.Time());
        end
        
        % Gives the RMS error between this and a given trajectory
        function error = rmsErrorTo(obj, truth)
            % Checks
            assert(obj.length() == truth.length(), 'Trajectories different lengths. Consider using rmsErrorToWithResampling');
            % Calculating the rms
            error = rms(obj.positions - truth.positions());
        end
        
        % Gives the RMS error between this and a given trajectory
        % This function accepts trajectories of different sampling
        % frequencies and lengths
        function error = rmsErrorToWithResampling(obj, truth)
            % TODO(alexmillane): At the moment we assume truth is at a
            % higher rate. Remove this assumption
            % Resampling this trajectory
            truth_copy = truth.copy();
            obj_copy = obj.copy();
            obj_copy.truncateToTimes(truth_copy.times(1), truth_copy.times(end))
            truth_copy.resample(obj_copy.times());
            % Calculating the rms
            error = rms(obj_copy.positions() - truth_copy.positions());
        end
        
        % Returns the error trajectory between this and a give trajectory
        function error_trajectory = errorTrajectoryTo(obj, truth)
            % Checks
            assert(obj.length() == truth.length(), 'Trajectories different lengths. CONSIDER WRITING FUNCTION WITH RESAMPLING');
            % Calculating the error trajectory
            error_trajectory = sqrt(sum((obj.positions() - truth.positions()).^2, 2));
        end
        
        % Returns an array of the norms of the trajectory vectors
        function norms = getNorms(obj)
            norms = zeros(obj.length(), 1);
            for i = 1:obj.length
                norms(i) = norm(obj.positions(i,:));
            end
        end
        
        % Plots the trajectory
        function h = plot(obj, symbol)
            if nargin < 2
                symbol = '';
            end
            holdstate = ishold;
            hold on
            h_temp = plot3( obj.positions(:,1),...
                            obj.positions(:,2),...
                            obj.positions(:,3),...
                            symbol);
            plot3( obj.positions(1,1),...
                   obj.positions(1,2),...
                   obj.positions(1,3), 'go');
            plot3( obj.positions(end,1),...
                   obj.positions(end,2),...
                   obj.positions(end,3), 'ro');
            if nargout > 0 
                h = h_temp;
            end
            if ~holdstate
              hold off
            end
        end
        
        % Plots the trajectory sample times
        function h = plotSampleTimes(obj, color)
            if nargin < 2
                color = 'b';
            end
            holdstate = ishold;
            hold on
            h_temp = plot([  obj.times               obj.times]',...
                          [  zeros(obj.length(),1)   ones(obj.length(),1)]',...
                             color);
            h_temp = h_temp(1);
            if nargout > 0 
                h = h_temp;
            end
            if ~holdstate
              hold off
            end
        end
        
        % Plots the trajectory sample times
        function h = plot3Axis(obj, symbol)
            if nargin < 2
                symbol = '';
            end
            holdstate = ishold;
            hold on
            subplot(3,1,1)
            plot(obj.times, obj.positions(:,1), symbol)
            xlabel('Time (s)'); ylabel('X (m)');
            hold on
            subplot(3,1,2)
            plot(obj.times, obj.positions(:,2), symbol)
            xlabel('Time (s)'); ylabel('Y (m)');
            hold on
            subplot(3,1,3)
            plot(obj.times, obj.positions(:,3), symbol)
            xlabel('Time (s)'); ylabel('Z (m)');
            
            if nargout > 0 
                h = h_temp;
            end
            if ~holdstate
              hold off
            end
        end
        
        % Plots the trajectory in XY plan
        function h = plotXY(obj, symbol)
            if nargin < 2
                symbol = '';
            end
            holdstate = ishold;
            hold on
            h_temp = plot( obj.positions(:,1),...
                           obj.positions(:,2),...
                           symbol);
            plot( obj.positions(1,1),...
                  obj.positions(1,2), 'go');
            plot( obj.positions(end,1),...
                  obj.positions(end,2), 'ro');
            if nargout > 0 
                h = h_temp;
            end
            if ~holdstate
              hold off
            end
        end
                        
    end
    
end

