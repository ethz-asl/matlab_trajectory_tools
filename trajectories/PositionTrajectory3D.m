classdef PositionTrajectory3D < handle & matlab.mixin.Copyable
    %TRAJECTORY Encapsulates trajectory functionality
    %   Detailed explanation goes here
    
    properties
        data
        length
    end
    
    methods
        
        % Constructor
        % Initializes the trajectory from a timeseries 
        function obj = PositionTrajectory3D(data)
            obj.data = data;
            obj.length = size(data.Time, 1);
        end
        
        % Sets the data
        function setData(obj, data)
            obj.data = data;
            obj.length = size(data.Time, 1);
        end
        
        % Gets the position at a time index
        function position = getPositionAtIndex(obj, index)
            position = obj.data.Data(index,:);
        end
        
        % Truncates the trajectory based on passed times
        function truncate(obj, start_time, end_time)
            % Finding the index bounds for truncation
            start_index = find(obj.data.Time >= start_time, 1, 'first');
            end_index = find(obj.data.Time <= end_time, 1, 'last');
            % Rewriting the data
            obj.truncateToIndicies(start_index, end_index);
        end
        
        % Truncates the trajectory to indicies
        function truncateToIndicies(obj, start_index, end_index)
            % Rewriting the data
            obj.setData(timeseries(obj.data.Data(start_index:end_index, :),...
                                   obj.data.Time(start_index:end_index)));
        end
        
        % Resamples the trajectory at the passed times
        function resample(obj, times)
            obj.setData(obj.data.resample(times));
        end
        
        % Gives the RMS error between this and a given trajectory
        function error = rmsErrorTo(obj, truth)
            % Checks
            assert(obj.length() == truth.length(), 'Trajectories different lengths. Consider using rmsErrorToWithResampling');
            % Calculating the rms
            error = rms(obj.data().Data() - truth.data().Data());
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
            obj_copy.truncate(truth_copy.data().Time(1), truth_copy.data().Time(end))
            truth_copy.resample(obj_copy.data().Time());
            % Calculating the rms
            error = rms(obj_copy.data().Data() - truth_copy.data().Data());
        end
        
        % Returns the error trajectory between this and a give trajectory
        function error_trajectory = errorTrajectoryTo(obj, truth)
            % Checks
            assert(obj.length() == truth.length(), 'Trajectories different lengths. CONSIDER WRITING FUNCTION WITH RESAMPLING');
            % Calculating the error trajectory
            error_trajectory = sqrt(sum((obj.data().Data() - truth.data().Data()).^2, 2));
        end
        
        % Returns an array of the norms of the trajectory vectors
        function norms = getNorms(obj)
            norms = zeros(obj.length, 1);
            for i = 1:obj.length
                norms(i) = norm(obj.data.Data(i,:));
            end
        end
        
        % Plots the trajectory
        function h = plot(obj, symbol)
            if nargin < 2
                symbol = '';
            end
            holdstate = ishold;
            hold on
            h_temp = plot3( obj.data.Data(:,1),...
                            obj.data.Data(:,2),...
                            obj.data.Data(:,3),...
                            symbol);
            plot3( obj.data.Data(1,1),...
                   obj.data.Data(1,2),...
                   obj.data.Data(1,3), 'go');
            plot3( obj.data.Data(end,1),...
                   obj.data.Data(end,2),...
                   obj.data.Data(end,3), 'ro');
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
            h_temp = plot([  obj.data.Time           obj.data.Time]',...
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
            plot(obj.data.Time, obj.data.Data(:,1), symbol)
            xlabel('Time (s)'); ylabel('X (m)');
            hold on
            subplot(3,1,2)
            plot(obj.data.Time, obj.data.Data(:,2), symbol)
            xlabel('Time (s)'); ylabel('Y (m)');
            hold on
            subplot(3,1,3)
            plot(obj.data.Time, obj.data.Data(:,3), symbol)
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
            h_temp = plot( obj.data.Data(:,1),...
                           obj.data.Data(:,2),...
                           symbol);
            plot( obj.data.Data(1,1),...
                  obj.data.Data(1,2), 'go');
            plot( obj.data.Data(end,1),...
                  obj.data.Data(end,2), 'ro');
            if nargout > 0 
                h = h_temp;
            end
            if ~holdstate
              hold off
            end
        end
                        
    end
    
end

