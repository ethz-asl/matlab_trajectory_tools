classdef PositionTrajectoryWithCovariance < PositionTrajectory3D
    %TRAJECTORY Encapsulates trajectory functionality
    %   Detailed explanation goes here
    
    properties
        covariance
    end
    
    methods
        
        % Constructor
        % Initializes the trajectory from a timeseries 
        function obj = PositionTrajectoryWithCovariance(data, covariance)
            % Calling the superclass constructor
            obj = obj@PositionTrajectory3D(data);
            % Saving the data
            obj.covariance = covariance;
        end
        
        % Returns the variance trajectory
        function std_trajectory = getStdTrajectory(obj)
            std_trajectory = [sqrt(squeeze(obj.covariance(:,1,1))),...
                              sqrt(squeeze(obj.covariance(:,2,2))),...
                              sqrt(squeeze(obj.covariance(:,3,3)))];
        end

        % Plots the trajectory sample times
        function h = plot3AxisWithCovariance(obj, num_std, symbol)
            if nargin < 3
                symbol = '';
            end
            holdstate = ishold;
            hold on

            % Plotting the data
            obj.plot3Axis(symbol);
            % Constructing the std lines
            std_trajectory = obj.getStdTrajectory();
            plus_std = obj.data.Data + num_std * std_trajectory;
            minus_std = obj.data.Data - num_std * std_trajectory;
            % Potting the std lines
            hold on
            subplot(3,1,1)
            plot(obj.data.Time, plus_std(:,1), '--')
            plot(obj.data.Time, minus_std(:,1), '--')
            hold on
            subplot(3,1,2)
            plot(obj.data.Time, plus_std(:,2), '--')
            plot(obj.data.Time, minus_std(:,2), '--')
            hold on
            subplot(3,1,3)
            plot(obj.data.Time, plus_std(:,3), '--')
            plot(obj.data.Time, minus_std(:,3), '--')
            
            if nargout > 0 
                h = h_temp;
            end
            if ~holdstate
              hold off
            end
        end
                        
    end
    
end

