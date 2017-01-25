classdef PoseTrajectoryAligner6Dof < PoseTrajectoryAlignerBase
    %POSETRAJECTORYALIGNER6DOF Contains functionality relating to a dense mapping system
   
    properties
    end
    
    methods
        
        % Constructor
        function obj = PoseTrajectoryAligner6Dof()
            % Calling the superclass constructor
            obj = obj@PoseTrajectoryAlignerBase();
        end

    end
    
    methods(Static)
        
        % Calculates an alignment transform from aligned position data
        function T_alignment = calculateAlignmentTransform(aligned_to_trajectory,...
                                                           aligned_from_trajectory,...
                                                           orientation_scaling,...
                                                           sigma_init)
            % Checks
            assert(aligned_to_trajectory.length() == aligned_from_trajectory.length(), 'Vectors to be aligned must have the same size.')
            % The initial value of the optimization variable if not passed
            if nargin < 4
                sigma_init = zeros(6,1);
            end
            % Assigning the data matrices in the correct form
            X = [aligned_from_trajectory.positions aligned_from_trajectory.orientations]';
            Y = [aligned_to_trajectory.positions aligned_to_trajectory.orientations]';
            % Parameterizing the cost functions with dataset
            f_residuals = @(sigma)PoseTrajectoryAligner6Dof.get_alignment_residuals( sigma, Y, X, orientation_scaling);
            % Setting up the opimization options
            opt_TolFun = 1e-3;
            % opt_MaxFunEvals = 4*5000;
            % opt_MaxIter = 1000;
            opt_Display = 'final'; %'off' %'final'
            % options = optimoptions('lsqnonlin', 'TolFun', opt_TolFun, 'Display',...
            %                        opt_Display, 'MaxFunEvals', opt_MaxFunEvals,...
            %                        'MaxIter', opt_MaxIter);
            options = optimoptions('lsqnonlin', 'TolFun', opt_TolFun, 'Display',...
                                   opt_Display);
            % Performing the optimization
            sigma_star = lsqnonlin(f_residuals, sigma_init, [], [], options);
            % Calculating the intial and final costs
            f_init = norm(f_residuals(sigma_init),2)^2;
            f_star = norm(f_residuals(sigma_star),2)^2;
            fprintf('Optimization initial cost: %0.2e\n', f_init);
            fprintf('Optimization final cost: %0.2e\n', f_star);
            % Extracting the individual elements of the optimization variable
            w_r = sigma_star(1);
            w_p = sigma_star(2);
            w_y = sigma_star(3);
            t_x = sigma_star(4);
            t_y = sigma_star(5);
            t_z = sigma_star(6);
            % The transform encoded by the optimization variables
            q = ypr2quat(w_y, w_p, w_r)';
            t = [t_x, t_y, t_z];
            T_alignment = Transformation(q, t);
        end
        
        % Returns the alignment residuals for a given a value for the
        % parameter vector (sigma) and the data vectors (X, Y)
        function f_sigma = get_alignment_residuals( sigma, Y, X, orientation_scaling )
            % Data parameters
            n_points = size(X,2);
            % Extracting the data parts
            X_t = X(1:3,:);
            Y_t = Y(1:3,:);
            X_q = X(4:7,:);
            Y_q = Y(4:7,:);
            % Extracting the parameters
            w_r = sigma(1);
            w_p = sigma(2);
            w_y = sigma(3);
            t_x = sigma(4);
            t_y = sigma(5);
            t_z = sigma(6);
            % The transform encoded by the optimization variables
            q = ypr2quat(w_y, w_p, w_r)';
            t = [t_x, t_y, t_z];
            T_sigma = Transformation(q, t);
            % Constructing transform trajectories trajectory
            dummy_time = (1:n_points)';
            X_trajectory = TransformationTrajectory(X_q', X_t', dummy_time);
            Y_trajectory = TransformationTrajectory(Y_q', Y_t', dummy_time);
            % Applying the current alignment transform
            X_trajectory_trans = X_trajectory.applyStaticTransformLHS(T_sigma);
            % Error trajectories
            error_trajectory_pos = Y_trajectory.positions() - X_trajectory_trans.positions();
            error_trajectory_quat = Y_trajectory.getOrientationTrajectory().compose(...
                X_trajectory_trans.getOrientationTrajectory().inverse());
            %  Residuals
            position_error = sum(error_trajectory_pos.^2,2);
            orientation_error = sum(error_trajectory_quat.orientations(:,2:4).^2,2);
            f_sigma = position_error + (orientation_scaling * orientation_error);
        end
    end
    
end

