classdef PositionTrajectoryAligner6Dof < PositionTrajectoryAlignerBase
    %POSITIONTRAJECTORYALIGNERYAWONLY Contains functionality relating to a dense mapping system
   
    properties
    end
    
    methods
        
        % Constructor
        function obj = PositionTrajectoryAligner6Dof()
            % Calling the superclass constructor
            obj = obj@PositionTrajectoryAlignerBase();
        end

    end
    
    methods(Static)
        
        % Calculates an alignment transform from aligned position data
        function T_alignment = calculateAlignmentTransform(aligned_to_trajectory,...
                                                           aligned_from_trajectory,...
                                                           sigma_init)
            % Checks
            assert(aligned_to_trajectory.length() == aligned_from_trajectory.length(), 'Vectors to be aligned must have the same size.')
            % The initial value of the optimization variable if not passed
            if nargin < 3
                sigma_init = zeros(6,1);
            end
            % Assigning the data matrices in the correct form
            X = aligned_from_trajectory.positions()';
            Y = aligned_to_trajectory.positions()';
            % Parameterizing the cost functions with dataset
            f_residuals = @(sigma)PositionTrajectoryAligner6Dof.get_alignment_residuals( sigma, Y, X );
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
            w_x = sigma_star(1);
            w_y = sigma_star(2);
            w_z = sigma_star(3);
            t_x = sigma_star(4);
            t_y = sigma_star(5);
            t_z = sigma_star(6);
            % Reconstructing the transformation matrix
            R = PositionTrajectoryAligner6Dof.yprToRotationMatrix(w_z, w_y, w_x);
            t = [ t_x; t_y; t_z ]; 
            T_alignment_matrix = [ R             t ;
                                   zeros(1,3)    1 ; ];
            % Transformation object construction
            T_alignment = Transformation();
            T_alignment.initializeFromMatrix(T_alignment_matrix);
        end
        
        % Returns the alignment residuals for a given a value for the
        % parameter vector (sigma) and the data vectors (X, Y)
        function f_sigma = get_alignment_residuals( sigma, Y, X )
            % Data parameters
            n_points = size(X,2);
            % Extracting the parameters
            w_x = sigma(1);
            w_y = sigma(2);
            w_z = sigma(3);
            t_x = sigma(4);
            t_y = sigma(5);
            t_z = sigma(6);
            T = repmat([t_x;t_y;t_z],1,n_points);
            % Forming the rotation matrix
            R = PositionTrajectoryAligner6Dof.yprToRotationMatrix(w_z, w_y, w_x);            
            % Calculating the error
            f_sigma = sum((Y - R*X - T).^2,1)';
        end
        
        % Converts euler angles to a rotation matrix
        function R = yprToRotationMatrix(w_z, w_y, w_x)
            % Forming the rotation matrix
            R = [ 1         0           0;
                  0         cos(w_x)    sin(w_x);
                  0         -sin(w_x)   cos(w_x);] * ...
                [ cos(w_y)  0           -sin(w_y);
                  0         1           0;
                  sin(w_y)  0           cos(w_y);] * ...
                [ cos(w_z)  sin(w_z)    0       ;
                  -sin(w_z) cos(w_z)    0       ;
                  0           0         1       ;];
        end
        
    end
    
end