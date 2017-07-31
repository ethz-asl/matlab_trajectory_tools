classdef PositionTrajectoryAligner7Dof < PositionTrajectoryAlignerBase
    %POSITIONTRAJECTORYALIGNERYAWONLY Contains functionality relating to a dense mapping system
   
    properties
    end
    
    methods
        
        % Constructor
        function obj = PositionTrajectoryAligner7Dof()
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
                sigma_init = [zeros(6,1) ; 1];
            end
            % Assigning the data matrices in the correct form
            X = aligned_from_trajectory.positions()';
            Y = aligned_to_trajectory.positions()';
            % Using an external lib to do the alignment with scaling.
            regParams = absor(X,Y, 'doScale', 1, 'doTrans', 1);
            % Reconstructing the transformation matrix
            R = regParams.R;
            t = regParams.t / regParams.s;
            scale = regParams.s;
            T_alignment_matrix = [ R             t ;
                                   zeros(1,3)    1/scale ; ];
            % Transformation object construction
            T_alignment = SimTransformation();
            T_alignment.initializeFromMatrix(T_alignment_matrix);
        end

            
%             % Parameterizing the cost functions with dataset
%             f_residuals = @(sigma)PositionTrajectoryAligner7Dof.get_alignment_residuals( sigma, Y, X );
%             % Setting up the opimization options
%             opt_TolFun = 1e-12;
%             % opt_MaxFunEvals = 4*5000;
%             % opt_MaxIter = 1000;
%             opt_Display = 'iter'; %'off' %'final' %'iter'
%             % options = optimoptions('lsqnonlin', 'TolFun', opt_TolFun, 'Display',...
%             %                        opt_Display, 'MaxFunEvals', opt_MaxFunEvals,...
%             %                        'MaxIter', opt_MaxIter);
%             options = optimoptions('lsqnonlin', 'TolFun', opt_TolFun, 'Display',...
%                                    opt_Display);
%             % Performing the optimization
%             sigma_star = lsqnonlin(f_residuals, sigma_init, [], [], options);
%             % Calculating the intial and final costs
%             f_init = norm(f_residuals(sigma_init),2)^2;
%             f_star = norm(f_residuals(sigma_star),2)^2;
%             fprintf('Optimization initial cost: %0.2e\n', f_init);
%             fprintf('Optimization final cost: %0.2e\n', f_star);
%             % Extracting the individual elements of the optimization variable
%             w_x = sigma_star(1);
%             w_y = sigma_star(2);
%             w_z = sigma_star(3);
%             t_x = sigma_star(4);
%             t_y = sigma_star(5);
%             t_z = sigma_star(6);
%             scale = sigma_star(7);
%             % Reconstructing the transformation matrix
%             R = PositionTrajectoryAligner7Dof.yprToRotationMatrix(w_z, w_y, w_x);
%             t = [ t_x; t_y; t_z ]; 
%             T_alignment_matrix = [ R             t ;
%                                    zeros(1,3)    1/scale ; ];
%             % Transformation object construction
%             T_alignment = SimTransformation();
%             T_alignment.initializeFromMatrix(T_alignment_matrix);
%         end
        
%         % Returns the alignment residuals for a given a value for the
%         % parameter vector (sigma) and the data vectors (X, Y)
%         function f_sigma = get_alignment_residuals( sigma, Y, X )
%             % Data parameters
%             n_points = size(X,2);
%             % Extracting the parameters
%             w_x = sigma(1);
%             w_y = sigma(2);
%             w_z = sigma(3);
%             t_x = sigma(4);
%             t_y = sigma(5);
%             t_z = sigma(6);
%             scale = sigma(7);
%             T = repmat([t_x;t_y;t_z],1,n_points);
%             % Forming the rotation matrix
%             R = PositionTrajectoryAligner7Dof.yprToRotationMatrix(w_z, w_y, w_x);            
%             % Calculating the error
%             f_sigma = sum((Y - (R*X + T)*scale).^2,1)';
%         end
        
    end
    
end