function [x, y, z] = get_xyz_expressions(dh_parameters, verbose)
   % get_xyz_expressions Applies homogeneous transforms to DH parameters to compute the transformation matrix from the base to the end-effector.
    %
    % This function calculates the transformation matrix from the base of the robot to the end-effector, based on the DH parameters. 
    % It returns symbolic expressions for x, y, and z positions of the end-effector.
    %
    % Parameters:
    %   dh_parameters : A symbolic matrix that represents the DH parameters of the robot.
    %   
    %   verbose : A boolean flag that, if true, enables verbose logging of the final transformation matrix and position expressions.
    %
    % Returns:
    %   [x, y, z] : Symbolic expressions for the x, y, and z positions of the end-effector.
    %

    % Calculate transformation matrix from base to end-effector
    T0_ee = eye(4);
    % Apply each transformation
    for i = 1:size(dh_parameters, 1)
        T{i} = get_DH_matrix(dh_parameters(i,1), ...
            dh_parameters(i,2), ...
            dh_parameters(i,3), ...
            dh_parameters(i,4));
        T0_ee = T0_ee*T{i};
    end

    x = T0_ee(1,4);
    y = T0_ee(2,4);
    z = T0_ee(3,4);

    if verbose
        disp('final transformation matrix from base to end-effector:')
        disp(T0_ee)
        disp('expressions for position:')
        fprintf('x = %s\n', char(x))
        fprintf('y = %s\n', char(y))
        fprintf('z = %s\n', char(z))
    end

end
