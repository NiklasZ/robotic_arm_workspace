function plot2dworkspace(dh_parameters, parameter_ranges, unused_position_threshold, verbose)
    % plot2dworkspace Plots the reachable 2D workspace of a robot manipulator based on its Denavit-Hartenberg (DH) parameters.
    % 
    % This function uses the provided DH parameters and ranges to calculate and plot the reachable 2D workspace of a robot manipulator.
    % The function first validates the inputs and computes x, y, and z expressions. It then calculates the positions from the expressions 
    % and plots the workspace. Note that this is a numerical method that works by applying all possible parameter combinations provided in
    % parameter_ranges. See example files on 2D robots for more details.
    %
    % Parameters:
    %   dh_parameters : An n x 4 matrix of DH parameters (real values and symbols) for n joints. The column order is
    %   (a, alpha, d, theta)
    %   
    %   parameter_ranges : A containers.Map that represents the range of each symbolic DH parameter.
    %
    %   unused_position_threshold : (Optional) A threshold used to determine if a position dimension (x, y, or z) is near 0 and thus unused in a 2D workspace.
    %                              Default is 0.0001.
    %   
    %   verbose : (Optional) A boolean flag that, if true, enables verbose logging of calculation details such as the transformation matrix.
    %             Default is false.
    %
    % Throws:
    %   An error if all positions on at least one axis are not near 0 (as per the unused_position_threshold) - i.e. if the workspace does not seem to be 2D.
    %
    % Example:
    %   plot2dworkspace(dh_parameters, parameter_ranges)
    %   plot2dworkspace(dh_parameters, parameter_ranges, 0.001, true)
    %

    arguments
        dh_parameters
        parameter_ranges
        unused_position_threshold = 0.0001
        verbose = false % Whether to log various calculation details like the transformation matrix and more.
    end

    validate_inputs(dh_parameters, parameter_ranges);
    [x,y,z] = get_xyz_expressions(dh_parameters, verbose);
    

    % Get parameter grid
    if verbose
        ranges = values(parameter_ranges);        
        lengths = cellfun(@length, ranges);        
        product_of_lengths = prod(lengths);
        fprintf('allocating %d possible DH parameter combinations...\n', product_of_lengths);
    end

    keys = parameter_ranges.keys;
    param_values = parameter_ranges.values(keys);
    [param_grid{1:numel(param_values)}] = ndgrid(param_values{:});
    
    % Reshape into value arrays in map
    param_value_map = containers.Map;
    for i = 1:numel(param_grid)
        param_value_map(keys{i}) = reshape(param_grid{i}, 1, []);
        param_grid{i} = 0;
    end
    

    if verbose
        disp('calculating positions from expressions...');
    end
    % Calculate x,y,z positions
    positions = cell(3,1);
    expressions = {x,y,z};
    for i = 1:3
        % Get variables from expression
        expr = expressions{i};
        sym_cell = symvar(expr);
        keys_cell = arrayfun(@char, sym_cell, 'UniformOutput', false);
        % Convert from symbolic to numerical functions for faster evaluation
        pos_func = matlabFunction(expr, 'Vars', sym_cell);
        func_params = values(param_value_map, keys_cell);
        positions{i} = pos_func(func_params{:});
    end

    % Determine the largest absolute value in each cell
    minValues = cellfun(@(x) max(abs(x(:))), positions);
    axis_labels = {'X','Y','Z'};
    
    % Get the smallest value from all cells
    [~, min_idx] = min(minValues);

    if minValues(min_idx) > unused_position_threshold
        error(['With a 2D workspace, it is expected that one of the X,Y,Z positions' ...
            ' is always near 0 (< %d). Instead the smallest candidate %s has a max position magnitude of' ...
            '%d'], unused_position_threshold, axis_labels{min_idx}, minValues(min_idx))
    end

    % Plot results
    chosen_indices = true(size(axis_labels));
    chosen_indices(min_idx) = false;
    chosen_positions = positions(chosen_indices);
    chosen_axes = axis_labels(chosen_indices);

    figure(1)
    plot(chosen_positions{:},'.');
    title('Reachable Workspace')
    xlabel(chosen_axes{1}) 
    ylabel(chosen_axes{2}) 
    axis equal
    grid on
end