function [environment, bounds] = main()
    % Defining the size of the 3D environment
    % size along the x, y and z axis
    bounds.x = 250; 
    bounds.y = 200; 
    bounds.z = 100; % This is like the height of the environment

    % Initialisation of the environment as a struct
    environment = zeros(bounds.x, bounds.y, bounds.z);  

    % Display the environment if needed
    visualiseEnvironment(bounds);
end

% Visualising the 3D environment
    %This however will be commented out as main.m is called, multiple
    %environments will be visualised
    
function visualiseEnvironment(bounds)
%{
    fig = figure('Name', '3D Environment Visualisation');
    ax = axes('Parent', fig);
    hold(ax, 'on');
    view(ax, 3);
    axis(ax, [1 bounds.x 1 bounds.y 1 bounds.z]);
    xlabel(ax, 'X');
    ylabel(ax, 'Y');
    zlabel(ax, 'Z');
    title(ax, '3D Environment');
    grid(ax, 'on');
    
%}



    drawnow; % Update the figure window
end
