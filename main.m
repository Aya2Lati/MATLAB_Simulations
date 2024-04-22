% This will setup the 3D environment for the different simulations that will be run.
% It will be called
function [environment, bounds] = main()
    % size of the 3D environment
    environment = zeros(100, 100, 50);
    bounds.x = 100; % size of the x-dimension
    bounds.y = 100; % y-dimension
    bounds.z = 50;  % z-dimension

    % Initialising the environment
    environment = zeros(bounds.x, bounds.y, bounds.z);

    % Display the environment if needed
    visualiseEnvironment(environment, bounds);
end

function visualiseEnvironment(env, bounds)
    % This function visualizes the 3D environment.
    % For a large environment, a more sophisticated visualization might be necessary.
    
    [X, Y, Z] = ndgrid(1:bounds.x, 1:bounds.y, 1:bounds.z);
    scatter3(X(:), Y(:), Z(:), 1, env(:), 'filled');
    axis([1 bounds.x 1 bounds.y 1 bounds.z]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('3D Environment');
    grid on;
    drawnow;

    return;
end
