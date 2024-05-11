function formation(~)
    % Setup environment by calling main.m
    [~, bounds] = main();
    
    % Define initial coordinates and delay
    initialCoords = [50, 100, 10];
    targetCoords = [225, 100, 10];
    delayBeforeStart = 5; % seconds
    numParticles = 7; % including leader
    spacing = 10;

    % Create 7 sphere particles in a V formation
    particles = initialiseFormation(numParticles, initialCoords, targetCoords, spacing);

    % Delay before starting the movement
    pause(delayBeforeStart);

    % Initialise GIF creation
    filename = 'V_Formation.gif';
    firstFrame = true;  % Flag to check if it's the first frame of the GIF

    % Main simulation loop
    while ~hasReachedTarget(particles, targetCoords)
        % Apply the consensus algorithm
        particles = applyConsensusAlgorithm(particles, bounds);

        % Avoid collisions
        particles = avoidCollisions(particles, bounds);

        % Move particles towards the target
        particles = moveParticles(particles, targetCoords, bounds);

        % Update the visualisation and capture the frame for the GIF
        updateVisualisation(particles, bounds, filename, firstFrame);
        firstFrame = false;  % Update flag after the first frame

        % Check if the simulation should pause here for visualisation purposes
        pause(0.005); % Adjust pause duration for real-time effect or remove if not needed
    end
    
    disp('Leader has reached the target, simulation ended.');
end

%

function particles = initialiseFormation(numParticles, initialCoords, targetCoords, spacing)
    % Initialise particles array
    particles = zeros(numParticles, 3);
    
    % Set the leader at the initial coordinates
    particles(1, :) = initialCoords;
    
    % Calculate direction vector towards the target and normalise
    directionVector = targetCoords - initialCoords;
    unitDirectionVector = directionVector / norm(directionVector);
    
    % Calculate the perpendicular vector(s)
    if all(unitDirectionVector(1:2) == 0)
        % Special case when the movement is vertical
        perpendicularVector = [1, 0, 0];
    else
        % General case for a direction in the XY plane
        perpendicularVector = [unitDirectionVector(2), -unitDirectionVector(1), 0];
        perpendicularVector = perpendicularVector / norm(perpendicularVector);
    end
    
    % Determine the spacing for the inverted V formation
    rowSpacing = spacing;  % Distance between the leader and the next particle along the movement direction
    columnSpacing = spacing / 2;  % Lateral distance between particles in the V-formation
    
    % Calculate and set the positions for the left and right wing particles
    for i = 1:(numParticles-1)/2
        % Left wing (inverted)
        leftPosition = initialCoords - unitDirectionVector * rowSpacing * i + perpendicularVector * columnSpacing * i;
        particles(2*i, :) = leftPosition;
        
        % Right wing (inverted)
        rightPosition = initialCoords - unitDirectionVector * rowSpacing * i - perpendicularVector * columnSpacing * i;
        particles(2*i+1, :) = rightPosition;
    end
end


%


function particles = applyConsensusAlgorithm(particles, bounds)
    num_particles = size(particles, 2);
    
    % Assuming all particles are neighbors for simplicity
    neighbors = true(num_particles, num_particles);
    neighbors = neighbors & ~eye(num_particles);  % Exclude self by setting diagonal to false
    
    % Initialise new positions with current positions
    new_positions = particles;
    
    % The consensus algorithm will adjust each particle's position towards the average of its neighbors
    for i = 1:num_particles
        % Calculate the average position of neighbors excluding the particle itself
        average_position = mean(particles(neighbors(i, :), :), 1);
        
        % Update the particle's position slightly towards the average position
        alpha = 0.1;  % Example value for alpha, controls the rate of adjustment
        new_positions(i, :) = particles(i, :) + alpha * (average_position - particles(i, :));
        
        % Ensure the new position does not exceed the environment bounds
        new_positions(i, 1) = min(max(new_positions(i, 1), 1), bounds.x);
        new_positions(i, 2) = min(max(new_positions(i, 2), 1), bounds.y);
        new_positions(i, 3) = min(max(new_positions(i, 3), 1), bounds.z);
    end
    
    % Update particles with the new calculated positions
    particles = new_positions;
end


%


function particles = avoidCollisions(particles, bounds)
    num_particles = size(particles, 1);
    safe_distance = 8;  % Define a safe distance between particles

    % Loop over each particle and compare with all others
    for i = 1:num_particles
        for j = i+1:num_particles  % Avoid double checking by only comparing with subsequent particles
            distance_vector = particles(j, :) - particles(i, :);
            distance = norm(distance_vector);
            
            % If the particles are too close, push them apart
            if distance < safe_distance
                correction_vector = distance_vector / distance * (safe_distance - distance) / 2;
                particles(i, :) = max(min(particles(i, :) - correction_vector, bounds.x), 1);
                particles(j, :) = max(min(particles(j, :) + correction_vector, bounds.x), 1);
            end
        end
    end
end


%


function particles = moveParticles(particles, targetCoords, bounds)
    numParticles = size(particles, 1);
    stepSize = 1;
    formationSpacing = 10;  % Define inside the function
    sideOffset = 5;          % Define inside the function

    for i = 1:numParticles
        if i == 1  % Leader moves directly towards the target
            direction = targetCoords - particles(i, :);
            direction = direction / norm(direction);
            particles(i, :) = particles(i, :) + stepSize * direction;
        else
            row = ceil((i-1)/2);
            side = (-1)^(i+1);
            directionToTarget = targetCoords - particles(1, :);
            angleToTarget = atan2(directionToTarget(2), directionToTarget(1));
            xOffset = cos(angleToTarget) * formationSpacing * row - sin(angleToTarget) * side * sideOffset * row;
            yOffset = sin(angleToTarget) * formationSpacing * row + cos(angleToTarget) * side * sideOffset * row;
            targetPosition = particles(1, :) + [xOffset, yOffset, 0];
            moveVector = targetPosition - particles(i, :);
            moveDirection = moveVector / norm(moveVector);
            particles(i, :) = particles(i, :) + stepSize * moveDirection;
        end
    end
end

%

function updateVisualisation(particles, bounds, filename, firstFrame)
    % Check if a figure exists, if not create one
    fig = findobj('Type', 'figure', 'Name', 'Particle Simulation');
    if isempty(fig)
        fig = figure('Name', 'Particle Simulation');
        ax = axes('Parent', fig);
        hold(ax, 'on');
        view(ax, 3);
        axis(ax, [1 bounds.x 1 bounds.y 1 bounds.z]);
        xlabel(ax, 'X');
        ylabel(ax, 'Y');
        zlabel(ax, 'Z');
        title(ax, '3D Particle Movement');
        grid(ax, 'on');
        scatter3(ax, particles(:,1), particles(:,2), particles(:,3), 'filled', 'MarkerFaceColor', 'b');
    else
        ax = get(fig, 'Children');
        % Retrieve the scatter plot object
        scat = findobj(ax, 'Type', 'scatter');
        % Update particle positions on the scatter plot
        set(scat, 'XData', particles(:,1), 'YData', particles(:,2), 'ZData', particles(:,3));
    end

    % Adjust the axis limits if particles move outside the initial bounds
    xlim(ax, [0 bounds.x]);
    ylim(ax, [0 bounds.y]);
    zlim(ax, [0 bounds.z]);

    drawnow; % Ensure the plot updates are rendered immediately

    % Capture the frame for the GIF
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    if firstFrame
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append');
    end
end


%

function hasReached = hasReachedTarget(particles, targetCoords)
    % Assume leader is the first particle in the array
    leaderIndex = 1; 
    % Define a tolerance within which the leader is considered to have reached the target
    tolerance = 1;  % You can adjust this tolerance based on your simulation needs
    
    % Calculate the Euclidean distance from the leader's current position to the target
    distance = norm(particles(leaderIndex, :) - targetCoords);
    
    % Check if the leader is within the tolerance distance to the target coordinates
    hasReached = distance <= tolerance;
end

