function dynamic(~)
    % Include the main file which sets up the 3D environment
    [~, bounds] = main();

    % Initial setup for the simulation
    [particles, currentFormation, targetPositions, currentTime, nextTransitionTime, formationList, formationIndex, bounds] = initialiseSimulationParameters();

    % Initialise GIF creation
    filename = 'dynamic_particleFormation.gif';
    firstFrame = true;  % Flag to check if it's the first frame of the GIF

    % Main simulation loop
    while true
        
        % Update the current time of the simulation
        currentTime = updateSimulationTime(currentTime);

        % Time to transition to the next formation
        if currentTime >= nextTransitionTime
            [currentFormation, targetPositions, nextTransitionTime, formationIndex] = transitionToNextFormation(currentFormation, formationList, formationIndex, currentTime);
        end
        
        % Update particle states
        particles = updateParticleStates(particles, targetPositions, bounds);

        % Update visualisation and capture the frame for the GIF
        updateVisualisation(particles, bounds, filename, firstFrame);
        firstFrame = false;  % Update flag after the first frame

        % Update simulation time
        currentTime = updateSimulationTime(currentTime);

        % Check if the simulation should stop
        if shouldStopSimulation(currentTime)
            break;
        end

        % Pause for real-time visualisation
        pause(0.0005);
    end
end



%

function [particles, currentFormation, targetPositions, currentTime, nextTransitionTime, formationList, formationIndex, bounds] = initialiseSimulationParameters()
    % Define the number of particles
    numParticles = 9;
    
    % Define the center position for the formations
    centerPosition = [125, 100, 50];  % Center of formations
    
    % List of formations the particles will adopt
    formationList = {'V', 'Line', 'Circle'};
    
    % Start with the first formation in the list
    formationIndex = 1;
    currentFormation = formationList{formationIndex};
    
    % Duration each formation will last
    formationDuration = 30;  % Duration for each formation in seconds
    
    % Initialise the current simulation time
    currentTime = 0;
    
    % Calculate the next transition time
    nextTransitionTime = formationDuration;
    
    % Initialise particle positions, initially set to zeros
    particles = zeros(numParticles, 3);  
    
    % Calculate the target positions for the current formation
    targetPositions = calculateFormation(currentFormation, numParticles, centerPosition);
    
    % Define bounds for the simulation space correctly
    bounds.x = 250;
    bounds.y = 200;
    bounds.z = 150;
end



%

function particles = moveParticlesTowardsFormation(particles, targetPositions)
    stepSize = 1;  % Define the step size for movement
    numParticles = size(particles, 1);
    
    for i = 1:numParticles
        % Calculate the vector from the current position to the target position
        moveVector = targetPositions(i, :) - particles(i, :);
        distance = norm(moveVector);
        
        % Normalise the move vector if distance is not zero
        if distance > 0
            moveVector = moveVector / distance;
        end
        
        % Move the particle towards the target by the smaller of the step size or the remaining distance
        particles(i, :) = particles(i, :) + moveVector * min(stepSize, distance);
    end
end


%

function [currentFormation, targetPositions, nextTransitionTime, formationIndex] = transitionToNextFormation(currentFormation, formationList, formationIndex, currentTime)
    % Duration for each formation in seconds
    formationDuration = 30;

    % Update the formation index to the next one in the list, cycling back to the start if necessary
    formationIndex = mod(formationIndex, length(formationList)) + 1;

    % Update the current formation based on the new index
    currentFormation = formationList{formationIndex};

    % Define the number of particles
    numParticles = 9;

    % Define the center position for formations
    centerPosition = [125, 100, 50];

    % Calculate the target positions for the new formation
    targetPositions = calculateFormation(currentFormation, numParticles, centerPosition);

    % Set the next transition time by adding the formation duration to the current time
    nextTransitionTime = currentTime + formationDuration;
end


%

function particles = updateParticleStates(particles, targetPositions, bounds)
    % Apply the consensus algorithm to adjust particle positions based on their neighbors
    particles = consensusAlgorithm(particles, targetPositions, bounds);
    
    % Apply collision avoidance to ensure particles do not overlap or come too close to each other
    particles = collisionAvoidance(particles, bounds);
    
    % Move particles towards their designated positions in the current formation
    particles = moveParticlesTowardsFormation(particles, targetPositions);
end

%

function particles = consensusAlgorithm(particles, targetPositions, bounds)
    alpha = 0.05; % Small step factor for movement towards the average position
    numParticles = size(particles, 1);
    communicationRange = 10; % Define a communication range within which particles influence each other
    
    % Iterate over each particle to update its position
    for i = 1:numParticles
        % Initialise variables to calculate average position
        sumPosition = zeros(1, 3);
        countNeighbors = 0;
        
        % Accumulate positions of neighbors within the communication range
        for j = 1:numParticles
            if i ~= j && norm(particles(i, :) - particles(j, :)) <= communicationRange
                sumPosition = sumPosition + particles(j, :);
                countNeighbors = countNeighbors + 1;
            end
        end
        
        % Calculate the average position of neighbors
        if countNeighbors > 0
            averagePosition = sumPosition / countNeighbors;
            movementVector = alpha * (averagePosition - particles(i, :));
            particles(i, :) = particles(i, :) + movementVector;
        end
        
        % Ensure particles remain within bounds
        particles(i, 1) = max(min(particles(i, 1), bounds.x), 0);  % For x dimension
        particles(i, 2) = max(min(particles(i, 2), bounds.y), 0);  % For y dimension
        particles(i, 3) = max(min(particles(i, 3), bounds.z), 0);  % For z dimension
    end
end

%

function particles = collisionAvoidance(particles, bounds)
    safeDistance = 5; % Define a safe operational distance
    numParticles = size(particles, 1);

    % Iterate over all unique pairs of particles
    for i = 1:numParticles-1
        for j = i+1:numParticles
            % Calculate the vector between particles and its magnitude
            distanceVector = particles(j, :) - particles(i, :);
            distance = norm(distanceVector);

            % Check if the distance is less than the safe distance
            if distance < safeDistance
                % Calculate the necessary adjustment vector to push particles apart
                adjustmentVector = (safeDistance - distance) / 2 * (distanceVector / distance);

                % Adjust positions of both particles
                particles(i, :) = particles(i, :) - adjustmentVector;
                particles(j, :) = particles(j, :) + adjustmentVector;

                % Ensure particles remain within the defined spatial bounds
                particles(i, 1) = max(min(particles(i, 1), bounds.x), 0);  % For x dimension
                particles(i, 2) = max(min(particles(i, 2), bounds.y), 0);  % For y dimension
                particles(i, 3) = max(min(particles(i, 3), bounds.z), 0);  % For z dimension

                particles(j, 1) = max(min(particles(j, 1), bounds.x), 0);  % For x dimension
                particles(j, 2) = max(min(particles(j, 2), bounds.y), 0);  % For y dimension
                particles(j, 3) = max(min(particles(j, 3), bounds.z), 0);  % For z dimension
            end
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
        xlabel(ax, 'X');
        ylabel(ax, 'Y');
        zlabel(ax, 'Z');
        title(ax, '3D Particle Movement');
        grid(ax, 'on');
        scatterPlot = scatter3(ax, particles(:,1), particles(:,2), particles(:,3), 'filled', 'MarkerFaceColor', 'b');
        fig.UserData.ax = ax;
        fig.UserData.scatterPlot = scatterPlot;  % Store scatter plot in UserData
    else
        ax = fig.UserData.ax;
        if isfield(fig.UserData, 'scatterPlot')
            scatterPlot = fig.UserData.scatterPlot;
            set(scatterPlot, 'XData', particles(:,1), 'YData', particles(:,2), 'ZData', particles(:,3));
        else
            error('scatterPlot is not set up properly in UserData.');
        end
    end

    % Adjust the axis limits based on bounds
    xlim(ax, [0 bounds.x]);
    ylim(ax, [0 bounds.y]);
    zlim(ax, [0 bounds.z]);

    drawnow; % Ensure the plot updates are rendered immediately

    % Capture the frame for the GIF
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    if firstFrame
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf);  % Start the GIF
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append');  % Append to the GIF
    end
end



%

function currentTime = updateSimulationTime(currentTime)
    % Increment the current time by 0.1 seconds
    currentTime = currentTime + 0.1;  % Increment by 0.1 seconds
end


%

function stop = shouldStopSimulation(currentTime)
    maxSimulationTime = 600;  % Maximum allowed time in seconds
    stop = currentTime >= maxSimulationTime;
end

%

function positions = calculateFormation(formationType, numParticles, center)
    positions = zeros(numParticles, 3);
    switch formationType
        case 'V'
            % Adjust angle for V shape
            angle = pi / 4;
            % Calculate positions for V formation
            for i = 1:numParticles
                if mod(i, 2) == 0
                    % Even index, one side of the V
                    positions(i, :) = center + (floor(i/2)) * [cos(angle), sin(angle), 0];
                else
                    % Odd index, other side of the V
                    positions(i, :) = center + (floor(i/2)) * [cos(-angle), sin(-angle), 0];
                end
            end

        case 'Line'
            % Line formation calculation
            for i = 1:numParticles
                % Adjust positions along the Y-axis, centered around the middle
                positions(i, :) = center + [0, (i - (numParticles + 1)/2) * 3, 0];
            end

        case 'Circle'
            % Circle formation calculation
            radius = 10;  % Set radius size
            for i = 1:numParticles
                % Distribute particles evenly around a circle
                angle = 2 * pi * (i - 1) / numParticles;
                positions(i, :) = center + radius * [cos(angle), sin(angle), 0];
            end
    end
end
