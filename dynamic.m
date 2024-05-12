function dynamicFormation(~)
    % 3D environment setup
    [~, bounds] = main();

    % Initial simulation setup
    [particles, currentFormation, targetPositions, currentTime, nextTransitionTime, formationList, formationIndex, bounds] = initialiseSimulationParameters();

    %GIF initialisation
    filename = 'dynamic_particleFormation.gif';
    firstFrame = true;  %Due to issues this was implemented to check if it's the first frame of the GIF

    % Main simulation loop
    while true
        
        %update the current time of the simulation
        currentTime = updateSimulationTime(currentTime);

        %transitioning to the next formation
        if currentTime >= nextTransitionTime
            [currentFormation, targetPositions, nextTransitionTime, formationIndex] = transitionToNextFormation(currentFormation, formationList, formationIndex, currentTime);
        end
        
        % particle states are updated
        particles = updateParticleStates(particles, targetPositions, bounds);

        %GIF capture
        updateVisualisation(particles, bounds, filename, firstFrame);
        firstFrame = false;

        %simulation time is update
        currentTime = updateSimulationTime(currentTime);

        %checking to see if the simulation should stop
        if shouldStopSimulation(currentTime)
            break;
        end

        %this is to ensure real time sim visualisation
        pause(0.0005);
    end
end



%

function [particles, currentFormation, targetPositions, currentTime, nextTransitionTime, formationList, formationIndex, bounds] = initialiseSimulationParameters()
    %No. of particles
    numParticles = 9;
    
    %these are the coordinates where the particles will form
    centerPosition = [125, 100, 50];
    
    %formations that will be formed
    formationList = {'V', 'Line', 'Circle'};
    
    %highlighting what formation will go 1st
    formationIndex = 1;
    currentFormation = formationList{formationIndex};
    
    %this is how long the formations will last in seconds
    formationDuration = 30; 
    
    % Initialisation of the current simulation in regards to time
    currentTime = 0;
    
    %calculating transition time
    nextTransitionTime = formationDuration;
    
    %the particles are, initially set to zeros in regards to position
    particles = zeros(numParticles, 3);  
    
    %target positions for whatever formation is being formed
    targetPositions = calculateFormation(currentFormation, numParticles, centerPosition);
    
    %bounds
    bounds.x = 250;
    bounds.y = 200;
    bounds.z = 150;
end



%

%The particles are moved to where they need to form
function particles = moveParticlesTowardsFormation(particles, targetPositions)
    stepSize = 1; 
    numParticles = size(particles, 1);
    
    for i = 1:numParticles
        %vector calculation from current position to the target position
        moveVector = targetPositions(i, :) - particles(i, :);
        distance = norm(moveVector);
        
        %if distance is not zero, normalise the vetor
        if distance > 0
            moveVector = moveVector / distance;
        end
                
        particles(i, :) = particles(i, :) + moveVector * min(stepSize, distance);
    end
end

%

function [currentFormation, targetPositions, nextTransitionTime, formationIndex] = transitionToNextFormation(currentFormation, formationList, formationIndex, currentTime)
    %this is how long the formations will last in seconds
    formationDuration = 30;

    %after the first formation, which one is up next
    formationIndex = mod(formationIndex, length(formationList)) + 1;

    %formation update
    currentFormation = formationList{formationIndex};

    %No. of particles
    numParticles = 9;

    %formation position
    centerPosition = [125, 100, 50];

    %formation targets
    targetPositions = calculateFormation(currentFormation, numParticles, centerPosition);

    %next transition
    nextTransitionTime = currentTime + formationDuration;
end

%

function particles = updateParticleStates(particles, targetPositions, bounds)
    %Applying the consensus algorithm to adjust particle positions based on their neighbours
    particles = consensusAlgorithm(particles, targetPositions, bounds);
    
    %collision avoidance to ensure particles don't collide with one another
    particles = collisionAvoidance(particles, bounds);
    
    %moving particles to designated positions
    particles = moveParticlesTowardsFormation(particles, targetPositions);
end

%

function particles = consensusAlgorithm(particles, targetPositions, bounds)
    alpha = 0.05;
    numParticles = size(particles, 1);
    %communication range within which particles influence each other
    communicationRange = 10; 
    
    %iterating over each particle to update its position
    for i = 1:numParticles
        %variables for calculating the average position
        sumPosition = zeros(1, 3);
        countNeighbours = 0;
        
        for j = 1:numParticles
            if i ~= j && norm(particles(i, :) - particles(j, :)) <= communicationRange
                sumPosition = sumPosition + particles(j, :);
                countNeighbours = countNeighbours + 1;
            end
        end
        
        %calculating the average position of particle neighbours
        if countNeighbours > 0
            averagePosition = sumPosition / countNeighbours;
            movementVector = alpha * (averagePosition - particles(i, :));
            particles(i, :) = particles(i, :) + movementVector;
        end
        
        %making sure that the particles are within bounds
        particles(i, 1) = max(min(particles(i, 1), bounds.x), 0);  % For x
        particles(i, 2) = max(min(particles(i, 2), bounds.y), 0);  % For y
        particles(i, 3) = max(min(particles(i, 3), bounds.z), 0);  % For z
    end
end

%

function particles = collisionAvoidance(particles, bounds)
    safeDistance = 5; %defining the operational distance
    numParticles = size(particles, 1);

    %iterating over all unique pairs of particles
    for i = 1:numParticles-1
        for j = i+1:numParticles
            %vector between particles as well as its magnitude
            distanceVector = particles(j, :) - particles(i, :);
            distance = norm(distanceVector);

            %checking if the distance is less than the safe distance
            if distance < safeDistance
                %adjustment vector to push particles apart
                adjustmentVector = (safeDistance - distance) / 2 * (distanceVector / distance);

                %position adjustment for the particles
                particles(i, :) = particles(i, :) - adjustmentVector;
                particles(j, :) = particles(j, :) + adjustmentVector;

                %thi will ensure that the particles remain within the
                %bounds
                particles(i, 1) = max(min(particles(i, 1), bounds.x), 0);  
                particles(i, 2) = max(min(particles(i, 2), bounds.y), 0);  
                particles(i, 3) = max(min(particles(i, 3), bounds.z), 0);  

                particles(j, 1) = max(min(particles(j, 1), bounds.x), 0);  
                particles(j, 2) = max(min(particles(j, 2), bounds.y), 0);  
                particles(j, 3) = max(min(particles(j, 3), bounds.z), 0);  
            end
        end
    end
end

%

function updateVisualisation(particles, bounds, filename, firstFrame)
    %checking to see if a figure exists
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
        fig.UserData.scatterPlot = scatterPlot;
    else
        ax = fig.UserData.ax;
        if isfield(fig.UserData, 'scatterPlot')
            scatterPlot = fig.UserData.scatterPlot;
            set(scatterPlot, 'XData', particles(:,1), 'YData', particles(:,2), 'ZData', particles(:,3));
        else
            error('scatterPlot is not set up properly in UserData.');
        end
    end

    xlim(ax, [0 bounds.x]);
    ylim(ax, [0 bounds.y]);
    zlim(ax, [0 bounds.z]);

    drawnow;

    %GIF frame 
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
    %incrementation of the  time by 0.1 seconds
    currentTime = currentTime + 0.1;
end


%

function stop = shouldStopSimulation(currentTime)
    maxSimulationTime = 600;  %maximum time allowed in seconds
    stop = currentTime >= maxSimulationTime;
end

%

function positions = calculateFormation(formationType, numParticles, center)
    positions = zeros(numParticles, 3);
    switch formationType
        case 'V'
            %angle adjustment for V shape
            angle = pi / 4;
            %the position that will be taken up by the particles
            for i = 1:numParticles
                if mod(i, 2) == 0
                    positions(i, :) = center + (floor(i/2)) * [cos(angle), sin(angle), 0];
                else
                    positions(i, :) = center + (floor(i/2)) * [cos(-angle), sin(-angle), 0];
                end
            end

        case 'Line'
            %line formation
            for i = 1:numParticles
                %adjusting positions along the Y-axis, centered around the middle
                positions(i, :) = center + [0, (i - (numParticles + 1)/2) * 3, 0];
            end

        case 'Circle'
            %circle formation
            radius = 10;  %radius size
            for i = 1:numParticles
                %even distribution of particles around a circle
                angle = 2 * pi * (i - 1) / numParticles;
                positions(i, :) = center + radius * [cos(angle), sin(angle), 0];
            end
    end
end
