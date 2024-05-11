function dynamicNavigation()
    % Setup environment and initialise
    [environment, bounds] = main();
    goals = [25, 125, 5; 100, 100, 10; 150, 50, 10; 200, 175, 10];
    obstacles = [20, 75, 5; 75, 100, 10; 125, 60, 10; 175, 125, 10];
    obstacleSize = 10;
    
    % Define the grid resolution for the navigation grid
    gridResolution = 5;  % Grid resolution defines the granularity of the grid

    % Initialise visualisation and particle
    figHandle = figure;
    axesHandle = axes('Parent', figHandle);
    setupAxes(axesHandle, bounds);
    drawCubes(obstacles, obstacleSize, 'r', axesHandle);
    particle = InitialiseParticle([0, 0, 0], 1);

    % Create the navigation grid based on environment boundaries and obstacles
    navigationGrid = createNavigationGrid(bounds, obstacles, gridResolution);

    % Navigate through goals using the custom navigation grid
    %navigateParticle(particle, goals, obstacles, navigationGrid, gridResolution, axesHandle);
    navigateParticle(particle, goals, obstacles, obstacleSize, navigationGrid, gridResolution, axesHandle);

end

function navigateParticle(particle, goals, obstacles, obstacleSize, navigationGrid, gridResolution, axesHandle)
    disp('Starting navigation...');
    for idx = 1:size(goals, 1)
        goal = goals(idx, :);
        navigationPath = AStarSearch(navigationGrid, particle.position, goal);

        if isempty(navigationPath)
            disp('No path found to goal.');
            continue;
        end

        for node = navigationPath'  % Iterate through the path
            nextPosition = node * gridResolution;
            disp(['Moving from ', mat2str(particle.position), ' to ', mat2str(nextPosition)]);
            if detectObstacles(particle.position, nextPosition, obstacles, obstacleSize)
                particle.position = avoidObstacles(particle.position, nextPosition, obstacles);
            else
                particle.position = nextPosition;
            end
            updatevisualisation(particle, obstacles, axesHandle);
            if atGoal(particle.position, goal)
                disp('Reached goal.');
                break;
            end
            pause(0.1);  % Ensure the movement is visible
        end
    end
    disp('Particle reached all goals!');
end





function obstacleDetected = detectObstacles(currentPosition, nextPosition, obstacles, obstacleSize)
    obstacleDetected = false;
    for i = 1:size(obstacles, 1)
        obstacle = obstacles(i, :);
        if lineIntersectsSphere(currentPosition, nextPosition, obstacle, obstacleSize)
            obstacleDetected = true;
            break;
        end
    end
end

function intersects = lineIntersectsSphere(p1, p2, center, radius)
    % Vector from p1 to p2
    d = p2 - p1;
    % Vector from p1 to the center of the sphere
    f = p1 - center;
    
    a = dot(d, d);
    b = 2 * dot(f, d);
    c = dot(f, f) - radius^2;
    
    discriminant = b^2 - 4 * a * c;
    intersects = (discriminant >= 0);
end



function navigationGrid = createNavigationGrid(bounds, obstacles, gridResolution)
    % Calculate the number of grid cells in each dimension
    gridSizeX = ceil(bounds.x / gridResolution);
    gridSizeY = ceil(bounds.y / gridResolution);
    gridSizeZ = ceil(bounds.z / gridResolution);

    % Initialise the grid as a 3D matrix of zeros (free spaces)
    navigationGrid = zeros(gridSizeX, gridSizeY, gridSizeZ);

    % Process each obstacle to mark the grid cells it occupies
    for i = 1:size(obstacles, 1)
        % Convert obstacle coordinates to grid indices
        xIndex = min(max(round(obstacles(i, 1) / gridResolution) + 1, 1), gridSizeX);
        yIndex = min(max(round(obstacles(i, 2) / gridResolution) + 1, 1), gridSizeY);
        zIndex = min(max(round(obstacles(i, 3) / gridResolution) + 1, 1), gridSizeZ);

        % Mark the corresponding grid cell as obstructed (1)
        navigationGrid(xIndex, yIndex, zIndex) = 1;
    end
end



% Define other helper functions (InitialiseParticle, atGoal, moveTo, etc.) here
function grid = createGrid(bounds, obstacles, gridResolution)
    % Calculate the number of grid cells in each dimension
    gridSizeX = ceil(bounds.x / gridResolution);
    gridSizeY = ceil(bounds.y / gridResolution);
    gridSizeZ = ceil(bounds.z / gridResolution);

    % Initialise the grid as a 3D matrix of zeros (free spaces)
    grid = zeros(gridSizeX, gridSizeY, gridSizeZ);

    % Process each obstacle to mark the grid cells it occupies
    for i = 1:size(obstacles, 1)
        % Assume each obstacle is a point for simplicity. For extended obstacles, more complex logic is needed.
        % Convert obstacle coordinates to grid indices
        xIndex = min(max(round(obstacles(i, 1) / gridResolution) + 1, 1), gridSizeX);
        yIndex = min(max(round(obstacles(i, 2) / gridResolution) + 1, 1), gridSizeY);
        zIndex = min(max(round(obstacles(i, 3) / gridResolution) + 1, 1), gridSizeZ);

        % Mark the corresponding grid cell as obstructed (1)
        grid(xIndex, yIndex, zIndex) = 1;
    end
end

function setupAxes(ax, bounds)
    hold(ax, 'on');
    axis(ax, 'equal');
    xlim(ax, [0 bounds.x]);
    ylim(ax, [0 bounds.y]);
    zlim(ax, [0 bounds.z]);
    grid(ax, 'on');
    xlabel(ax, 'X');
    ylabel(ax, 'Y');
    zlabel(ax, 'Z');
    view(ax, 3);
end

function path = AStarSearch(grid, startCoords, goalCoords)
    % Extract grid dimensions for boundary checks
    [gridSizeX, gridSizeY, gridSizeZ] = size(grid);
    
    % Initialise the open and closed lists
    openList = containers.Map('KeyType','char','ValueType','any');
    closedList = containers.Map('KeyType','char','ValueType','any');
    
    % Create the start and goal nodes
    startNode = struct('Position', startCoords, 'G', 0, 'H', heuristic(startCoords, goalCoords), 'F', heuristic(startCoords, goalCoords), 'Parent', []);
    goalNode = struct('Position', goalCoords, 'G', 0, 'H', 0, 'F', 0, 'Parent', []);
    openList(getKeyFromPosition(startCoords)) = startNode;
    
    % Main loop of A*
    while ~isempty(openList)
        % Find the node in open list with the lowest F score
        currentNode = extractMin(openList);
        
        % Move current node from open list to closed list
        keyCurrent = getKeyFromPosition(currentNode.Position);
        openList.remove(keyCurrent);
        closedList(keyCurrent) = currentNode;
        
        % Check if the goal is reached
        if all(currentNode.Position == goalNode.Position)
            path = reconstructPath(currentNode);
            return;
        end
        
        % Generate neighbors (adjacent grid cells)
        neighbors = getNeighbors(currentNode.Position, gridSizeX, gridSizeY, gridSizeZ);
        
        % Iterate through each neighbor
        for i = 1:size(neighbors, 1)
            neighborCoords = neighbors(i, :);
            keyNeighbor = getKeyFromPosition(neighborCoords);
            
            % Skip if neighbor is in the closed list or is blocked
            if isKey(closedList, keyNeighbor) || grid(neighborCoords(1), neighborCoords(2), neighborCoords(3)) == 1
                continue;
            end
            
            % Calculate the G score for the neighbor
            tentativeG = currentNode.G + norm(neighborCoords - currentNode.Position);
            
            % If neighbor is not in open list or tentative G score is lower
            if ~isKey(openList, keyNeighbor) || tentativeG < openList(keyNeighbor).G
                neighborNode = struct('Position', neighborCoords, 'G', tentativeG, ...
                                      'H', heuristic(neighborCoords, goalCoords), 'F', tentativeG + heuristic(neighborCoords, goalCoords), ...
                                      'Parent', currentNode);
                openList(keyNeighbor) = neighborNode;
            end
        end
    end
    
    % If no path is found
    path = [];
end

function key = getKeyFromPosition(pos)
    key = sprintf('%d,%d,%d', pos(1), pos(2), pos(3));
end

function minNode = extractMin(openList)
    % Extract the node with the minimum F value from the open list
    minF = inf;
    minNode = [];
    keys = openList.keys;
    for i = 1:length(keys)
        node = openList(keys{i});
        if node.F < minF
            minF = node.F;
            minNode = node;
        end
    end
end

function neighbors = getNeighbors(position, maxX, maxY, maxZ)
    % Define possible moves (8-connected in 2D, similar extension can be made for 3D)
    moves = [-1, 0, 0; 1, 0, 0; 0, -1, 0; 0, 1, 0; 0, 0, -1; 0, 0, 1];
    neighbors = [];
    for move = moves'
        newPos = position + move';
        if all(newPos >= 1 & newPos <= [maxX, maxY, maxZ])  % Check grid boundaries
            neighbors = [neighbors; newPos];
        end
    end
end

function h = heuristic(pos, goalPos)
    % Euclidean distance as a heuristic
    h = norm(pos - goalPos);
end

function path = reconstructPath(node)
    % Reconstruct the path from start to goal by following parent links
    path = [];
    while ~isempty(node)
        path = [node.Position; path];
        node = node.Parent;
    end
end

function particle = InitialiseParticle(initialPosition, stepSize)
    % Ensure initialPosition is a row vector
    if size(initialPosition, 1) > 1
        initialPosition = initialPosition';  % Transpose if it's a column vector
    end
    particle = struct;
    particle.position = initialPosition;  % Set as a row vector
    particle.stepSize = stepSize;
end




function drawCubes(obstacles, obstacleSize, color, ax)
    % Each cube is defined by 8 vertices and 6 faces
    halfSize = (obstacleSize / 2) * 2;  % Enlarge the cube by 2.5 times
    vertices = [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; ...
                -1 -1 1; 1 -1 1; 1 1 1; -1 1 1] * halfSize;
    faces = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    
    % Draw each obstacle as a cube
    for i = 1:size(obstacles, 1)
        % Calculate the absolute positions of vertices for each cube
        cubeVertices = vertices + obstacles(i, :);
        % Plot each face of the cube
        for j = 1:size(faces, 1)
            face = cubeVertices(faces(j, :), :);
            fill3(ax, face(:,1), face(:,2), face(:,3), color, 'FaceAlpha', 0.5);
        end
    end
end



%

function flag = atGoal(position, goal)
    % Define the proximity threshold
    threshold = 1;  % Distance within which the particle is considered to have reached the goal

    % Calculate the Euclidean distance between the current position and the goal
    distance = norm(position - goal);

    % Check if the particle is within the threshold distance
    flag = distance <= threshold;
end

%

function pos = moveTo(position, goal)
    % Ensure input vectors are row vectors
    position = position(:)';
    goal = goal(:)';

    direction = goal - position;
    distance = norm(direction);
    direction = direction / max(distance, eps);  % Normalize safely

    pos = position + direction * min(1, distance);  % Move towards the goal
end


%

function grid = updateGridWithObstacles(grid, obstacles, obstacleSize, gridResolution)
    % Function to update the navigation grid dynamically based on obstacles
    
    % Iterate through each obstacle and mark the grid cells as blocked
    for i = 1:size(obstacles, 1)
        % Calculate grid cell coordinates from obstacle coordinates
        gridX = round(obstacles(i, 1) / gridResolution) + 1;
        gridY = round(obstacles(i, 2) / gridResolution) + 1;
        gridZ = round(obstacles(i, 3) / gridResolution) + 1;
        
        % Mark surrounding cells as blocked based on the obstacle size
        range = ceil(obstacleSize / (2 * gridResolution));
        minX = max(1, gridX - range);
        maxX = min(size(grid, 1), gridX + range);
        minY = max(1, gridY - range);
        maxY = min(size(grid, 2), gridY + range);
        minZ = max(1, gridZ - range);
        maxZ = min(size(grid, 3), gridZ + range);
        
        grid(minX:maxX, minY:maxY, minZ:maxZ) = 1;  % Mark cells as blocked
    end
end

%
function pos = avoidObstacles(position, goal, obstacles, bounds)
    nearestObstacle = findObstacleOnPath(position, obstacles, goal);
    toGoal = goal - position;
    toObstacle = nearestObstacle - position;
    distanceToObstacle = norm(toObstacle);
    
    if distanceToObstacle > 0 && distanceToObstacle < norm(toGoal)
        avoidanceVector = projectPerpendicular(toGoal, toObstacle);
        avoidanceStrength = 0.5;
        newPos = position + (toGoal + avoidanceStrength * avoidanceVector);
    else
        newPos = position + toGoal;
    end
    
    newPos = max(min(newPos, bounds.max'), bounds.min');
    pos = newPos;
end

function obstacle = findObstacleOnPath(position, obstacles, goal)
    minAngleDiff = inf;
    obstacleIdx = 0;
    for i = 1:size(obstacles, 1)
        toObstacle = obstacles(i, :) - position;
        angleDiff = acos(dot(toGoal, toObstacle) / (norm(toGoal) * norm(toObstacle)));
        if angleDiff < minAngleDiff
            minAngleDiff = angleDiff;
            obstacleIdx = i;
        end
    end
    obstacle = obstacles(obstacleIdx, :);
end

function projectedVec = projectPerpendicular(vec1, vec2)
    projection = dot(vec1, vec2) / norm(vec2)^2 * vec2;
    projectedVec = vec1 - projection;
end




function nearestObstacle = findNearestObstacle(position, obstacles)
    % Calculate Euclidean distances from the current position to each obstacle
    distances = sqrt(sum((obstacles - position).^2, 2));
    
    % Find the index of the minimum distance
    [minDistance, idx] = min(distances);
    
    % Select the nearest obstacle based on the index of the minimum distance
    nearestObstacle = obstacles(idx, :);
end


function updatevisualisation(particle, obstacles, axesHandle)
    persistent particlePlot obstaclePlots;

    if isempty(particlePlot) || ~isvalid(particlePlot)
        particlePlot = scatter3(axesHandle, particle.position(1), particle.position(2), particle.position(3), 'o', 'SizeData', 120, 'MarkerFaceColor', 'g');
        disp(['Created particle plot at: ', mat2str(particle.position)]);
    else
        set(particlePlot, 'XData', particle.position(1), 'YData', particle.position(2), 'ZData', particle.position(3));
        disp(['Updated particle position to: ', mat2str(particle.position)]);
    end

    if isempty(obstaclePlots)
        obstaclePlots = gobjects(size(obstacles, 1), 1);  % Object handles
        for i = 1:size(obstacles, 1)
            obstaclePlots(i) = scatter3(axesHandle, obstacles(i, 1), obstacles(i, 2), obstacles(i, 3), 'filled', 'MarkerFaceColor', 'r', 'SizeData', 150);
        end
    end
    drawnow;
end
