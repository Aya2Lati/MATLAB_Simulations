function singleParticle()
    % Calling main.m to setup the 3D environment
    [~, bounds] = main();

    % Setting up a figure for visualisation
    figure;
    hold on;
    axis equal;
    xlim([0 bounds.x]);
    ylim([0 bounds.y]);
    zlim([0 bounds.z]);

    % Defining goals, initial settings, as well as circling parameters
    goals = [20, 175, 5; 
            50, 90, 10; 
            125, 150, 10; 
            200, 50, 10];
    origin = [3, 3, 3];
    particle_pos = origin;
    goalIndex = 1;
    
    % setting the particle.
    h = scatter3(particle_pos(1), particle_pos(2), particle_pos(3), 'filled', 'MarkerFaceColor', 'r');

    view(3); %This will ensure that the view is a 3D perspective
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('3D Navigation Simulation with Goals');

    slowdownThreshold = 10; %this is the distance at which the partisle will start to slow down
    circlingThreshold = 5;  % Distance at which to start circling
    circlingRadius = 5;     %circling radius
    circlingSpeed = 0.01;    %circling speed(essentially radians per iteration)
    minStepSize = 0.5;      %the minimum step size when rotating

    % Initialising GIF
    filename = 'singleParticle_simulation.gif';
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    % Start the GIF
    imwrite(imind, cm, filename, 'gif', 'Loopcount', inf);
    
    %Setting up the particle to navigate towards each goal
    while goalIndex <= size(goals, 1)
        goal = goals(goalIndex, :);
        stepSize = 1; % Resetting for new goal
        
        % as the aprticle approaches the goal, it will slow down
        while norm(particle_pos - goal) > circlingThreshold
            if norm(particle_pos - goal) < slowdownThreshold
                stepSize = max(minStepSize, (norm(particle_pos - goal) / slowdownThreshold) * stepSize);
            end
            direction = (goal - particle_pos) / norm(goal - particle_pos) * stepSize;
            particle_pos = particle_pos + direction;

            %position update
            set(h, 'XData', particle_pos(1), 'YData', particle_pos(2), 'ZData', particle_pos(3));
            plot3(particle_pos(1), particle_pos(2), particle_pos(3), '.b');  % Trail
            drawnow;
            pause(0.005);

            %saving frame to GIF
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, ~] = rgb2ind(im, cm);
            imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append'); % AppendS to the GIF
        end
        
        %when close to the goal, the particle will start to encircle
        if norm(particle_pos - goal) < circlingThreshold
            %calculating the number of steps to complete a full circle
            numSteps = round(2 * pi * circlingRadius / stepSize);
            
            for step = 1:numSteps
                angle = 2 * pi * (step / numSteps); % Compute angle for current step
                offset = circlingRadius * [cos(angle), sin(angle), 0];
                particle_pos = goal + offset;
                
                %updates the position of the particle in the plot
                set(h, 'XData', particle_pos(1), 'YData', particle_pos(2), 'ZData', particle_pos(3));
                %for better visualiation, this limit is added
                drawnow limitrate;
                pause(0.00005);

                % Capture frame and add it to GIF using the consistent color map
                frame = getframe(gcf);
                im = frame2im(frame);
                [imind, ~] = rgb2ind(im, cm);
                imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append'); % Append to the GIF
            end
        end

        goalIndex = goalIndex + 1; %move on to the next goal
    end

    disp('Particle reached all goals!');
end
