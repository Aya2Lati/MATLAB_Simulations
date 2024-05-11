function singleParticle()
    % Call the main function to setup the 3D environment and retrieve boundaries
    [~, bounds] = main();

    % Setup the figure for navigation visualisation
    figure;
    hold on;
    axis equal;
    xlim([0 bounds.x]);
    ylim([0 bounds.y]);
    zlim([0 bounds.z]);

    % Define goals, initial settings, and circling parameters
    goals = [20, 175, 5; 
            50, 90, 10; 
            125, 150, 10; 
            200, 50, 10];
    origin = [3, 3, 3];
    particle_pos = origin;
    goalIndex = 1;
    
    % Particle visual representation as a red sphere
    h = scatter3(particle_pos(1), particle_pos(2), particle_pos(3), 'filled', 'MarkerFaceColor', 'r');
    view(3); % Sets the view to 3D perspective
    grid on; % Display grid
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('3D Navigation Simulation with Goals');

    slowdownThreshold = 10; % Distance at which to start slowing down
    circlingThreshold = 5;  % Distance at which to start circling
    circlingRadius = 5;     % Radius of circling
    circlingSpeed = 0.01;    % Angular speed for circling (radians per iteration)
    minStepSize = 0.5;      % Minimum step size when slowing down

    % Initialise GIF file
    filename = 'singleParticle_simulation.gif';
    
    % Initialise the color map from the first frame to ensure consistency
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    imwrite(imind, cm, filename, 'gif', 'Loopcount', inf); % Start the GIF
    
    % Navigate towards each goal
    while goalIndex <= size(goals, 1)
        goal = goals(goalIndex, :);
        stepSize = 1; % Reset step size for new goal
        
        % Slow down as we approach the goal
        while norm(particle_pos - goal) > circlingThreshold
            if norm(particle_pos - goal) < slowdownThreshold
                stepSize = max(minStepSize, (norm(particle_pos - goal) / slowdownThreshold) * stepSize);
            end
            direction = (goal - particle_pos) / norm(goal - particle_pos) * stepSize;
            particle_pos = particle_pos + direction;

            % Update the position of the particle in the plot
            set(h, 'XData', particle_pos(1), 'YData', particle_pos(2), 'ZData', particle_pos(3));
            plot3(particle_pos(1), particle_pos(2), particle_pos(3), '.b');  % Trail
            drawnow;
            pause(0.005);

            % Capture frame and add it to GIF using the consistent color map
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind, ~] = rgb2ind(im, cm);
            imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append'); % Append to the GIF
        end
        
        % Start circling when close to the goal
        if norm(particle_pos - goal) < circlingThreshold
            % Calculate number of steps to complete a full circle based on step size
            numSteps = round(2 * pi * circlingRadius / stepSize);
            
            for step = 1:numSteps
                angle = 2 * pi * (step / numSteps); % Compute angle for current step
                offset = circlingRadius * [cos(angle), sin(angle), 0];
                particle_pos = goal + offset;
                
                % Update the position of the particle in the plot
                set(h, 'XData', particle_pos(1), 'YData', particle_pos(2), 'ZData', particle_pos(3));
                drawnow limitrate; % For smoother animation
                pause(0.00005);

                % Capture frame and add it to GIF using the consistent color map
                frame = getframe(gcf);
                im = frame2im(frame);
                [imind, ~] = rgb2ind(im, cm);
                imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append'); % Append to the GIF
            end
        end

        goalIndex = goalIndex + 1; % Move to the next goal
    end

    disp('Particle reached all goals!');
end
