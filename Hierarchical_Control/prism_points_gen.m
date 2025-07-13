% Function to generate points on the lateral faces of a rectangular prism
% Used for modeling an obstacle as a prism in 3D space

% Inputs:
%   - ObstaclePositions: [x, y] coordinates of the prism center on the XY-plane
%   - ObstacleHeights: height of the prism (in the Z direction)
%   - ObstacleWidths: width of the square base (assumes square base)

% Output:
%   - obstacle: 3xN matrix containing the 3D coordinates of the sampled surface points

% function obstacle = prism_points_gen(ObstaclePositions, ObstacleHeights, ObstacleWidths)
% 
%     % Extract center coordinates
%     x_center = ObstaclePositions(1);
%     y_center = ObstaclePositions(2);
%     side = ObstacleWidths;
%     height = ObstacleHeights;
% 
%     % Discretization parameters
%     num_side_points = 40;      % number of points along each side of the base
%     num_height_points = 100;   % number of points along the height (Z-axis)
% 
%     % Compute the 4 vertices of the square base (clockwise)
%     half_side = side / 2;
%     vertices = [ x_center - half_side, y_center - half_side;  % V1
%                  x_center + half_side, y_center - half_side;  % V2
%                  x_center + half_side, y_center + half_side;  % V3
%                  x_center - half_side, y_center + half_side]; % V4
% 
%     % Discretize height values from 0 to total height
%     z_vals = linspace(0, height, num_height_points);
% 
%     % Preallocate array for obstacle points
%     total_points = 4 * num_side_points * num_height_points;
%     obstacle = zeros(3, total_points);
%     idx = 1;
% 
%     % Generate points on the four lateral faces
%     for k = 1:4
%         % Select the two vertices defining one side of the base
%         p1 = vertices(k, :);
%         p2 = vertices(mod(k, 4) + 1, :);  % Wrap around for last vertex
% 
%         % Linearly interpolate points along the base side
%         x_side = linspace(p1(1), p2(1), num_side_points);
%         y_side = linspace(p1(2), p2(2), num_side_points);
% 
%         % For each height level, add surface points on this face
%         for z = z_vals
%             for i = 1:num_side_points
%                 obstacle(:, idx) = [x_side(i); y_side(i); -z];  % Z is negative
%                 idx = idx + 1;
%             end
%         end
%     end
% 
% end


function obstacle = prism_points_gen(ObstaclePositions, ObstacleHeights, ObstacleWidths)
    % Function to generate 3D points on the lateral surface of a vertical cylinder
    % The cylinder is inscribed in a square-based rectangular prism.
    % This means the circular base fits perfectly inside the square base,
    % touching all four sides.
    %
    % Inputs:
    %   - ObstaclePositions: [x, y] coordinates of the center of the prism base
    %   - ObstacleHeights: total height of the prism (and the cylinder)
    %   - ObstacleWidths: width of the square base (cylinder is inscribed in this square)
    %
    % Output:
    %   - obstacle: 3xN matrix of points sampled on the cylindrical surface

    % Extract geometric parameters
    x_center = ObstaclePositions(1);
    y_center = ObstaclePositions(2);
    side = ObstacleWidths;
    height = ObstacleHeights;

    % Discretization parameters
    num_circle_points = 40;    % number of points sampled along the circle
    num_height_points = 70;    % number of layers along the height

    % Radius of the cylinder inscribed in a square (half the side length)
    radius = side / 2;

    % Generate angles (from 0 to just before 2π to avoid duplicate point)
    theta = linspace(0, 2*pi, num_circle_points + 1);
    theta(end) = [];  % remove duplicate at 2π

    % Height values from bottom (0) to top (height)
    z_vals = linspace(0, height, num_height_points);

    % Total number of points to be generated
    total_points = num_circle_points * num_height_points;
    obstacle = zeros(3, total_points);
    idx = 1;

    % Generate points on the cylindrical surface
    for z = z_vals
        for t = theta
            x = x_center + radius * cos(t);
            y = y_center + radius * sin(t);
            obstacle(:, idx) = [x; y; -z];  % Z is negative (NED convention)
            idx = idx + 1;
        end
    end
end
