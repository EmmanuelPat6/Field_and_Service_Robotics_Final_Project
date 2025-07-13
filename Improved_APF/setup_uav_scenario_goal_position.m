function Scenario = setup_uav_scenario(pos_0, ori_0, ObstaclePositions, ObstacleHeights, ObstaclesWidths, pos_goal, Ts)
    % Function to configure a UAV scenario in 3D space with static obstacles and initial UAV state.
    %
    % Inputs:
    %   - pos_0: 1x3 vector specifying the initial position of the UAV [x, y, z]
    %   - ori_0: 1x3 vector with the UAV's initial orientation in roll-pitch-yaw (RPY) angles [rad]
    %   - ObstaclePositions: Nx2 matrix, each row defines the [x, y] center of a square-based obstacle
    %   - ObstacleHeights: Nx1 vector with the height (z-dimension) of each obstacle
    %   - ObstaclesWidths: Nx1 vector with the side length of the square base for each obstacle
    %   - pos_goal: 1x3 vecotr specifyin the position goal of the UAV [x, y, z]
    %   - Ts: sampling time [s] for scenario updates
    %
    % Output:
    %   - Scenario: a configured uavScenario object with obstacles and UAV

    % Create the UAV scenario with a specified update rate and reference origin
    Scenario = uavScenario("UpdateRate", 1/Ts, "ReferenceLocation", [0 0 0]);
    
    % Marker to indicate the Starting Position of the UAV
    addMesh(Scenario,"cylinder",{[0 0 1] [0 .01]},[0 1 0]);

    % Loop through all obstacles and add them to the scenario as square prisms (polygonal mesh)
    for i = 1:size(ObstaclePositions,1)
        % Define the 4 vertices of the square base centered at ObstaclePositions(i,:)
        % The obstacle is modeled as a vertical prism from z=0 to z=ObstacleHeights(i)
        base_vertices = [ObstaclePositions(i,1) - ObstaclesWidths(i)/2, ObstaclePositions(i,2) - ObstaclesWidths(i)/2;
                         ObstaclePositions(i,1) + ObstaclesWidths(i)/2, ObstaclePositions(i,2) - ObstaclesWidths(i)/2;
                         ObstaclePositions(i,1) + ObstaclesWidths(i)/2, ObstaclePositions(i,2) + ObstaclesWidths(i)/2;
                         ObstaclePositions(i,1) - ObstaclesWidths(i)/2, ObstaclePositions(i,2) + ObstaclesWidths(i)/2];
        
        % Add the obstacle as a 3D polygonal mesh, extruded from 0 to the specified height
        % The color is set to a shade of gray [0.651 0.651 0.651]
        addMesh(Scenario, "polygon", {base_vertices, [0 ObstacleHeights(i)]}, 0.651*ones(1,3));
    end

    % Add the UAV platform to the scenario
    platUAV = uavPlatform("UAV", Scenario, ...
        "ReferenceFrame", "NED", ...                      % North-East-Down convention
        "InitialPosition", pos_0, ...                     % Initial position in [x, y, z]
        "InitialOrientation", eul2quat(ori_0));           % Convert RPY to quaternion for orientation
    
    % Update the mesh of the UAV: use a quadrotor model with scale factor 1.2
    % Color is set to blue [0 0 1] and a rotation of pi around Z is applied
    updateMesh(platUAV, "quadrotor", {1.2}, [0 0 1], eul2tform([0 0 pi]));
    
    addMesh(Scenario,"cylinder",{[pos_goal(2) pos_goal(1) 1] [0 0.1]},[1 0 0]);

    % Visualize the scenario with 3D rendering
    show3D(Scenario);

    hold on
    plot3([pos_0(1,2); pos_goal(:,2)],[pos_0(1,2); pos_goal(:,1)],[-pos_0(1,3); -pos_goal(:,3)],"--m")
    lgd = legend(["Start Position","Obstacle","Position Goal", "Direct Path"]);
    lgd.AutoUpdate = 'off';



    % Finalize scenario setup
    setup(Scenario);
end
