%% UAV Obstacle Avoidance with Geometric Control and APF

%% Inizialization
clear all
close all
clc

%% UAV Parameters
mass = 1.2;
Ib = diag([1.2416 1.2416 2*1.2416]);
g = 9.81;

%% Sampling Time
Ts = 0.01;

%% Initial and Final Conditions
x0 = 0; xf = -3.0; dot_x0 = 0; dot_xf = 0; ddot_x0 = 0; ddot_xf = 0;
y0 = 0; yf = 10; dot_y0 = 0; dot_yf = 0; ddot_y0 = 0; ddot_yf = 0;
z0 = -7; zf = -7; dot_z0 = 0; dot_zf = 0; ddot_z0 = 0; ddot_zf = 0;
psi0 = 0;
psif = 0; dot_psi0 = 0; dot_psif = 0; ddot_psi0 = 0; ddot_psif = 0;

pos_0 = [x0 y0 z0];
ori_0 = [0 0 0];
lin_vel_0 = [0; 0; 0];
w_bb_0 = [0; 0; 0];

pos_goal = [xf yf zf];
% pos_goal = [-1 10 -7];

%% Obstacles
ObstaclePositions = [10 -1];
% ObstaclePositions = [10 0];
ObstacleHeights = 15;
ObstacleWidths = 3;

%% APF (Artificial Potential Fields) Parameters
K_a = 1.0;
K_r = 0.5;
rho_0 = 2.0;
K_v = 0.1;

%% Scenario Inizialization

obstacles_points = [];

for i = 1:size(ObstaclePositions, 1)
    points = prism_points_gen(ObstaclePositions(i,:), ObstacleHeights(i), ObstacleWidths(i));
    obstacles_points = [obstacles_points, points];  % Concatenazione colonne
end

Scenario = setup_uav_scenario_goal_position(pos_0, ori_0, ObstaclePositions, ObstacleHeights, ObstacleWidths, pos_goal, Ts);
show3D(Scenario);

% Points Visualization
% figure;
% scatter3(obstacles_points(1, :), obstacles_points(2, :), obstacles_points(3, :), 5, 'filled');
% xlabel('X'); ylabel('Y'); zlabel('Z');
% view(3);

%% Simulink

out = sim("geometric_control_apf_goal_position.slx");


%% Plot From Simulink

%% Position and Velocity Errors
plot_1 = figure('Name', 'Position x, y, z Errors', 'Position', [10 10 900 350]);
plot(out.err_p.Time, out.err_p.Data(:,1), '-', 'LineWidth', 2.5)
hold on
plot(out.err_p.Time, out.err_p.Data(:,2), '-', 'Color', [0.9, 0, 0], 'LineWidth', 2.5)
plot(out.err_p.Time, out.err_p.Data(:,3), '-', 'Color', [0.9, 0.7, 0.15], 'LineWidth', 2.5)
xlabel('$t\ [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$e\ [m]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, out.err_p.Time(end)])
ylim([ ...
    min([out.err_p.Data(:,1); out.err_p.Data(:,2); out.err_p.Data(:,3)]), ...
    max([out.err_p.Data(:,1); out.err_p.Data(:,2); out.err_p.Data(:,3)]) ...
]);
legend({'$e_{p_x}(t)$', '$e_{p_y}(t)$', '$e_{p_z}(t)$'}, ...
    'Interpreter', 'latex', 'Location', 'eastoutside', 'FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle', [0 0 1 1], 'Color', 'w');
exportgraphics(plot_1, 'Position_x_y_z_Errors_Geometric_Control_APF_Position_Goal.pdf');


plot_2 = figure('Name', 'Velocity x, y, z Errors', 'Position', [10 10 900 350]);
plot(out.dot_err_p.Time,out.dot_err_p.Data(:,1),'-', 'LineWidth', 2.5)
hold on
plot(out.dot_err_p.Time,out.dot_err_p.Data(:,2),'-','Color', [0.9, 0, 0], 'LineWidth', 2.5)
plot(out.dot_err_p.Time,out.dot_err_p.Data(:,3),'-','Color', [0.9, 0.7, 0.15], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$e [m/s]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, out.dot_err_p.Time(end)])
ylim([min([out.dot_err_p.Data(:,1); out.dot_err_p.Data(:,2); out.dot_err_p.Data(:,3)]), ...
      max([out.dot_err_p.Data(:,1); out.dot_err_p.Data(:,2); out.dot_err_p.Data(:,3)])])
legend({'$e_{\dot{p}_x}(t)$', '$e_{\dot{p}_y}(t)$', '$e_{\dot{p}_z}(t)$'}, 'Interpreter', 'latex', 'Location', 'eastoutside','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_2, 'Velocity_x_y_z_Errors_Geometric_Control_APF_Position_Goal.pdf');


%% Thrust and Drag Moment
plot_3 = figure('Name', 'Thrust', 'Position', [10 10 900 350]);
plot(out.uT.Time,out.uT.Data,'-', 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$F [N]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, out.uT.Time(end)])
ylim([min([out.uT.Data]), max([out.uT.Data])])
legend({'$u_T(t)$'}, 'Interpreter', 'latex', 'Location', 'eastoutside','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_3, 'uT_Geometric_Control_APF_Position_Goal.pdf');

plot_4 = figure('Name', 'Drag Moment x, y, z', 'Position', [10 10 900 350]);
plot(out.tau_b.Time,out.tau_b.Data(:,1),'-', 'LineWidth', 2.5)
hold on
plot(out.tau_b.Time,out.tau_b.Data(:,2),'-','Color', [0.9, 0, 0], 'LineWidth', 2.5)
plot(out.tau_b.Time,out.tau_b.Data(:,3),'Color', [0.9, 0.7, 0.15], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$\tau_b [Nm]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, out.tau_b.Time(end)])
ylim([min([out.tau_b.Data(:,1); out.tau_b.Data(:,2); out.tau_b.Data(:,3)]), ...
    max([out.tau_b.Data(:,1); out.tau_b.Data(:,2); out.tau_b.Data(:,3)])])
legend({'$\tau_x(t)$', '$\tau_y(t)$', '$\tau_z(t)$'}, 'Interpreter', 'latex', 'Location', 'eastoutside','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_4, 'tau_b_x_y_z_Geometric_Control_APF_Position_Goal.pdf');



%% Scenario and Trajectory in 3D
figure
%Scenario
show3D(Scenario)
position_des = squeeze(out.position_des.Data);
position_des_x = position_des(1, :);
position_des_y = position_des(2, :);
position_des_z = position_des(3, :);
hold on
% 3D Trajectory
% Remember the Different Reference Frames x -> y, y -> x and z -> -z
plot3(position_des_y, position_des_x, -position_des_z, 'Color', [1 0.5 0], 'LineWidth', 1.5)
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
exportgraphics(gcf, 'Scenario_and_Trajectory_Position_Goal.pdf');
view(0, 90)
exportgraphics(gcf, 'Scenario_and_Trajectory_Top_View_Position_Goal.pdf');


%% Desired Values
plot_5 = figure('Name', 'Desired Position', 'Position', [10 10 900 350]);
plot(out.position_des.Time,position_des_x,'-', 'LineWidth', 2.5)
hold on
plot(out.position_des.Time,position_des_y,'-','Color', [0.9, 0, 0], 'LineWidth', 2.5)
plot(out.position_des.Time,position_des_z,'Color', [0.9, 0.7, 0.15], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$p_d [Nm]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, out.position_des.Time(end)])
ylim([min([position_des_y, position_des_x, position_des_z])-2, ...
      max([position_des_y, position_des_x, position_des_z])+2])
legend({'$x(t)$', '$y(t)$', '$z(t)$'}, 'Interpreter', 'latex', 'Location', 'eastoutside','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_5, 'Desired_Position_Geometric_Control_APF_Position_Goal.pdf');


linear_velocity_des = squeeze(out.linear_velocity_des.Data);
linear_velocity_des_x = linear_velocity_des(1, :);
linear_velocity_des_y = linear_velocity_des(2, :);
linear_velocity_des_z = linear_velocity_des(3, :);

plot_6 = figure('Name', 'Desired Velocity', 'Position', [10 10 900 350]);
plot(out.linear_velocity_des.Time,linear_velocity_des_x,'-', 'LineWidth', 2.5)
hold on
plot(out.linear_velocity_des.Time,linear_velocity_des_y,'-','Color', [0.9, 0, 0], 'LineWidth', 2.5)
plot(out.linear_velocity_des.Time,linear_velocity_des_z,'Color', [0.9, 0.7, 0.15], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$\dot{p}_d [Nm]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, out.linear_velocity_des.Time(end)])
ylim([min([linear_velocity_des_y, linear_velocity_des_x, linear_velocity_des_z])-0.2, ...
      max([linear_velocity_des_y, linear_velocity_des_x, linear_velocity_des_z])+0.2])
legend({'$\dot{x}(t)$', '$\dot{y}(t)$', '$\dot{z}(t)$'}, 'Interpreter', 'latex', 'Location', 'eastoutside','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_6, 'Desired_Velocity_Geometric_Control_APF_Position_Goal.pdf');


%% Attractive Force
F_att = squeeze(out.F_att.Data);
F_att_x = F_att(1, :);
F_att_y = F_att(2, :);
F_att_z = F_att(3, :);

plot_7 = figure('Name', 'Attractive Force', 'Position', [10 10 900 350]);
plot(out.F_att.Time,F_att_x,'-', 'LineWidth', 2.5)
hold on
plot(out.F_att.Time,F_att_y,'-','Color', [0.9, 0, 0], 'LineWidth', 2.5)
plot(out.F_att.Time,F_att_z,'Color', [0.9, 0.7, 0.15], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$F [N]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, out.F_att.Time(end)])
ylim([min([F_att_y, F_att_x, F_att_z])-0.2, ...
      max([F_att_y, F_att_x, F_att_z])+0.2])
legend({'$F_{att_x}(t)$', '$F_{att_y}(t)$', '$F_{att_z}(t)$'}, 'Interpreter', 'latex', 'Location', 'eastoutside','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_7, 'F_att_Geometric_Control_APF_Position_Goal.pdf');


%% Repulsive Force
F_rep = squeeze(out.F_rep.Data);
F_rep_x = F_rep(1, :);
F_rep_y = F_rep(2, :);
F_rep_z = F_rep(3, :);

plot_8 = figure('Name', 'Repulsive Force', 'Position', [10 10 900 350]);
plot(out.F_rep.Time,F_rep_x,'-', 'LineWidth', 2.5)
hold on
plot(out.F_rep.Time,F_rep_y,'-','Color', [0.9, 0, 0], 'LineWidth', 2.5)
plot(out.F_rep.Time,F_rep_z,'Color', [0.9, 0.7, 0.15], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$F [N]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, out.F_rep.Time(end)])
ylim([min([F_rep_y, F_rep_x, F_rep_z])-0.2, ...
      max([F_rep_y, F_rep_x, F_rep_z])+0.2])
legend({'$F_{rep_x}(t)$', '$F_{rep_y}(t)$', '$F_{rep_z}(t)$'}, 'Interpreter', 'latex', 'Location', 'eastoutside','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_8, 'F_rep_Geometric_Control_APF_Position_Goal.pdf');