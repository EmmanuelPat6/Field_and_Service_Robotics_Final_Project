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


%% Momentum-Based Estimator
r = 2;
c0 = 10;
k0 = c0;    % k0/c0 = 1
s = tf('s');
G = k0^r/(s+c0)^r;  % rth-Order Transfer Function

K = zeros(r,1);
c = cell2mat(G.Denominator);    % G.Denominator is a cell array with the
                    % vector of the coefficients of the denominator of G
c = c(2:end);   % No s^r coefficient which is always 1
                % In this way the first elementi is c_{r-1}

% Ki Computation
aux = 1;    % Auxiliary Variable
% From 1 to r (it is like from 0 to r-1 in the formula)
for j = 1:r
    % prod_{i=j+1}^{r} = c(j)
    K(j) = c(j)/aux;
    aux = aux*K(j);
end

K = flip(K);    % Reverse the order because c(1)=c_{r-1}


%% External Disturbances
Dxy = 1;    % Along x-axis
Dyaw = -0.8;    % Around Yaw Axis
Dz = -1;    % Along z-axis

% For Simulink
f_e = [Dxy Dxy Dz];
tau_e = [0 0 Dyaw];


%% Initial and Final Conditions
x0 = 0; xf = 0; dot_x0 = 0; dot_xf = 0; ddot_x0 = 0; ddot_xf = 0;
y0 = 0; yf = 20; dot_y0 = 0; dot_yf = 0; ddot_y0 = 0; ddot_yf = 0;
z0 = -7; zf = -7; dot_z0 = 0; dot_zf = 0; ddot_z0 = 0; ddot_zf = 0;
psi0 = 0;
psif = 0; dot_psi0 = 0; dot_psif = 0; ddot_psi0 = 0; ddot_psif = 0;

pos_0 = [x0 y0 z0];
ori_0 = [0 0 0];
lin_vel_0 = [0; 0; 0];
w_bb_0 = [0; 0; 0];

%pos_goal = [xf yf zf];
% Waypoints
% pos_1 = [0 20 -7];
% pos_2 = [20 20 -7];
% pos_3 = [20 0 -7];
% Waypoints = [pos_1; pos_2; pos_3];

Waypoints = [ 0  20  -6;
              8  28  -8;
             22  27  -6;
             26  20  -7;
             26   2  -9;
             12   6  -7;
             11  15 -10;
             11  25  -6;
             17  11  -3;
              4   0  -7];


%% Obstacles
% ObstaclePositions = [10 -1; 20 10; 10 20];
% ObstacleHeights = [15; 15; 10];
% ObstacleWidths = [3; 4; 2];

ObstaclePositions = [10 -1;
                     20 10;
                     10 20;
                     5 5;
                     15 0;
                     25 5;
                     5 25;
                     18 22;
                     28 15
                     11 10;
                     15 16;
                     23 18;
                     24 25;
                     3 17];

ObstacleHeights =   [13;
                     7;
                     10;
                     4;
                     14;
                     10;
                     5;
                     13;
                     6;
                     15;
                     9;
                     10;
                     8;
                     5];

ObstacleWidths =    [3;
                     4;
                     2;
                     2.5;
                     3;
                     2.5;
                     2;
                     3.5;
                     3;
                     3.5;
                     2;
                     2;
                     3;
                     3];


%% APF (Artificial Potential Fields) Parameters
% K_a = 1.0;
% K_r = 0.5;
% rho_0 = 3.0;

K_a = 1.25;
K_r = 0.5;
rho_0 = 2.0;
%% Scenario Inizialization

obstacles_points = [];

for i = 1:size(ObstaclePositions, 1)
    points = prism_points_gen(ObstaclePositions(i,:), ObstacleHeights(i), ObstacleWidths(i));
    obstacles_points = [obstacles_points, points];  % Concatenazione colonne
end

Scenario = setup_uav_scenario_waypoints(pos_0, ori_0, ObstaclePositions, ObstacleHeights, ObstacleWidths, Waypoints, Ts);
show3D(Scenario);
exportgraphics(gcf, 'Scenario.pdf');

% Obstacles Points Visualization in their own Frame
% figure;
% scatter3(obstacles_points(1, :), obstacles_points(2, :), obstacles_points(3, :), 5, 'filled');
% axis equal;
% grid on;
% xlabel('X'); ylabel('Y'); zlabel('Z');


%% Simulink

out = sim("geometric_control.slx");

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
exportgraphics(plot_1, 'Position_x_y_z_Errors_Geometric_Control_APF.pdf');


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
exportgraphics(plot_2, 'Velocity_x_y_z_Errors_Geometric_Control_APF.pdf');


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
exportgraphics(plot_3, 'uT_Geometric_Control_APF.pdf');

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
exportgraphics(plot_4, 'tau_b_x_y_z_Geometric_Control_APF.pdf');



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
exportgraphics(gcf, 'Scenario_and_Trajectory.pdf');
view(0, 90)
exportgraphics(gcf, 'Scenario_and_Trajectory_Top_View.pdf');


%% Desired Values
plot_5 = figure('Name', 'Desired Position', 'Position', [10 10 900 350]);
plot(out.position_des.Time,position_des_x,'-', 'LineWidth', 2.5)
hold on
plot(out.position_des.Time,position_des_y,'-','Color', [0.9, 0, 0], 'LineWidth', 2.5)
plot(out.position_des.Time,position_des_z,'Color', [0.9, 0.7, 0.15], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 26)
ylabel('$p_d [m]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, out.position_des.Time(end)])
ylim([min([position_des_y, position_des_x, position_des_z])-2, ...
      max([position_des_y, position_des_x, position_des_z])+2])
legend({'$x(t)$', '$y(t)$', '$z(t)$'}, 'Interpreter', 'latex', 'Location', 'eastoutside','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_5, 'Desired_Position_Geometric_Control_APF.pdf');


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
ylabel('$\dot{p}_d [m/s]$', 'Interpreter', 'latex', 'FontSize', 26)
set(gca, 'FontSize', 26);
grid on
box on
xlim([0, out.linear_velocity_des.Time(end)])
ylim([min([linear_velocity_des_y, linear_velocity_des_x, linear_velocity_des_z])-0.2, ...
      max([linear_velocity_des_y, linear_velocity_des_x, linear_velocity_des_z])+0.2])
legend({'$\dot{x}(t)$', '$\dot{y}(t)$', '$\dot{z}(t)$'}, 'Interpreter', 'latex', 'Location', 'eastoutside','FontSize', 30)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_6, 'Desired_Velocity_Geometric_Control_APF.pdf');


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
exportgraphics(plot_7, 'F_att_Geometric_Control_APF.pdf');


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
exportgraphics(plot_8, 'F_rep_Geometric_Control_APF.pdf');





%% Estimator Plot
% Remember that there are two supposing disturbances during the flight:
% 1 N along the x- and y- axis of the World Frame
% -1 N along the z- axis of the World Frame
% -0.4 Nm around the Yaw Axis
wrench_hat = out.wrench_hat.Data;
t = out.wrench_hat.Time;
plot_9 = figure('Name', 'f_x_hat and disturbance', 'Position', [10 10 900 350]);
plot(t,wrench_hat(1,:),'Color', [0.9, 0, 0], 'LineWidth', 2.5)
hold on
plot(t,out.Dx.Data,'k--','Color',[0.9, 0.7, 0.15], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 24)
ylabel('$F[N]$', 'Interpreter', 'latex', 'FontSize', 24)
set(gca, 'FontSize', 24);
grid on
box on
xlim([0, t(end)])
ylim([min(wrench_hat(1,:))-0.1, max(wrench_hat(1,:))+0.1])
legend({'$\hat{f}_x$', 'Dx'}, 'Interpreter', 'latex', 'Location', 'eastoutside','FontSize', 24)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_9, 'f_hat_x_and_disturbace.pdf');

plot_10 = figure('Name', 'f_y_hat and disturbance', 'Position', [10 10 900 350]);
plot(t,wrench_hat(2,:),'Color', [0.9, 0, 0], 'LineWidth', 2.5)
hold on
plot(t,out.Dy.Data,'k--','Color',[0.9, 0.7, 0.15], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 24)
ylabel('$F[N]$', 'Interpreter', 'latex', 'FontSize', 24)
set(gca, 'FontSize', 24);
grid on
box on
xlim([0, t(end)])
ylim([min(wrench_hat(2,:))-0.1, max(wrench_hat(2,:))+0.1])
legend({'$\hat{f}_y$', 'Dy'}, 'Interpreter', 'latex', 'Location', 'eastoutside','FontSize', 24)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_10, 'f_hat_y_and_disturbace.pdf');

plot_11 = figure('Name', 'f_z_hat', 'Position', [10 10 900 350]);
plot(t,wrench_hat(3,:),'Color', [0.9, 0, 0], 'LineWidth', 2.5)
hold on
plot(t,out.Dz.Data,'k--','Color',[0.9, 0.7, 0.15], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 24)
ylabel('$F[N]$', 'Interpreter', 'latex', 'FontSize', 24)
set(gca, 'FontSize', 24);
grid on
box on
xlim([0, t(end)])
ylim([min(wrench_hat(3,:))-0.1, max(wrench_hat(3,:))+0.1])
legend({'$\hat{f}_z$','Dz'}, 'Interpreter', 'latex', 'Location', 'best','FontSize', 24)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_11, 'f_hat_z_and_disturbance.pdf');

% plot_12 = figure('Name', 'tau_x_hat', 'Position', [10 10 900 350]);
% plot(t,wrench_hat(4,:),'Color', [0.9, 0, 0], 'LineWidth', 2.5)
% xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 24)
% ylabel('$\tau[Nm]$', 'Interpreter', 'latex', 'FontSize', 24)
% set(gca, 'FontSize', 24);
% grid on
% box on
% xlim([0, t(end)])
% ylim([min(wrench_hat(4,:))-0.01, max(wrench_hat(4,:))+0.01])
% legend({'$\hat{\tau}_x$'}, 'Interpreter', 'latex', 'Location', 'best','FontSize', 30)
% set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
% annotation('rectangle',[0 0 1 1],'Color','w');
% exportgraphics(plot_12, 'tau_hat_x.pdf');
% 
% plot_13 = figure('Name', 'tau_y_hat', 'Position', [10 10 900 350]);
% plot(t,wrench_hat(5,:),'Color', [0.9, 0, 0], 'LineWidth', 2.5)
% xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 24)
% ylabel('$\tau[Nm]$', 'Interpreter', 'latex', 'FontSize', 24)
% set(gca, 'FontSize', 24);
% grid on
% box on
% xlim([0, t(end)])
% ylim([min(wrench_hat(5,:))-0.01, max(wrench_hat(5,:))+0.01])
% legend({'$\hat{\tau}_y$'}, 'Interpreter', 'latex', 'Location', 'best','FontSize', 30)
% set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
% annotation('rectangle',[0 0 1 1],'Color','w');
% exportgraphics(plot_13, 'tau_hat_y.pdf');

plot_14 = figure('Name', 'tau_z_hat and disturbance', 'Position', [10 10 900 350]);
plot(t,wrench_hat(6,:),'Color', [0.9, 0, 0], 'LineWidth', 2.5)
hold on
plot(t,out.Dyaw.Data,'k--','Color',[0.9, 0.7, 0.15], 'LineWidth', 2.5)
xlabel('$t [s]$', 'Interpreter', 'latex', 'FontSize', 24)
ylabel('$\tau[Nm]$', 'Interpreter', 'latex', 'FontSize', 24)
set(gca, 'FontSize', 24);
grid on
box on
xlim([0, t(end)])
ylim([min(wrench_hat(6,:))-0.01, max(wrench_hat(6,:))+0.01])
legend({'$\hat{\tau}_z$', 'DYaw'}, 'Interpreter', 'latex', 'Location', 'eastoutside','FontSize', 24)
set(gca, 'InnerPosition', [0.1400 0.32 0.82 0.55])
annotation('rectangle',[0 0 1 1],'Color','w');
exportgraphics(plot_14, 'tau_hat_z_and_disturbance.pdf');
