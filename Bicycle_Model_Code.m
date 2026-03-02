clear
%% DLC 50 km/h
load('DLC_50kph.mat');

%% DLC 100 km/h
%load('DLC_100kph.mat');

% Convert vehicle speed to m/s
x_dot_ts = timeseries(Vx_kph.Data / 3.6, Vx_kph.Time);

% Convert steering to radians
steer_ts = timeseries(front_wheel_steer_deg.Data * pi/180, front_wheel_steer_deg.Time);

out = sim("Bicycle_model_1.slx");

% Get time and data
t = out.y_ddot_out.Time;
y_ddot = out.y_ddot_out.Data;

t_yaw = out.yaw_ddot_out.Time;
yaw_acc = out.yaw_ddot_out.Data;

t_yaw_rate = out.yaw_dot_out.Time;
yaw_rate = out.yaw_dot_out.Data;

t_yaw_rate_DE = out.yaw_dot_out_DE.Time;
yaw_rate_DE = out.yaw_dot_out_DE.Data;


t_y_dot = out.y_dot_out.Time;
y_dot = out.y_dot_out.Data;

t_y_dot_DE = out.y_dot_out_DE.Time;
y_dot_DE = out.y_dot_out_DE.Data;

% Plot yaw rate vs yaw rate DE
figure
plot(t_yaw_rate, yaw_rate,'LineWidth', 1.5)
hold on
plot(t_yaw_rate_DE, yaw_rate_DE,'LineWidth', 1.5)
hold off

xlabel('Time [s]')
ylabel('Yaw Rate [rad/s]')
title('Yaw Rate Comparison (100 km/h DLC)')
legend('Yaw Rate from Block', 'Yaw Rate from Differential Equations')
grid on

% Plot y_dot vs y_dot DE
figure
plot(t_y_dot, y_dot,'LineWidth', 1.5)
hold on
plot(t_y_dot_DE, y_dot_DE,'LineWidth', 1.5)
hold off

xlabel('Time [s]')
ylabel('Lateral Velocity [m/s]')
title('Lateral Velocity Comparison (50 km/h DLC)')
legend('Lateral Velocity from Block', 'Lateral Velocity from Differential Equations')
grid on


% Plot lateral acceleration
figure
plot(t, y_ddot,'LineWidth', 1.5)
xlabel('Time [s]')
ylabel('Lateral Acceleration [m/s^2]')
title('Lateral Acceleration from Differential Equations (100 km/h)')

% Plot yaw acceleration
figure
plot(t_yaw, yaw_acc,'LineWidth', 1.5)
xlabel('Time [s]')
ylabel('Yaw Acceleration [rad/s^2]')
title('Yaw Acceleration from Differential Equations (100 km/h)')

mean(Vx_kph.Data(end-200:end))
std(Vx_kph.Data(end-200:end))