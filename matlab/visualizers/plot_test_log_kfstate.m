

clear all; close all; clc;
%state = "time_ms, lin_x_pos, lin_y_pos, lin_z_pos, lin_x_vel, lin_y_vel, lin_z_vel, "
% ang_h_pos, ang_p_pos, ang_s_pos, ang_h_vel, ang_p_vel, ang_s_vel"

%meas_csvlog << "time_ms, meas_time_ms, frame_id, lin_x_m, lin_y_m, lin_z_m, ang_h_rad, ang_p_rad, ang_s_rad, disc_index, player" << endl;

M_state     = csvread('~/disc_vision_deluxe/DiscVisionDeluxe/dvd_DvisEst/state.csv', 1, 0);
M_state_out = csvread('~/disc_vision_deluxe/DiscVisionDeluxe/dvd_DvisEst/state_out.csv', 1, 0);
M_meas      = csvread('~/disc_vision_deluxe/DiscVisionDeluxe/dvd_DvisEst/meas.csv', 1, 0);

% parse state
time_ms_state   = M_state(:, 1);
lin_xyz_pos_state = [M_state(:, 2), M_state(:, 3), M_state(:, 4)];
lin_xyz_vel_state = [M_state(:, 5), M_state(:, 6), M_state(:, 7)];
ang_hps_pos_state = [M_state(:, 8), M_state(:, 9), M_state(:, 10)];
ang_hps_vel_state = [M_state(:, 11), M_state(:, 12), M_state(:, 13)];
lin_xyz_pos_var_state = [M_state(:, 14), M_state(:, 15), M_state(:, 16)];
lin_xyz_vel_var_state = [M_state(:, 17), M_state(:, 18), M_state(:, 19)];
ang_hps_pos_var_state = [M_state(:, 20), M_state(:, 21), M_state(:, 22)];
ang_hps_vel_var_state = [M_state(:, 23), M_state(:, 24), M_state(:, 25)];

% parse state out
out_time_ms_state   = M_state_out(:, 1);
out_lin_xyz_pos_state = [M_state_out(:, 2), M_state_out(:, 3), M_state_out(:, 4)];
out_lin_xyz_vel_state = [M_state_out(:, 5), M_state_out(:, 6), M_state_out(:, 7)];
out_ang_hps_pos_state = [M_state_out(:, 8), M_state_out(:, 9), M_state_out(:, 10)];
out_ang_hps_vel_state = [M_state_out(:, 11), M_state_out(:, 12), M_state_out(:, 13)];
out_lin_xyz_pos_var_state = [M_state_out(:, 14), M_state_out(:, 15), M_state_out(:, 16)];
out_lin_xyz_vel_var_state = [M_state_out(:, 17), M_state_out(:, 18), M_state_out(:, 19)];
out_ang_hps_pos_var_state = [M_state_out(:, 20), M_state_out(:, 21), M_state_out(:, 22)];
out_ang_hps_vel_var_state = [M_state_out(:, 23), M_state_out(:, 24), M_state_out(:, 25)];

% parse meas
time_ms_meas        = M_meas(:, 1);
meas_time_ms_meas   = M_meas(:, 2);
frame_f_meas        = M_meas(:, 3);
lin_xyz_pos_meas    = [M_meas(:, 4), M_meas(:, 5), M_meas(:, 6)];
ang_hps_pos_meas    = [M_meas(:, 7), M_meas(:, 8), M_meas(:, 9)];
disc_index_meas     = M_meas(:, 10);
player              = M_meas(:, 11);
filt_active         = M_meas(:, 12);


% get delta between meas and state times for better meas plotting (need to sort this)
%dtimemeas = mean(time_ms_meas - meas_time_ms_meas);


% find first index for non-zero state
nonzero_idx = (time_ms_state > 0);
t_start = time_ms_state(nonzero_idx);
t_start = t_start(1)

waitidx = figure; hold on;
plot(time_ms_state, lin_xyz_pos_state, '-', 'LineWidth', 2)
reset_colours
plot(out_time_ms_state(1), out_lin_xyz_pos_state(1, :), 'p', 'LineWidth', 2, 'MarkerSize', 15)
reset_colours
plot(meas_time_ms_meas, lin_xyz_pos_meas, 'o', 'MarkerSize', 3)
legend('X', 'Y', 'Z')
grid on;
title('Lin Pos Meas and State')
fig=gcf;
fig.Units='normalized';
fig.OuterPosition=[0 1 0.4 0.5];

figure; hold on;
plot(time_ms_state, lin_xyz_vel_state, '-')
reset_colours
plot(out_time_ms_state(1), out_lin_xyz_vel_state(1, :), 'p', 'LineWidth', 2, 'MarkerSize', 15)
reset_colours
plot(meas_time_ms_meas(2:end), diff(lin_xyz_pos_meas)./repmat(diff(meas_time_ms_meas*0.001), 1, 3), '.')
%reset_colours
%plot(time_ms_state(2:end), diff(lin_xyz_pos_state)./repmat(diff(time_ms_state*0.001), 1, 3), '--')
legend('X', 'Y', 'Z')
grid on;
title('Lin Vel State')
ylim([-30, 30])
fig=gcf;
fig.Units='normalized';
fig.OuterPosition=[0 0 0.4 0.5];

figure; hold on;
plot(time_ms_state, ang_hps_pos_state, '-', 'LineWidth', 2)
reset_colours
plot(out_time_ms_state(1), out_ang_hps_pos_state(1, :), 'p', 'LineWidth', 2, 'MarkerSize', 15)
reset_colours
plot(meas_time_ms_meas, ang_hps_pos_meas, 'o', 'MarkerSize', 3)
legend('HYZER', 'PITCH', 'SPIN')
grid on;
title('Ang Pos Meas and State')
fig=gcf;
fig.Units='normalized';
fig.OuterPosition=[0.4 1 0.4 0.5];

figure; hold on;
plot(time_ms_state, ang_hps_vel_state, '-')
reset_colours
plot(out_time_ms_state(1), out_ang_hps_vel_state(1, :), 'p', 'LineWidth', 2, 'MarkerSize', 15)
reset_colours
dang = wrap2pi(diff(ang_hps_pos_meas));
plot(meas_time_ms_meas(2:end), dang./repmat(diff(meas_time_ms_meas*0.001), 1, 3), '.')
legend('HYZER', 'PITCH', 'SPIN')
grid on;
title('Ang Vel State')
ylim([-120, 120])
fig=gcf;
fig.Units='normalized';
fig.OuterPosition=[0.4 0 0.4 0.5];

figure; hold on;
plot(time_ms_state, lin_xyz_pos_var_state)
plot(time_ms_state, ang_hps_pos_var_state)
legend('X', 'Y', 'Z', 'HYZER', 'PITCH', 'SPIN')
title('Lin and Ang Pos Variance')
fig=gcf;
fig.Units='normalized';
fig.OuterPosition=[0.8 1 0.2 0.5];

figure; hold on;
plot(time_ms_state, lin_xyz_vel_var_state)
plot(time_ms_state, ang_hps_vel_var_state)
legend('X', 'Y', 'Z', 'HYZER', 'PITCH', 'SPIN')
title('Lin and Ang Vel Variance')
fig=gcf;
fig.Units='normalized';
fig.OuterPosition=[0.8 0 0.2 0.5];


%waitfor(waitidx)
%pause;






