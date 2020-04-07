

clear all; close all; clc;
%state = "time_ms, lin_x_pos, lin_y_pos, lin_z_pos, lin_x_vel, lin_y_vel, lin_z_vel, "
% ang_h_pos, ang_p_pos, ang_s_pos, ang_h_vel, ang_p_vel, ang_s_vel"

%meas_csvlog << "time_ms, meas_time_ms, frame_id, lin_x_m, lin_y_m, lin_z_m, ang_h_rad, ang_p_rad, ang_s_rad, disc_index, player" << endl;

M_state = csvread('~/disc_vision_deluxe/DiscVisionDeluxe/dvd_DvisEst/state.csv', 1, 0);
M_meas  = csvread('~/disc_vision_deluxe/DiscVisionDeluxe/dvd_DvisEst/meas.csv', 1, 0);

% parse state
time_ms_state   = M_state(:, 1);
lin_xyz_pos_state = [M_state(:, 2), M_state(:, 3), M_state(:, 4)];
lin_xyz_vel_state = [M_state(:, 5), M_state(:, 6), M_state(:, 7)];
ang_hps_pos_state = [M_state(:, 8), M_state(:, 9), M_state(:, 10)];
ang_hps_vel_state = [M_state(:, 11), M_state(:, 12), M_state(:, 13)];

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
dtimemeas = mean(time_ms_meas - meas_time_ms_meas);


% find first index for non-zero state
nonzero_idx = (time_ms_state > 0);
t_start = time_ms_state(nonzero_idx);
t_start = t_start(1)

waitidx = figure; hold on;
%plot(time_ms_state, lin_xyz_pos_state, '-')

plot(meas_time_ms_meas + dtimemeas, lin_xyz_pos_meas, '.')
reset_colours
plot(time_ms_meas, lin_xyz_pos_meas, 'o')
grid on;


waitfor(waitidx)
%pause;






