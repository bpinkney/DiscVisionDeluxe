function dvd_DvisEst_KF_on_log()
%% define KF and transform funcs

clc;
close all;
clear all;

format long g

plot_to_last_valid = 1;

%meas preprocessing
Ang_hyzer_pitch_spin = @(R00,R01,R02,R12)[asin(R12),asin(R02),atan2(R01,R00)];

%meas update
Kalman_gain = @(S2dm,p11,p21)[p11./(S2dm+p11);p21./(S2dm+p11)];
xpk_update = @(Kk1,Kk2,innovation_dk,dmk,vmk)[dmk+Kk1.*innovation_dk;vmk+Kk2.*innovation_dk]; % innovation = (dk-dmk)
Ppk_update = @(Kk1,Kk2,p11,p12,p21,p22)reshape([-p11.*(Kk1-1.0),p21-Kk2.*p11,-p12.*(Kk1-1.0),p22-Kk2.*p12],[2,2]);

%predict
xmk_predict = @(dt,dk1,vk1)[dk1+dt.*vk1;vk1];
Pmk_predict = @(S2dp,S2vp,dt,p11,p12,p21,p22)reshape([S2dp+p11+dt.*(p12+p21)+dt.^2.*p22,p21+dt.*p22,p12+dt.*p22,S2vp+p22],[2,2]);

LP_FILT = @(var, new_val, N) ((var * N + new_val) / (N + 1));

%% define parameters
% run at 522Hz for now (match camera framerate)
dt_pred = 1.0/522.0;

% fixed meas covar for now
lin_S2dm = 0.01;
ang_S2dm = 0.01;

% process covariance
lin_S2dp = 1.0; %should be a function of dt
lin_S2vp = 200.0; %should be a function of dt

ang_S2dp = 1.0; %should be a function of dt
ang_S2vp = 200.0; %should be a function of dt

% initial covariances values
LIN_POS_VAR_INIT = 1;  % (m)^2
LIN_VEL_VAR_INIT = 50; % (m/s)^2
ANG_POS_VAR_INIT = 1;  % (rad)^2
ANG_VEL_VAR_INIT = 50; % (rad/s)^2
% max time delta between measurements before KF state is invalidated
max_valid_time_s = dt_pred * 3; % (~3 missed samples)

%% load log and determine measurement count
ld = dvd_DvisEst_load_csv_log('/home/bpinkney/disc_vision_deluxe/DiscVisionDeluxe/resources/test_throws/windy_snow0/imgs_groundplane2/csvlog.csv');

ld.unprocessed_measurement = ld.time_s * 0 + 1;

time_end = max(ld.time_s) - min(ld.time_s);
steps = ceil(time_end / dt_pred)

%% define states and covariance matrices
kf.state_valid = zeros(steps, 1);
% linear XYZ
kf.lin_xyz_pos = zeros(steps, 3);
kf.lin_xyz_vel = zeros(steps, 3);
kf.lin_xyz_var = cell(steps, 3); % 2x2 covariance matrix for each

% angular HYZER, PITCH, SPIN
kf.ang_hps_pos = zeros(steps, 3);
kf.ang_hps_vel = zeros(steps, 3);
kf.ang_hps_var = cell(steps, 3);% 2x2 covariance matrix for each

%init covariance matrices to zeros
for i = 1:steps
    for j = 1:3
        kf.lin_xyz_var{i, j} = [0,0;0,0];
        kf.ang_hps_var{i, j} = [0,0;0,0];
    end    
end    

%% Run Kalman Filter
t = 0;
meas_idx = 1;

%queue size to start estimate
init_queue_size = 5;  
init_queue.lin_xyz_pos  = zeros(init_queue_size, 3);
init_queue.ang_hps_pos  = zeros(init_queue_size, 3);
init_queue.queue_time_s = zeros(init_queue_size, 1);
init_queue.queue_count  = 0;
init_queue.ok_to_prime  = 0; % don't allow the queue to prime if the vel variance is too high
init_queue.primed       = 0; % mark whether or not the queue has been consumed to generate the initial estimate

% shift measurement timestamps to zero
ld.time_s = ld.time_s - ld.time_s(1);

t_steps = 0:dt_pred:(dt_pred*steps-dt_pred);

%track lin vel var
mv_var = zeros(steps, 1);

for s = 2:steps
    disp(sprintf('t = %0.5fs, next meas = %0.5f s', t, ld.time_s(meas_idx)));
    % perform update step if required
    if (t > ld.time_s(meas_idx) && ld.unprocessed_measurement(meas_idx) == 1)
        % mark measurement as processed
        ld.unprocessed_measurement(meas_idx) = 0;

        % process measurement into KF measurement
        meas.time_s      = ld.time_s(meas_idx);
        meas.lin_xyz_pos = ld.pos_xyz(meas_idx, :);
        meas.ang_hps_pos = Ang_hyzer_pitch_spin(ld.R00(meas_idx),ld.R01(meas_idx),ld.R02(meas_idx),ld.R12(meas_idx));

        if(init_queue.ok_to_prime == 0)
            % we have just started getting measurements
            % (in c-code, we need to throw out old entries in case of false early detections!)
            % fill the queue, and do not apply a measurement update to the
            % actual states yet

            % add new meas to the queue, 
            % shift elements of the queue over for index fun
            % technically the oldest element is discarded here
            % which would be useful if we were running this awaiting a
            % member-variance check or something
            for i = init_queue_size:-1:2
                init_queue.lin_xyz_pos(i, :) = init_queue.lin_xyz_pos(i-1, :);
                init_queue.ang_hps_pos(i, :) = init_queue.ang_hps_pos(i-1, :);
                init_queue.queue_time_s(i)   = init_queue.queue_time_s(i-1);
            end

            init_queue.lin_xyz_pos(1, :) = meas.lin_xyz_pos;
            init_queue.ang_hps_pos(1, :) = meas.ang_hps_pos;
            init_queue.queue_time_s(1)   = meas.time_s;
            
            % increase queue count
            init_queue.queue_count = init_queue.queue_count + 1;
            if(init_queue.queue_count > init_queue_size)
              init_queue.queue_count = init_queue_size; % old entries will be discarded
            end
            
            vel_var_limit = 1.0;
            max_extra_meas = 30;
            % check for velocity variance to allow queue priming
            if(init_queue.queue_count >= init_queue_size)
              lin_xyz_vel = diff(init_queue.lin_xyz_pos) ./ repmat(diff(init_queue.queue_time_s), 1, 3);
              lin_xyz_vel_mean = mean(lin_xyz_vel);
              lin_xyz_vel_var = (lin_xyz_vel - lin_xyz_vel_mean).^2;
              
              % if all elements in the queue are within the velocity
              % variance limit, we are clear to initialize the Kalman
              % Filter
              % otherwise, allow a prime if we have already seen
              % 'max_extra_meas' measurements
              if(sum(sum(lin_xyz_vel_var > vel_var_limit)) == 0 || meas_idx >= max_extra_meas)
                init_queue.ok_to_prime = 1;
              end
            end   
              
        end

        % check for full, unprimed queue
        if(~init_queue.primed)
            if(init_queue.ok_to_prime > 0)
                % TODO: we probably want to add a velocity variance check
                % before the queue is considered valid!

                % the queue is full, but we haven't initialized our KF
                % filter states yet; use the average of the samples to
                % initialize the filter states.
                % Note: for a queue with larger dts, we can initialize to
                % the oldest entry, then run the measurement update
                % steps using the queued entries (and an appropriate,
                % queue-driven dt for the prediction step between each!)
                % hopefully that is over-kill for our small/fast queue init

                % mark queue as consumed
                init_queue.primed = 1;

                init_queue.init_t = t;
                % linear states
                % filter positions from the queue in with a simple
                % first-order filter
                % start with the oldest value in the queue
                kf.lin_xyz_pos(s, :) = init_queue.lin_xyz_pos(init_queue_size, :);
                for i = (init_queue_size):-1:1
                  kf.lin_xyz_pos(s, :) = LP_FILT(kf.lin_xyz_pos(s, :), init_queue.lin_xyz_pos(i, :), 0);
                end
                
                init_queue.init_lin_pos = kf.lin_xyz_pos(s, :);
                % differentiate elements in the queue to get the initial
                % velocity, just take a straight mean here
                lin_xyz_vel = diff(init_queue.lin_xyz_pos) ./ repmat(diff(init_queue.queue_time_s), 1, 3);
                kf.lin_xyz_vel(s, :) = mean(lin_xyz_vel);
                init_queue.init_lin_vel = kf.lin_xyz_vel(s, :);

                % angular states
                % filter positions from the queue in with a simple
                % first-order filter
                % start with the oldest value in the queue
                kf.ang_hps_pos(s, :) = init_queue.ang_hps_pos(init_queue_size, :);
                for i = (init_queue_size):-1:1
                  for j = 1:3
                    kf.ang_hps_pos(s, j) = wrap2pi(LP_FILT(kf.ang_hps_pos(s, j), init_queue.ang_hps_pos(i, j), 0));
                  end
                end
                
                init_queue.init_ang_pos = kf.ang_hps_pos(s, :);
                
                % differentiate elements in the queue to get the initial
                % velocity, just take a straight mean here
                % don't constrain angular veloicty to +-2*pi!
                % DO wrap the delta pos to +-pi!
                ang_hps_pos_diff = diff(init_queue.ang_hps_pos);
                ang_hps_pos_diff_size = size(ang_hps_pos_diff);
                for i = 1:ang_hps_pos_diff_size(1)
                  for j = 1:ang_hps_pos_diff_size(2)
                    ang_hps_pos_diff(i,j) = wrap2pi(ang_hps_pos_diff(i,j));
                  end
                end  
                ang_hps_vel = (ang_hps_pos_diff ./ repmat(diff(init_queue.queue_time_s), 1, 3));
                kf.ang_hps_vel(s, :) = (mean(ang_hps_vel));
                init_queue.init_ang_vel = kf.ang_hps_vel(s, :);

                % set initial state covariance values
                P_lin_init = ...
                [ ...
                    LIN_POS_VAR_INIT, 0; ...
                    0, LIN_VEL_VAR_INIT ...
                ];
                P_ang_init = ...
                [ ...
                    ANG_POS_VAR_INIT, 0; ...
                    0, ANG_VEL_VAR_INIT ...
                ];
                kf.lin_xyz_var{s, 1} = P_lin_init;
                kf.lin_xyz_var{s, 2} = P_lin_init;
                kf.lin_xyz_var{s, 3} = P_lin_init;
                kf.ang_hps_var{s, 1} = P_ang_init;
                kf.ang_hps_var{s, 2} = P_ang_init;
                kf.ang_hps_var{s, 3} = P_ang_init;
                
                % mark state as valid
                kf.state_valid(s) = 1;
            end
        else
            disp('Update Step')
            % queue has been filled, and already used to prime last cycle
            % apply measurement update normally
            
            % linear axes
            for i= 1:3
                P_last = kf.lin_xyz_var{s-1, i};
                % calculate Kalman gain
                Kgain = Kalman_gain(lin_S2dm,P_last(1,1),P_last(2,1));
                % update KF states using measurement and Kalman gain
                last_pos = kf.lin_xyz_pos(s-1, i);
                last_vel = kf.lin_xyz_vel(s-1, i);
                xpk    = xpk_update(Kgain(1),Kgain(2),(meas.lin_xyz_pos(i) - last_pos),last_pos,last_vel);
                kf.lin_xyz_pos(s, i) = xpk(1);
                kf.lin_xyz_vel(s, i) = xpk(2);
                % update state covariance using Kalman gain                
                P_new = Ppk_update(Kgain(1),Kgain(2),P_last(1,1),P_last(1,2),P_last(2,1),P_last(2,2));
                kf.lin_xyz_var{s, i} = P_new;
            end
            
            % angular axes
            for i= 1:3
                P_last = kf.ang_hps_var{s-1, i};
                % calculate Kalman gain
                Kgain = Kalman_gain(ang_S2dm,P_last(1,1),P_last(2,1));
                % update KF states using measurement and Kalman gain
                last_pos = kf.ang_hps_pos(s-1, i);
                last_vel = kf.ang_hps_vel(s-1, i);
                % since we're working in the angular space, we need to wrap
                % the innovation before using it
                xpk    = xpk_update(Kgain(1),Kgain(2),wrap2pi(meas.ang_hps_pos(i) - last_pos),last_pos,last_vel);
                kf.ang_hps_pos(s, i) = wrap2pi(xpk(1));
                kf.ang_hps_vel(s, i) = xpk(2);
                % update state covariance using Kalman gain                
                P_new = Ppk_update(Kgain(1),Kgain(2),P_last(1,1),P_last(1,2),P_last(2,1),P_last(2,2));
                kf.ang_hps_var{s, i} = P_new;
            end
        end    
        
        % increment meas_idx to next entry
        meas_idx = meas_idx + 1;
    else    
        % no measurement was applied this loop, copy over states from the
        % previous loop for the prediction step to action on
        kf.lin_xyz_pos(s, :) = kf.lin_xyz_pos(s-1, :);
        kf.lin_xyz_vel(s, :) = kf.lin_xyz_vel(s-1, :);
        kf.ang_hps_pos(s, :) = kf.ang_hps_pos(s-1, :);
        kf.ang_hps_vel(s, :) = kf.ang_hps_vel(s-1, :);
        
        for i=1:3
            kf.lin_xyz_var{s, i} = kf.lin_xyz_var{s-1, i};
            kf.ang_hps_var{s, i} = kf.ang_hps_var{s-1, i};
        end
    end    
    
    
    % perform prediction step
    % if the last state was valid, so is this one
    if(kf.state_valid(s-1) > 0)
        kf.state_valid(s) = 1;
    end
    
    if(abs(t - ld.time_s(meas_idx)) > max_valid_time_s)
      kf.state_valid(s) = 0;
      if(sum(kf.state_valid) > 0)
        disp('Quitting early due to invalid KF states (not enough meas!)')
        break;
      end
    end  

    % only apply the prediction step if we have a valid KF state
    if(kf.state_valid(s))
        disp('Prediction Step')
        % linear axes
        for i= 1:3
            P_last = kf.lin_xyz_var{s, i};
            last_pos = kf.lin_xyz_pos(s, i);
            last_vel = kf.lin_xyz_vel(s, i);
            % propagate state forward using constant velocity assumption
            xmk = xmk_predict(dt_pred,last_pos,last_vel);
            kf.lin_xyz_pos(s, i) = xmk(1);
            kf.lin_xyz_vel(s, i) = xmk(2);
            % degrade covariance due to prediction uncertainty
            Pmk = Pmk_predict(lin_S2dp*(dt_pred),lin_S2vp*(dt_pred),dt_pred,P_last(1,1),P_last(1,2),P_last(2,1),P_last(2,2));
            kf.lin_xyz_var{s, i} = Pmk;
        end

        % angular axes
        for i= 1:3
            P_last = kf.ang_hps_var{s, i};
            last_pos = kf.ang_hps_pos(s, i);
            last_vel = kf.ang_hps_vel(s, i);
            % propagate state forward using constant velocity assumption
            xmk = xmk_predict(dt_pred,last_pos,last_vel);
            kf.ang_hps_pos(s, i) = wrap2pi(xmk(1));
            kf.ang_hps_vel(s, i) = xmk(2);
            % degrade covariance due to prediction uncertainty
            Pmk = Pmk_predict(ang_S2dp*(dt_pred),ang_S2vp*(dt_pred),dt_pred,P_last(1,1),P_last(1,2),P_last(2,1),P_last(2,2));
            kf.ang_hps_var{s, i} = Pmk;
        end
        
    end
    
    % Check variance of last few linear velocity states
    % If we are in a stable region (use the min for now!), 
    % mark where the C-version might exit early.
    % Check the last 'state_var_num' states
    state_var_num = 5;
    if(kf.state_valid(s) && s > state_var_num)
      states = kf.lin_xyz_vel(s-state_var_num:s, :);
      mv_mean = mean(states);
      for k=1:state_var_num
        vark(k) = sum(states(k, :) - mv_mean).^2;
      end
      mv_var(s) = mean(vark);
    end  
    
    % increment timer
    t = t + dt_pred;
end

if(sum(kf.state_valid) == 0)
  error('KF was never initialized as valid! Try increasing vel_var_limit')
end  

% plot outputs and meas
% only plot valid states from the first index, to the last valid 
% (so we can see the queue priming)
last_valid_idx = find(kf.state_valid == 1, 1, 'last');
mlast_valid_idx = find(interp1(t_steps, kf.state_valid, ld.time_s, 'previous', 'extrap') == 1, 1, 'last');
idx = ones(length(t_steps), 1) > 0;
midx = ones(length(ld.time_s), 1) > 0;
if(plot_to_last_valid)
    % for the KF states, just plot valid states
    idx(kf.state_valid==0)    = 0 > 1;
    %idx(last_valid_idx:end) = 0 > 1;
    midx(mlast_valid_idx:end) = 0 > 1;
end

% check state position and velocity covariance
kfposvar = zeros(steps, 3);
kfvelvar = zeros(steps, 3);
kfaposvar = zeros(steps, 3);
kfavelvar = zeros(steps, 3);
for s = 1:steps
  % linear  
  Px = kf.lin_xyz_var{s, 1};
  kfposvar(s, 1) = Px(1,1);
  kfvelvar(s, 1) = Px(2,2);
  Py = kf.lin_xyz_var{s, 2};
  kfposvar(s, 2) = Px(1,1);
  kfvelvar(s, 2) = Py(2,2);
  Pz = kf.lin_xyz_var{s, 3};
  kfposvar(s, 3) = Px(1,1);
  kfvelvar(s, 3) = Pz(2,2);
  
  % angular  
  Px = kf.ang_hps_var{s, 1};
  kfaposvar(s, 1) = Px(1,1);
  kfavelvar(s, 1) = Px(2,2);
  Py = kf.ang_hps_var{s, 2};
  kfaposvar(s, 2) = Px(1,1);
  kfavelvar(s, 2) = Py(2,2);
  Pz = kf.ang_hps_var{s, 3};
  kfaposvar(s, 3) = Px(1,1);
  kfavelvar(s, 3) = Pz(2,2);
end

figure; hold on;
grid on
plot(t_steps(idx), kfposvar(idx, 1))
plot(t_steps(idx), kfvelvar(idx, 1))
plot(t_steps(idx), kfaposvar(idx, 1))
plot(t_steps(idx), kfavelvar(idx, 1))
legend('Lin Pos Var (m^2)', 'Line Vel Var (m^2/s^2', 'Ang Pos Var (rad^2)', 'Ang Vel Var (rad^2/s^2)')
title('State KF Variances first axis')

% use either the minimum variance, or the first time it dips below 0.1
% whichever comes first
var_thres = 0.1; % disable for now
thres_mv_var_idx = find(mv_var <= var_thres & kf.state_valid > 0);
min_valid_var = min(mv_var(kf.state_valid > 0))
min_valid_var_kf = min(mv_var(kf.state_valid > 0))
min_mv_var_idx = find(mv_var == min_valid_var & kf.state_valid);
if(~isempty(thres_mv_var_idx))  
  mv_var_idx = min(thres_mv_var_idx(1), min_mv_var_idx);
else
  % nothing was found below the specified variance threshold
  mv_var_idx = min_mv_var_idx;
end

% use the linear state vel covariance to determine the optimal state
% min_velvar = min(kfvelvar(kf.state_valid > 0, :));
% min_valid_var = min_velvar(1);
% min_mv_var_idx = find(kfvelvar(:, 1) == min_valid_var & kf.state_valid)
% mv_var_idx = min_mv_var_idx(1);

disp(sprintf('Minimum VEL Variance = %0.4f (m/s)^2', mv_var(mv_var_idx)));

% lin states
figure; hold on;
plot(t_steps(idx), kf.lin_xyz_pos(idx, :))
reset_colours
plot(ld.time_s(midx), ld.pos_xyz(midx, :), '.')
reset_colours
plot(init_queue.init_t, init_queue.init_lin_pos, 'p', 'MarkerSize', 5, 'LineWidth', 3);
reset_colours
plot(t_steps(mv_var_idx), kf.lin_xyz_pos(mv_var_idx, :), 'x', 'MarkerSize', 10, 'LineWidth', 2)
xlabel('Time (s)'); ylabel('m');
legend( ...
  'KF pos X', 'KF pos Y', 'KF pos Z', ...
  'MEAS pos X', 'MEAS pos Y', 'MEAS pos Z', ...
  'Q init X', 'Q init Y', 'Q init Z', ...
  'KF pos X below var thres', 'KF pos Y below var thres', 'KF pos Z below var thres');
title('Linear XYZ POS meas and KF states');
grid on;

figure; hold on;
plot(t_steps(idx), kf.lin_xyz_vel(idx, :))
reset_colours
plot(ld.time_s(midx), [0, 0, 0; diff(ld.pos_xyz(midx, :))./repmat(diff(ld.time_s(midx)), 1, 3)], '.')
reset_colours
plot(init_queue.init_t, init_queue.init_lin_vel, 'p', 'MarkerSize', 5, 'LineWidth', 3);
reset_colours
plot(t_steps(mv_var_idx), kf.lin_xyz_vel(mv_var_idx, :), 'x', 'MarkerSize', 10, 'LineWidth', 2)
xlabel('Time (s)'); ylabel('m/s');
legend( ...
  'KF vel X', 'KF vel Y', 'KF vel Z', ...
  'MEAS vel X', 'MEAS vel Y', 'MEAS vel Z', ...
  'Q init X', 'Q init Y', 'Q init Z', ...
  'KF vel X below var thres', 'KF vel Y below var thres', 'KF vel Z below var thres');
title('Linear XYZ VEL meas and KF states');
grid on;
    
% ang states
meas_hps = Ang_hyzer_pitch_spin(ld.R00,ld.R01,ld.R02,ld.R12);

figure; hold on;
plot(t_steps(idx), rad2deg(kf.ang_hps_pos(idx, :)))
reset_colours
plot(ld.time_s(midx), rad2deg(meas_hps(midx, :)), '.')
reset_colours
plot(init_queue.init_t, rad2deg(init_queue.init_ang_pos), 'p', 'MarkerSize', 5, 'LineWidth', 3);
reset_colours
plot(t_steps(mv_var_idx), rad2deg(kf.ang_hps_pos(mv_var_idx, :)), 'x', 'MarkerSize', 10, 'LineWidth', 2)
xlabel('Time (s)'); ylabel('deg');
legend('KF pos HYZER', 'KF pos PITCH', 'KF pos SPIN', ...
  'MEAS pos HYZER', 'MEAS pos PITCH', 'MEAS pos SPIN', ...
  'Q init HYZER', 'Q init PITCH', 'Q init SPIN', ...
  'KF pos HYZER below var thres', 'KF pos PITCH below var thres', 'KF pos SPIN below var thres');
title('Angular HYZER PITCH SPIN POS meas and KF states');
grid on;

figure; hold on;
plot(t_steps(idx), rad2deg(kf.ang_hps_vel(idx, :)))
reset_colours
plot(ld.time_s(midx), [0,0,0; rad2deg(diff(meas_hps(midx, :))./repmat(diff(ld.time_s(midx)), 1, 3))], '.')
reset_colours
plot(init_queue.init_t, rad2deg(init_queue.init_ang_vel), 'p', 'MarkerSize', 5, 'LineWidth', 3);
reset_colours
plot(t_steps(mv_var_idx), rad2deg(kf.ang_hps_vel(mv_var_idx, :)), 'x', 'MarkerSize', 10, 'LineWidth', 2)
xlabel('Time (s)'); ylabel('deg/s');
legend('KF vel HYZER', 'KF vel PITCH', 'KF vel SPIN', ...
  'MEAS vel HYZER', 'MEAS vel PITCH', 'MEAS vel SPIN', ...
  'Q init HYZER', 'Q init PITCH', 'Q init SPIN', ...
  'KF vel HYZER below var thres', 'KF vel PITCH below var thres', 'KF vel SPIN below var thres');
title('Angular HYZER PITCH SPIN VEL meas and KF states');
grid on;


%plot3d
figure; hold on;
plot3(kf.lin_xyz_pos(idx , 1), kf.lin_xyz_pos(idx, 2), kf.lin_xyz_pos(idx, 3))

plot3(ld.pos_xyz(midx(:, 1), 1), ld.pos_xyz(midx(:, 1), 2), ld.pos_xyz(midx(:, 1), 3), '.')

plot3(init_queue.init_lin_pos(1), init_queue.init_lin_pos(2), init_queue.init_lin_pos(3), 'p', 'MarkerSize', 5, 'LineWidth', 3);

plot3(kf.lin_xyz_pos(mv_var_idx, 1), kf.lin_xyz_pos(mv_var_idx, 2), kf.lin_xyz_pos(mv_var_idx, 3), 'x', 'MarkerSize', 10, 'LineWidth', 2)
xlabel('X'); ylabel('Y');zlabel('Z');
legend( ...
  'KF pos XYZ', ...
  'MEAS pos XYZ', ...
  'Q init XYZ', ...
  'KF pos XYZ below var thres');
title('Linear XYZ POS 3d meas and KF states');
set(gca, 'YDir','reverse')
grid on;
axis equal
    

figure; hold on;
plot(t_steps(idx), mv_var(idx))
plot(t_steps(mv_var_idx), mv_var(mv_var_idx), 'x', 'MarkerSize', 10, 'LineWidth', 2)
title('Vel Variance mean(xyz)')
    
% run this command to simulate this throw
% pos_xyz, vel_xyz, hyzer, pitch, spin rate, wobble
disp('Your command (for some reason, positive hyzer and spin result in more distance in dvd_DfisX right now):')
disp((sprintf('new_throw (AVIAR,Eigen::Vector3d(%0.4f * 0,%0.4f * 0,%0.4f * 0 + 1.5),Eigen::Vector3d(%0.4f,%0.4f,%0.4f), fabs(%0.4f), %0.4f, fabs(%0.4f), 0);', ...
    kf.lin_xyz_pos(mv_var_idx, 1), kf.lin_xyz_pos(mv_var_idx, 2), kf.lin_xyz_pos(mv_var_idx, 3), ...
    kf.lin_xyz_vel(mv_var_idx, 1), kf.lin_xyz_vel(mv_var_idx, 2), kf.lin_xyz_vel(mv_var_idx, 3), ...
    kf.ang_hps_pos(mv_var_idx, 1), kf.ang_hps_pos(mv_var_idx, 2), kf.ang_hps_vel(mv_var_idx, 3))));
end
