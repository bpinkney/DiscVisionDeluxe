function dvd_DvisEst_KF_on_log()
%% define KF and transform funcs

clc;
close all;
clear all;

%meas preprocessing
Ang_hyzer_pitch_spin = @(R00,R01,R02,R12)[asin(R12),asin(R02),atan2(R01,R00)];

%meas update
Kalman_gain = @(S2dm,p11,p21)[p11./(S2dm+p11);p21./(S2dm+p11)];
xpk_update = @(Kk1,Kk2,dk,dmk,vmk)[dmk+Kk1.*(dk-dmk);vmk+Kk2.*(dk-dmk)];
Ppk_update = @(Kk1,Kk2,p11,p12,p21,p22)reshape([-p11.*(Kk1-1.0),p21-Kk2.*p11,-p12.*(Kk1-1.0),p22-Kk2.*p12],[2,2]);

%predict
xmk_predict = @(dt,dk1,vk1)[dk1+dt.*vk1;vk1];
Pmk_predict = @(S2dp,S2vp,dt,p11,p12,p21,p22)reshape([S2dp+p11+dt.*(p12+p21)+dt.^2.*p22,p21+dt.*p22,p12+dt.*p22,S2vp+p22],[2,2]);


%% define parameters
% run at 522Hz for now (match camera framerate)
dt_pred = 1.0/522.0;

% fixed meas covar for now
lin_S2dm = 0.001;
ang_S2dm = 0.001;

% process covariance
lin_S2dp = 100.0; %should be a function of dt
ang_S2dp = 100.0; %should be a function of dt
lin_S2vp = 100000.0; %should be a function of dt
ang_S2vp = 100000.0; %should be a function of dt

% initial covariances values
LIN_POS_VAR_INIT = 0.1;  % (m)^2
LIN_VEL_VAR_INIT = 0.01; % (m/s)^2
ANG_POS_VAR_INIT = 0.1;  % (rad)^2
ANG_VEL_VAR_INIT = 0.01; % (rad/s)^2
%queue size to start estimate
init_queue_size = 3;    

%% load log and determine measurement count
ld = dvd_DvisEst_load_csv_log('/home/jamestkirk/disc_vision_deluxe/DiscVisionDeluxe/resources/test_throws/blackflyframecapture_labshots0/imgs_drive15/csvlog.csv');
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

init_queue.lin_xyz_pos  = zeros(init_queue_size, 3);
init_queue.ang_hps_pos  = zeros(init_queue_size, 3);
init_queue.queue_time_s = zeros(init_queue_size, 1);
init_queue.queue_count  = 0;
init_queue.primed       = 0; % mark whether or not the queue has been consumed to generate the initial estimate

% shift measurement timestamps to zero
ld.time_s = ld.time_s - ld.time_s(1);
t_steps = 0:dt_pred:(dt_pred*steps-dt_pred);
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

        if(init_queue.queue_count < init_queue_size)
            % we have just started getting measurements
            % (in c-code, we need to throw out old entries in case of false early detections!)
            % fill the queue, and do not apply a measurement update to the
            % actual states yet

            % add new meas to the queue, 
            % shift elements of the queue over for index fun
            % technically the oldest element is discarded here
            % which would be useful if we were running this awaiting a
            % member-variance check or something
            for i = 2:init_queue_size
                init_queue.lin_xyz_pos(i, :) = init_queue.lin_xyz_pos(i-1, :);
                init_queue.ang_hps_pos(i, :) = init_queue.ang_hps_pos(i-1, :);
                init_queue.queue_time_s(i)   = init_queue.queue_time_s(i-1);
            end

            init_queue.lin_xyz_pos(1, :) = meas.lin_xyz_pos;
            init_queue.ang_hps_pos(1, :) = meas.ang_hps_pos;
            init_queue.queue_time_s(1)   = meas.time_s;
            
            % increase queue count
            init_queue.queue_count = init_queue.queue_count + 1;
                
        end

        % check for full, unprimed queue
        if(~init_queue.primed)
            if(init_queue.queue_count >= init_queue_size)
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
                kf.lin_xyz_pos(s, :) = mean(init_queue.lin_xyz_pos);
                init_queue.init_lin_pos = kf.lin_xyz_pos(s, :);
                % differentiate elements in the queue to get the initial
                % velocity
                lin_vel_xyz = diff(init_queue.lin_xyz_pos) ./ repmat(diff(init_queue.queue_time_s), 1, 3);
                kf.lin_vel_xyz(s, :) = mean(lin_vel_xyz);
                init_queue.init_lin_vel = kf.lin_vel_xyz(s, :);

                kf.ang_hps_pos(s, :) = wrap2pi(mean(init_queue.lin_xyz_pos));
                % differentiate elements in the queue to get the initial
                % velocity
                ang_hps_vel = wrap2pi(diff(init_queue.ang_hps_pos) ./ repmat(diff(init_queue.queue_time_s), 1, 3));
                kf.ang_hps_vel(s, :) = mean(ang_hps_vel);

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
                xpk    = xpk_update(Kgain(1),Kgain(2),meas.lin_xyz_pos(i),last_pos,last_vel);
                kf.lin_xyz_pos(s, i) = xpk(1);
                kf.lin_xyz_vel(s, i) = xpk(1);
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
                xpk    = xpk_update(Kgain(1),Kgain(2),meas.ang_hps_pos(i),last_pos,last_vel);
                kf.ang_hps_pos(s, i) = xpk(1);
                kf.ang_hps_vel(s, i) = xpk(1);
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
            kf.lin_xyz_vel(s, i) = xmk(1);
            % degrade covariance due to prediction uncertainty
            Pmk = Pmk_predict(lin_S2dp*(dt_pred*dt_pred),lin_S2vp*(dt_pred*dt_pred),dt_pred,P_last(1,1),P_last(1,2),P_last(2,1),P_last(2,2));
            kf.lin_xyz_var{s, i} = Pmk;
        end

        % angular axes
        for i= 1:3
            P_last = kf.ang_hps_var{s, i};
            last_pos = kf.ang_hps_pos(s, i);
            last_vel = kf.ang_hps_vel(s, i);
            % propagate state forward using constant velocity assumption
            xmk = xmk_predict(dt_pred,last_pos,last_vel);
            kf.ang_hps_pos(s, i) = xmk(1);
            kf.ang_hps_vel(s, i) = xmk(1);
            % degrade covariance due to prediction uncertainty
            Pmk = Pmk_predict(ang_S2dp*(dt_pred*dt_pred),ang_S2vp*(dt_pred*dt_pred),dt_pred,P_last(1,1),P_last(1,2),P_last(2,1),P_last(2,2));
            kf.ang_hps_var{s, i} = Pmk;
        end
        
    end 
    
    % increment timer
    t = t + dt_pred;
end

% plot outputs and meas


% lin states
figure; hold on;
plot(t_steps, kf.lin_xyz_pos)
reset_colours
plot(ld.time_s, ld.pos_xyz, '.')
reset_colours
plot(init_queue.init_t+dt_pred, init_queue.init_lin_pos, 'p', 'MarkerSize', 5, 'LineWidth', 3);
legend('KF pos X', 'KF pos Y', 'KF pos Z', 'MEAS pos X', 'MEAS pos Y', 'MEAS pos Z', 'Q init X', 'Q init Y', 'Q init Z');
title('Linear XYZ POS meas and KF states');

figure; hold on;
plot(t_steps, kf.lin_xyz_vel)
reset_colours
plot(ld.time_s(2:end), diff(ld.pos_xyz)./repmat(diff(ld.time_s), 1, 3), '.')
reset_colours
plot(init_queue.init_t+dt_pred, init_queue.init_lin_vel, 'p', 'MarkerSize', 5, 'LineWidth', 3);
legend('KF vel X', 'KF vel Y', 'KF vel Z', 'MEAS vel X', 'MEAS vel Y', 'MEAS vel Z', 'Q init X', 'Q init Y', 'Q init Z');
title('Linear XYZ VEL meas and KF states');
    

    
    
    %Ang_hyzer_pitch_spin(ld.R00(meas_idx),ld.R01(meas_idx),ld.R02(meas_idx),ld.R12(meas_idx))
    
    
    
    
    
    
end
