function dvd_DvisEst_KF_on_log()
%% define KF and transform funcs

xmk_predict = @(dk1,dt,vk1)[dk1+dt.*vk1;vk1];
Pmk_predict = @(S2dp,S2vp,dt,p11,p12,p21,p22)reshape([S2dp+p11+dt.*(p12+p21)+dt.^2.*p22,p21+dt.*p22,p12+dt.*p22,S2vp+p22],[2,2]);
Kalman_gain = @(S2dm,p11,p21)[p11./(S2dm+p11);p21./(S2dm+p11)];
xpk_update = @(Kk1,Kk2,dk,dmk,vmk)[dmk+Kk1.*(dk-dmk);vmk+Kk2.*(dk-dmk)];
Ppk_update = @(Kk1,Kk2,p11,p12,p21,p22)reshape([-p11.*(Kk1-1.0),p21-Kk2.*p11,-p12.*(Kk1-1.0),p22-Kk2.*p12],[2,2]);
Ang_hyzer_pitch_spin = @(R00,R01,R02,R12)[asin(R12),asin(R02),atan2(R01,R00)];


%% define parameters
% run at 522Hz for now (match camera framerate)
dt_pred = 1.0/522.0;
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
steps = ceil(time_end / dt_pred);

%% define states and covariance matrices
kf.state_valid = zeros(steps, 1);
% linear XYZ
kf.lin_xyz_pos = zeros(steps, 3);
kf.lin_vel_xyz = zeros(steps, 3);
kf.lin_var_xyz = cell(steps, 3); % 2x2 covariance matrix for each

% angular HYZER, PITCH, SPIN
kf.ang_hps_pos = zeros(steps, 3);
kf.ang_hps_vel = zeros(steps, 3);
kf.ang_var_hps = cell(steps, 3); % 2x2 covariance matrix for each

t = 0;
meas_idx = 1;

init_queue.lin_xyz_pos  = zeros(init_queue_size, 3);
init_queue.ang_hps_pos  = zeros(init_queue_size, 3);
init_queue.queue_time_s = zeros(init_queue_size, 1);
init_queue.queue_count  = 0;
init_queue.primed       = 0; % mark whether or not the queue has been consumed to generate the initial estimate
for s = 1:steps   
   
    % perform update step if required
    if (t > ld.time_s(meas_idx) && ld.unprocessed_measurement(meas_idx) == 1)
        % mark measurement as processed
        ld.unprocessed_measurement(meas_idx) = 0;

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

            init_queue.lin_xyz_pos(1, :) = % new meas
            init_queue.ang_hps_pos(1, :) = % new meas
            init_queue.queue_time_s(1)   = % new meas
            
            % increase queue count
            init_queue.queue_count = init_queue.queue_count + 1;
                
        end

        % check for full, unprimed queue
        if(init_queue.queue_count >= init_queue_size && ~init_queue.primed)
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

            kf.lin_xyz_pos(s, :) = mean(init_queue.lin_xyz_pos);
            % differentiate elements in the queue to get the initial
            % velocity
            lin_vel_xyz = diff(init_queue.lin_xyz_pos) ./ repmat(diff(init_queue.queue_time_s), 1, 3);
            kf.lin_vel_xyz(s, :) = mean(lin_vel_xyz);

            kf.ang_hps_pos(s, :) = wrap2pi(mean(init_queue.lin_xyz_pos));
            % differentiate elements in the queue to get the initial
            % velocity
            ang_hps_vel = wrap2pi(diff(init_queue.ang_hps_pos) ./ repmat(diff(init_queue.queue_time_s), 1, 3));
            kf.ang_hps_vel(s, :) = mean(ang_hps_vel);

            % mark state as valid
            kf.state_valid(s) = 1;                
        else
            % queue has been filled, and already used to prime last cycle
            % apply measurement update normally

        end 
        
        % increment meas_idx to next entry
        meas_idx = meas_idx + 1;
    end    
    
    
    % perform prediction step
    
    % increment timer
    t = t + dt_pred;
end    
    

    
    
    
    
    
    
    
    
    
end
