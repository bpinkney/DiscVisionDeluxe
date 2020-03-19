
%sample

clear all;
close all;
clc;

%t_ms, x_m, y_m, z_m, qw, qx, qy, qz, R00, R01, R02, R10, R11, R12, R20, R21, R22

% M = csvread('/home/jamestkirk/disc_vision_deluxe/DiscVisionDeluxe/resources/test_throws/blackflyframecapture_labshots0/imgs_groundplanesample/csvlog.csv', 1, 0);

ld = dvd_DvisEst_load_csv_log('/home/jamestkirk/disc_vision_deluxe/DiscVisionDeluxe/resources/test_throws/blackflyframecapture_labshots0/imgs_drive15/csvlog.csv');

%define ground plane and base frame rotation
% translation is within the updated plane space
% xyz_gp = [-1.05625692125599 -0.399592271534964 2.96652548871949]';
% R_gp = [0.94456619435117 -8.45968637499528e-05 0.328321027855843;-0.0299073136309127 -0.995864647930154 0.0857855208897862;0.326956047627703 -0.0908493029490065 -0.94066261064919];
% 
% 
% calc_groundplane = 0;
% if(calc_groundplane)
%     q_mean = mean(Qwxyz);
%     % can you actually do this? sure, fine for now
%     q_mean = normQ(q_mean);
%     R_mean = Q2R(q_mean);
%     % for this ground plane, we know y was pointing toward the target
%     % so we'll apply a -90 degree Z rotation to get things back to the normal
%     % axes
%     Rz = [ ...
%             cosd(-90)  -sind(-90)  0 ;
%             sind(-90)   cosd(-90)  0 ;
%             0           0           1 ;
%             ];
%     
% 
%     x_mean = mean(pos_x)
%     y_mean = mean(pos_y)
%     z_mean = mean(pos_z)
% 
%     R_gp = Rz * R_mean
%     xyz_gp = R_gp * [x_mean;y_mean;z_mean]
%     
%     disp(mat2str(R_gp))
%     disp(mat2str(xyz_gp'))
% 
% %     error('stoplz')
% end
% 
% % rotate and translate by ground plane
% for i=1:length(ld.time_s)
% 
% %     pos_xyz(i, :) = R_gp * pos_xyz(i, :)' - xyz_gp;
%     % just shift for now
% %     pos_xyz(i, :) = pos_xyz(i, :) - xyz_gp;
%     
% end


% Coordinate frame of an AprilTag:
% 
% Origin: marker center
% X-axis: pointing to the right
% Y-axis: pointing down
% Z-axis: pointing into the marker (completing a right-handed coordinate frame)
% 
% The directions refer to an "upright" AprilTag as shown here: https://april.eecs.umich.edu/software/apriltag.html.
% 
% Translation and rotation of a detected AprilTag:
% 
% x_translation: translation from left to right with respect to the camera projection center as seen by the camera
% y_translation: translation from bottom to top with respect to the camera projection center as seen by the camera
% z_translation: negative z-distance between AprilTag and camera projection center, increases towards 0 when getting closer
% x_rotation: rotation around the AprilTag's left-to-right x-axis, increases when lifting its upper half
% y_rotation: rotation around the AprilTag's top-to-bottom y-axis, increases when lifting its left half
% z_rotation: rotation around the AprilTag's z-axis, increases when turning clock-wise

time_lim = -1;
if (time_lim > 0)
    [~, time_max_idx] = min(abs(ld.time_s - time_lim));
else
    time_max_idx = length(ld.time_s);
end    

figure; hold on;
plot3(ld.pos_xyz(1:time_max_idx, 1), ld.pos_xyz(1:time_max_idx, 2), ld.pos_xyz(1:time_max_idx, 3), '.-')
% draw shadow
axis equal
zlims = zlim;
shadow = plot3(ld.pos_xyz(1:time_max_idx, 1), ld.pos_xyz(1:time_max_idx, 2), ld.time_s(1:time_max_idx)./ld.time_s(1:time_max_idx) .* zlims(1), 'k.');
% alpha(shadow, 0.1)

grid on
title('XYZ pos')
xlabel('X'); ylabel('Y'); zlabel('Z');
%shift shadow to bopttom without vioating axis equal
zlim(zlims)
%zlim([min(pos_z), min(pos_z) + (zlims(2) - zlims(1))])
%zlim([min(pos_z), min(pos_z)+0.3])

figure; hold on;
plot(ld.time_s(1:time_max_idx), ld.pos_xyz(1:time_max_idx, :))
grid on
title('XYZ pos')

figure; hold on;
vel_xyz = diff(ld.pos_xyz) ./ [diff(ld.time_s), diff(ld.time_s), diff(ld.time_s)];
plot(ld.time_s(2:time_max_idx), smooth(sqrt(vel_xyz(:, 1).^2 + vel_xyz(:, 2).^2), 20))
plot(ld.time_s(2:time_max_idx), smooth(sqrt(vel_xyz(:, 1).^2 + vel_xyz(:, 2).^2 + vel_xyz(:, 3).^2), 20))
grid on
legend(['xy vel', 'xyz vel'])
title('XYZ abs vel')


% actual trig time
% We define several axes of estimation here:
% 1. Spin (yaw) of the disc.
%    We aren't that interested in the position here, but rather the rate
%    Regardless, we add a position state to our KF to this end
%    (recall that our angular measurements are positions)
%    We estimate spin in the 'disc frame', which is an axis orthogonal
%    (down) to a plane defined by the disc surface
% 2. Hyzer (roll) of the disc.
%    To align with Mike's aero stuff, this is defined wrt the world frame
%    So we measure the angle about an axis directly toward 
%    the centre of the fairway, along the 'forward' axis
%    We can get an angle-axis definition for the total rotation from the
%    unit quaternion, and use the projection (dot product) to find the 
%    'hyzer component' (I think)
% 3. Pitch (...pitch) of the disc.
%    Again, we'll measure this in the world frame, about an axis defined to
%    the player's right.
%
% Note: Since we have 2 axes defined in the world frame, and one in the
%       disc-plane frame, we are susceptible to 'gimbal lock' when
%       throwing directly down (oh well)

ang_hyzer = zeros(time_max_idx, 1);
ang_pitch = zeros(time_max_idx, 1);
ang_spin = zeros(time_max_idx, 1);

euler_angles = zeros(time_max_idx, 3);

% figure;
for i = 1:time_max_idx
    % get the rotation matrix back from the quaternion:
    R = [ ...
            ld.R00(i), ld.R01(i), ld.R02(i); ...
            ld.R10(i), ld.R11(i), ld.R12(i); ...
            ld.R20(i), ld.R21(i), ld.R22(i) ...
        ];%Q2R(Qwxyz(i, :));
    
    % also apply rotation into ground plane
%     R = R * R_gp';

    % rotate these vectors by R to get the axes of the disc
    % z and y are negetive here
    x_base = [1; 0; 0];
    y_base = [0; 1; 0];
    z_base = [0; 0; 1];
    
    x_vec = R * x_base;
    y_vec = R * y_base;
    z_vec = R * z_base;

    % so we can use sin-law to simply find the angle for hyzer and pitch
    % (for a camera angled such that X is forward)
    % from the projection of z_vec onto the X (pitch) and Y (hyzer) axes
    ang_hyzer(i) = asin(z_vec(2));
    ang_pitch(i) = asin(z_vec(1));

%     euler_angles(i, 1) = acos(-x_vec(2) /  sqrt(1-x_vec(3)^2));
%     euler_angles(i, 2) = acos(-x_vec(3));
%     euler_angles(i, 3) = acos(y_vec(3) /  sqrt(1-x_vec(3)^2));

    % we want the yaw rotation from the body -> world frame
    % without complication from the other two rotations
    % so we compute the 'euler yaw' angle on the transposed matrix
    % (a good way to test this is to tilt the camera and then record 
    % a disc spinning on a pencil. If your velocity is approx constant
    % you have right rotation!)
    ang_spin(i)  = atan2(R(1,2), R(1,1));

    % spin angle is a little harder, we'll start by defining the y axis of
    % the disc as 'zero' spin angle
    % So we need to find the rotation between the y axis of the disc, 
    % and the nominal position  

%     clf
%     grid on
%     xlim([-1, 1])
%     ylim([-1, 1])
%     zlim([-1, 1])
%     view([-35, 25])
%     hold on;
%     xlabel('X'); ylabel('Y'); zlabel('Z');
%     %plot3([0, z_base(1)], [0, z_base(2)], [0, z_base(3)], '.-')
%     plot3([0, x_vec(1)], [0, x_vec(2)], [0, x_vec(3)], 'X-')
%     plot3([0, y_vec(1)], [0, y_vec(2)], [0, y_vec(3)], 'V-')
%     plot3([0, z_vec(1)], [0, z_vec(2)], [0, z_vec(3)], 'd-')

    %pause(0.01);

end

figure; hold on;
plot(ld.time_s(1:time_max_idx), rad2deg(ang_hyzer), '.-');
plot(ld.time_s(1:time_max_idx), rad2deg(ang_pitch), '.-');
plot(ld.time_s(1:time_max_idx), rad2deg(ang_spin), '.-');
grid on
legend('Hyzer (about forward vec)', 'Pitch (positive up)', 'Spin angle (in disc-frame plane)')

figure; hold on
for i = 2:time_max_idx
    
    ang_hyzer_d(i-1) = wrap2pi(ang_hyzer(i) - ang_hyzer(i-1)) ./ (ld.time_s(i) - ld.time_s(i-1));
    ang_pitch_d(i-1) = wrap2pi(ang_pitch(i) - ang_pitch(i-1)) ./ (ld.time_s(i) - ld.time_s(i-1));
    ang_spin_d(i-1)  = wrap2pi(ang_spin(i) - ang_spin(i-1)) ./ (ld.time_s(i) - ld.time_s(i-1));

end
plot(ld.time_s(2:time_max_idx), smooth(rad2deg(ang_hyzer_d), 20), '.-');
plot(ld.time_s(2:time_max_idx), smooth(rad2deg(ang_pitch_d), 20), '.-');
plot(ld.time_s(2:time_max_idx), smooth(rad2deg(ang_spin_d), 20), '.-'); 
grid on
legend('Hyzer rate (about forward vec)', 'Pitch rate (positive up)', 'Spin angle rate(in disc-frame plane)')




















