
%sample

clear all;
close all;
clc;

%t_ms, x_m, y_m, z_m, qw, qx, qy, qz
M = csvread('/home/jamestkirk/disc_vision_deluxe/DiscVisionDeluxe/dvd_DvisEst/sandbox/apriltag_cpu_test/lobtest.csv', 1, 0);

time_s      = M(:, 1);
pos_x       = M(:, 2);
pos_y       = M(:, 3);
pos_z       = M(:, 4);
Qwxyz(:, 1) = M(:, 5);
Qwxyz(:, 2) = M(:, 6);
Qwxyz(:, 3) = M(:, 7);
Qwxyz(:, 4) = M(:, 8);

disp(sprintf('%d entries over %0.4f seconds', length(time_s), max(time_s) - min(time_s)));

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

figure; hold on;
plot3(pos_x, pos_y, pos_z, '.-')
% draw shadow
shadow = plot3(pos_x, pos_y, time_s./time_s .* min(pos_z), 'k.');
alpha(shadow, 0.1)
axis equal
grid on
title('XYZ pos')
xlabel('X'); xlabel('Y'); xlabel('Z');
%shift shadow to bopttom without vioating axis equal
zlims = zlim;
zlim([min(pos_z), min(pos_z) + (zlims(2) - zlims(1))])
%zlim([min(pos_z), min(pos_z)+0.3])

figure; hold on;
plot(time_s, pos_x)
plot(time_s, pos_y)
plot(time_s, pos_z)
grid on
title('XYZ pos')

figure; hold on;
plot(time_s, Qwxyz)
grid on
title('Qwxyz')

for i=1:length(time_s)
    eul(i, :) = quat2euler(Qwxyz(i, :));
end

figure; hold on;
plot(time_s, eul)
grid on
title('Eulers')





















