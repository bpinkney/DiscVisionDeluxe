
close all; clear all; clc;

aoa = deg2rad(-20:0.1:160);

% dome triangle approx
dome_height = 0.05;
radius = 0.1;

% normal angle for triangular surfaces
normedge = atan2(radius, dome_height);

% exposued portions of front and back area
front = sin(aoa + (pi/2 - normedge)); front(front <=0) = 0;
back  = sin(aoa + ( normedge- pi/2)); back(back<=0) = 0;
% simple flat top approx
flattop = sin(aoa); flattop(flattop < 0) = 0;

% assume area is evenly split between front and back
% for now, simply use flat top area
area = pi * radius^2;
%area_dome = area;
% OR assume a hypot area:
area_dome = pi * (radius.^2 + dome_height.^2);

% assume same rho, v^2, and Cd for each and omit them

% component along disc plane
Fd_front_x = front * area_dome/2 * cos(normedge);
Fd_back_x = back * area_dome/2 * cos(normedge);

% component along disc normal
Fd_front_z = front * area_dome/2 * sin(normedge);
Fd_back_z = back * area_dome/2 * sin(normedge);
Fd_flat_z = flattop * area;

figure(1); hold on;
plot(rad2deg(aoa), Fd_front_x + Fd_back_x)
plot(rad2deg(aoa), Fd_front_z + Fd_back_z)
plot(rad2deg(aoa), Fd_flat_z);

title('simplified form drag forces for a dome top triangular approx')
legend('disc plane force front+back triangular surfaces', ...
  'disc norm force front+back triangular surfaces', ...
  'disc norm force flat top', 'Location', 'northwest')
grid on

% use the relative force magnitudes between the front and back to find the 
% centre of force location between [-radius/2, radius/2]
force_centre_position_x = radius/2 * ((front - back)./(sqrt(front.^2 + back.^2)));

% compute the resulting torque along the disc normal from this position
% note how as the force along the disc normal increases, the moment arm
% gets smaller
Fd_front_z + Fd_back_z



