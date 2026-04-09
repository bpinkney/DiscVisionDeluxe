
% // Tq = 0.5 * rho * omega^2 * b^5 * Cm
% // where omega is the angular vel
% // and 'b' is the radius
% 
% // Inertia of a thin disc:
% // Iz =      1/2 * m * r^2
% // Ix = Iy = 1/4 * m * r^2
% 
% // torque = accel * I
% 
% // rotational Re = omega * r^2 / linear_v

clear all; close all; clc;

vel = 0:1:100;
mass = 0.175;
r = 0.25/2;
Iz = 0.5 * mass * r^2;
lin_vel = 20;%vel.*(20/70); %e.g. 20
Re = vel * r^2 ./ lin_vel;

hacky_torque = vel * 5.0 * Iz;

Cm_laminar_base = 3.87 * 0.1;
Cm_laminar = (Cm_laminar_base .* Re.^(-1/2));
Cm_turbulent_base = 0.146 * 4.0;
Cm_turbulent = (Cm_turbulent_base .* Re.^(-1/5));

proper_torque_laminar = 0.5 .* 1.225 .* vel.^2 .* r.^5 .* Cm_laminar;
proper_torque_turbulent = 0.5 .* 1.225 .* vel.^2 .* r.^5 .* Cm_turbulent;

Cm_test = 1.5 .* Re.^(-1/2);
proper_torque_test = 0.5 .* 1.225 .* vel.^2 .* r.^5 .* Cm_test;

figure; hold on; grid on
plot(vel, hacky_torque, '.-')
%plot(vel, proper_torque_laminar, 's-')
%plot(vel, proper_torque_turbulent, '^-')
plot(vel, proper_torque_test, 'o-')
title('hacky linear torque vs Tq = 0.5 * rho * w^2 * r^5 * Cm at 20m/s linear speed')
xlabel('rotational speed (rad/s)')
ylabel('Torque (Nm)')
