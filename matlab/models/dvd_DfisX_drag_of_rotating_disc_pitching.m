
clc; close all; clear all;
% we need to compute the drag force at each cross section of the circle
% Fd = Cd * rho * 1/2 * A * v^2

% we can get the tangential linear velocity at a radius 'r', for an angular rate 'w' as:
% v = w * r

% then, for a given 'dx' from the centre of the disc circle,
% we can approximate a rectangluar cross section of part of our
% circle as 'L * dx', where 'L' will get narrower as 'x' increases

% for our purposes, we can compute 'L' and 'dx' as a fucntion of an angular
% offset from the 'y' axis which dictates the new 'x' position, and
% resulting 'L' length
% let 'l2' = L/2
% using basic trig:
% sin(theta) = op/hyp = x/r
%   x  = sin(theta) * r
% cos(theta) = adj/hyp = l2/r
%   l2 = cos(theta) * r

% now dx = x[k] - x[k-1]

% let's define some values, and take an integration sum
% as dtheta -> 0, our integration will become more correct

pkg load symbolic

syms v w r l2 L theta xk xkp1 dx Cd rho real;

dtheta = (pi/2)/10; %rad

theta = 0:dtheta:pi/2;

% plot circle segments for visual display purposes
figure; hold all;
r_val = 0.25/2;
cos_array = cos(0:0.1:2*pi+0.1);
sin_array = sin(0:0.1:2*pi+0.1);
plot(r_val*cos_array, r_val*sin_array)
axis([-r_val, r_val, -r_val, r_val])
grid on;

%sleep(5)

Fd_sum = 0;
Td_sum = 0;
x_last = 0 * r;
for i=1:length(theta)

  %x(i) = x_last;

  % get lengths and areas
  x_new = sin(theta(i)) * r;
  l2 = cos(theta(i)) * r;
  dx = x_new - x_last;
  
  % add that slice plot
  x_last_num  = double(subs(x_last, r, r_val));
  x_new_num   = double(subs(x_new, r, r_val));
  l2_num      = double(subs(l2, r, r_val));  
  rectangle('Position',[x_last_num -l2_num (x_new_num - x_last_num) l2_num*2]);
  
  x_last = x_new;
  A = l2 * 2 * dx;
  
  % get linear tangential velocity
  v = w * x_new;
  
  % get drag force
  Fd = Cd * rho * 1/2 * A * v^2;  
  Fd_sum = Fd_sum + Fd;
  
  %get resulting torque
  Td = Fd * r;
  Td_sum = Td_sum + Td;
  
  %Fd_new(i) = Fd;

end

Fd_sum = simplify(Fd_sum)
Fd_num = double(Fd_sum / Cd / (r^4) / rho / (w^2))

Td_sum = simplify(Td_sum)
Td_num = double(Td_sum / Cd / (r^5) / rho / (w^2))




















