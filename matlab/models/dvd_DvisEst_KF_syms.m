

% start with the algebraic syms to transform the AprilTag measurements into
% the correct angles

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

syms ang_hyzer ang_pitch ang_spin R00 R01 R02 R10 R11 R12 R20 R21 R22 real;


R = [R00 R01 R02; ...
     R10 R11 R12; ...
     R20 R21 R22];

% rotate these vectors by R to get the axes of the disc
% z and y are negetive here
x_base = [sym(1); 0; 0];
y_base = [0; sym(1); 0];
z_base = [0; 0; sym(1)];

x_vec = R * x_base;
y_vec = R * y_base;
z_vec = R * z_base;

% so we can use sin-law to simply find the angle for hyzer and pitch
% (for a camera angled such that X is forward)
% from the projection of z_vec onto the X (pitch) and Y (hyzer) axes
ang_hyzer = asin(z_vec(2));
ang_pitch = asin(z_vec(1));

% we want the yaw rotation from the body -> world frame
% without complication from the other two rotations
% so we compute the 'euler yaw' angle on the transposed matrix
% (a good way to test this is to tilt the camera and then record 
% a disc spinning on a pencil. If your velocity is approx constant
% you have right rotation!)
ang_spin  = atan2(R(1,2), R(1,1));

ang_hyzer = simplify(ang_hyzer)
ang_pitch = simplify(ang_pitch)
ang_spin  = simplify(ang_spin)



%% now let's define our Kalman Filter states for each axis

% We are going to be very lazy (and a little bit memory concious) here
% by defining all of our linear an angular states separately, and with
% separate covariance matrices. All states will take the form of:
% x = [xp, xd]
 
syms p11 p12 p21 p22 real; % Covariance
syms dt dk1 vk1 dk S2dp S2vp S2dm dmk vmk Kk1 Kk2 dpk vpk real;


%XYZ and angular state Kalman filter state space
% mk -> -k -> state at time 'k' after prediction/propagation step
% pk -> +k -> state at time 'k' after measurement update step


%% define states
xpk1 = [dk1; vk1]
yk = dk; %measurements are positions

%% define A, B, and C Jacobian matrices
%propagate the states forward
A = [...
  sym(1), dt; ...
  sym(0), sym(1)];
%no inputs in the prediction step!
B = [0; 0]
%apply the measurement 
C = [sym(1), sym(0)]  

%define state covariance
Ppk1 = [p11, p12; p21, p22];

% define process covariance and measurement covariance matrices
% process covariance is a function of our derivatives and prdiction forward in time
Q = [S2dp, sym(0); sym(0), S2vp]
% measurement covariance is a function of our measurement uncertainty 
% (this could vary depending on the AprilTag detection quality, but we need to generate it
% based on hamming error, etc.)
% It's probably sufficient to have a fixed measurement covariance for this for now.
R = [S2dm]

% output a matlab function for each of these so we can easily move them to C++
%% prediction/propagation step
% update state covariance of xmk (prediction step) due to prediction uncertainty (Q)
Pmk = A * Ppk1 * A' + Q;
  Pmk = collectby(simplify(Pmk), dt);
  Pmk_predict = matlabFunction(Pmk)
  Pmk = [p11, p12; p21, p22];
  
% propagate states forward by one timestep
xmk = A*xpk1;% + B*jk1
  xmk = collectby(simplify(xmk), dt);    
  xmk_predict = matlabFunction(xmk)  
  xmk = [dmk; vmk];
      
%% update step
xmk = [dmk; vmk];
yk = dk;
% calculate Kalman gain
Kk = Pmk * C' * inv(C * Pmk * C' + R)
  Kalman_gain = matlabFunction(simplify(Kk))  
  Kk = [Kk1;Kk2];  
% apply Kalman gain for measurement update
xpk = xmk + Kk * (yk - C * xmk);
  xpk_update = matlabFunction(simplify(xpk))
  xpk = [dpk; vpk];
%update state covariance/uncertainty during VO measurement step
I = eye(2);
Ppk = (I - Kk * C) * Pmk;  
  Ppk_update = matlabFunction(simplify(Ppk))
  


























    
    
