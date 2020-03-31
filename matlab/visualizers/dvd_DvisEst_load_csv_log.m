function ld = dvd_DvisEst_load_csv_log(filepath)

    M = csvread(filepath, 1, 0);

    ld.time_s      = M(:, 1);
    pos_x       = M(:, 2);
    pos_y       = M(:, 3);
    pos_z       = M(:, 4);
    ld.Qwxyz(:, 1) = M(:, 5);
    ld.Qwxyz(:, 2) = M(:, 6);
    ld.Qwxyz(:, 3) = M(:, 7);
    ld.Qwxyz(:, 4) = M(:, 8);
    ld.R00         = M(:, 9);
    ld.R01         = M(:, 10);
    ld.R02         = M(:, 11);
    ld.R10         = M(:, 12);
    ld.R11         = M(:, 13);
    ld.R12         = M(:, 14);
    ld.R20         = M(:, 15);
    ld.R21         = M(:, 16);
    ld.R22         = M(:, 17);

    ld.pos_xyz = [pos_x, pos_y, pos_z];
    
    mean(ld.pos_xyz)
    
    use_groundplane   = 1;
    calc_groundplane  = 0; % only enable this while looking at a ground-plane defining log
    if(use_groundplane)
      % define ground plane and base frame rotation
      % translation is within the ground-plane frame
      R_CG   = [0.995864632149249 0.0299075147135663 0.0857856339832013;-8.47361904241506e-05 0.944566143543897 -0.328321173990331;-0.0908494758048719 0.326956176014874 0.940662549329839]
      xyz_CG = [-0.0158304 0.128522400000004 -3.17]';%[-0.0158304 0.128522400000004 -3.17157020000002]'
      
      if(calc_groundplane)
        %get avg position for groundplanelog
        pos_xyz_mean = [mean(ld.pos_xyz(:, 1)), mean(ld.pos_xyz(:, 2)), mean(ld.pos_xyz(:, 3))];

        % for this ground plane, we know -ve x was pointing toward the target
        % so we'll apply a 90 degree Z rotation to get things back to the normal
        % axes
        % define the accidental disc plane (rotated 90 degrees due to Brandon's floor placement) as Gp
        % then this is the rotation from Gp to G, RGpG
        % We also note that nominally, the AprilTag lib defines rotations
        % in the camera frame as: ???? Fix this
        %  
        %  ___
        % |   | -> X
        % |___| x Z
        %   |
        %   v Y
        % but we want
        %  ___
        % |   | -> X
        % |___| . Z
        %   |
        %   v  Y
        
        % So add another 180 degree rotation about the X axis (first)  
        % the invert the Y axis
        % for a given matrix R0N = R01 * R12 * R23 * ... * R(N-1)N
        % our rotation from the camera frame to ground plane is then:
        % RCG = RCGp * RGpG 
        angle = -90; % this is just because the ground plane image you used last time was rotated!
        R_GpGz = [ ...
                cosd(angle)  -sind(angle)  0 ; ...
                sind(angle)   cosd(angle)  0 ; ...
                0           0           1 ; ...
                ];

        angle = 180;
        R_GpGx = [ ...
          1   0           0 ; ...
          0   cosd(angle)  -sind(angle) ; ...
          0   sind(angle) cosd(angle) ; ...
          ]; 
        
        angle = 0;
          R_GpGy = [ ...
          cosd(angle)  0   sind(angle) ;
          0           1   0 ;
          -sind(angle)  0   cosd(angle) ;
        ];

        
        R_GpG = R_GpGx * R_GpGy * R_GpGz;
        % get the average value of RCGp by averaging quaternions 
        % this isn't quite right, but it's a good enough approximation for 
        % similar quaternions
        Q_CGp = [0,0,0,0];
        for i=1:length(ld.R00)
          
          R = ...
          [ (ld.R00(i)), (ld.R01(i)), (ld.R02(i)) ; ...
            (ld.R10(i)), (ld.R11(i)), (ld.R12(i)) ; ...
            (ld.R20(i)), (ld.R21(i)), (ld.R22(i))];
          % convert to quaternion, add to weighted mean
          Q = R2Q(R);
          Q_CGp = Q_CGp + Q; 
          
        end
        
        Q_CGp = Q_CGp ./ length(ld.R00);
        Q_CGp = normQ(Q_CGp);
        
        R_CGp = Q2R(Q_CGp);
          
          
        % RCG = RCGp * RGpG
        R_CG = R_CGp * R_GpG;
        
        % rotate camera-frame points into the ground plane to get our base
        % offset?? No, just leave them defined in the camera frame
        %xyz_CG = R_CG * pos_xyz_mean'
        xyz_CG = pos_xyz_mean'

        disp(mat2str(R_CG))
        disp(mat2str(xyz_CG'))

        error('stoplz, just setting groundplane')
      end

      % rotate and translate wrt ground plane
      % to later calculate the 
      % for a given matrix R0N = R01 * R12 * R23 * ... * R(N-1)N
      % So we can calculate the updated rotation to our disc plane
      % from the ground plane to the disc frame
      % RGD = RGC * RCD = RCG_T * RCD
      
      for i=1:length(ld.time_s)           

           R_CD = [...
             ld.R00(i), ld.R01(i), ld.R02(i) ; ...
             ld.R10(i), ld.R11(i), ld.R12(i) ; ...
             ld.R20(i), ld.R21(i), ld.R22(i)];

           % rotate by base groundplane
           R_GD = R_CG * R_CD; %shouldn't this be transpose?

           ld.R00(i) = R_GD(1,1);
           ld.R01(i) = R_GD(1,2);
           ld.R02(i) = R_GD(1,3);
           ld.R10(i) = R_GD(2,1);
           ld.R11(i) = R_GD(2,2);
           ld.R12(i) = R_GD(2,3);
           ld.R20(i) = R_GD(3,1);
           ld.R21(i) = R_GD(3,2);
           ld.R22(i) = R_GD(3,3);
           
           % rotate xyz_CD positions into xyz_GD frame
           % subtract base xyz offset defined in CG frame
           pos_xyz_GD = R_CG * (ld.pos_xyz(i, :)' - xyz_CG);
           ld.pos_xyz(i, :) = pos_xyz_GD;
           % invert the y axis
           ld.pos_xyz(i, 2) = -ld.pos_xyz(i, 2);

      end
    end

    disp(sprintf('%d entries over %0.4f seconds', length(ld.time_s), max(ld.time_s) - min(ld.time_s)));

end

function Q = R2Q( R )
m00 = R(1,1);
m10 = R(1,2);
m20 = R(1,3);
m01 = R(2,1);
m11 = R(2,2);
m21 = R(2,3);
m02 = R(3,1);
m12 = R(3,2);
m22 = R(3,3);
  
trace1 = 1.0 + m00 - m11 - m22;
trace2 = 1.0 - m00 + m11 - m22;
trace3 = 1.0 - m00 - m11 + m22;
  
if( (trace1 > trace2) && (trace1 > trace3) )
    s = 0.5 / sqrt(trace1);
    Q(1) = (m12 - m21) * s;
    Q(2) = 0.25 / s;
    Q(3) = (m01 + m10) * s; 
    Q(4) = (m02 + m20) * s; 
elseif( (trace2 > trace1) && (trace2 > trace3) )
    s = 0.5 / sqrt(trace2);
    Q(1) = (m20 - m02) * s;
    Q(2) = (m01 + m10) * s; 
    Q(3) = 0.25 / s;
    Q(4) = (m12 + m21) * s; 
else
    s = 0.5 / sqrt(trace3);
    Q(1) = (m01 - m10) * s;
    Q(2) = (m02 + m20) * s;
    Q(3) = (m12 + m21) * s;
    Q(4) = 0.25 / s;
end

end

function R = Q2R( Q )
  q0 = Q(1);
  q1 = Q(2);
  q2 = Q(3);
  q3 = Q(4);
  
  R(1,1) = 1.0-2*(q2*q2 + q3*q3);
  R(1,2) =     2*(q1*q2 - q0*q3);
  R(1,3) =     2*(q1*q3 + q0*q2);
  
  R(2,1) =     2*(q1*q2 + q0*q3);
  R(2,2) = 1.0-2*(q1*q1 + q3*q3);
  R(2,3) =     2*(q2*q3 - q0*q1);
  
  R(3,1) =     2*(q1*q3 - q0*q2);
  R(3,2) =     2*(q2*q3 + q0*q1);
  R(3,3) = 1.0-2*(q1*q1 + q2*q2);

end