function fit_line_2d()
  % test it out
%   xyz = ...
%     [ 0.967898, 0.315868, 1.47779; ...
%       0.973359, 0.357077, 1.53373; ...
%       1.03334, 0.335661, 1.47884; ...
%       1.06878, 0.340881, 1.47258; ...
%       1.04762, 0.403202, 1.56905; ...
%       1.13443, 0.358005, 1.47055; ...
%       1.18085, 0.353355, 1.44601; ...
%       1.14965, 0.415989, 1.54722]

xyz = ...
[1.18085, 0.353355, 1.44601; ...
1.14965, 0.415989, 1.54722; ...
1.0037, 0.615142, 1.88647; ...
1.3965, 0.349804, 1.35731; ...
1.10748, 0.604248, 1.82814; ...
1.03281, 0.672457, 1.95276; ...
0.873247, 0.790302, 2.17954; ...
0.803138, 0.845876, 2.28298];
    
  dt = 0.00191571
  dt_series = cumsum(ones(length(xyz(:, 1)), 1)*dt) - dt;
  
  [slope_x, worked_x] = fit_line_2d_func(dt_series, xyz(:, 1));
  [slope_y, worked_y] = fit_line_2d_func(dt_series, xyz(:, 2));
  [slope_z, worked_z] = fit_line_2d_func(dt_series, xyz(:, 3));

  disp(sprintf('Slope XYZ = [%0.2f, %0.2f %0.2f]', slope_x, slope_y, slope_z))
end


% Get the linear fit from a set of x, y points
function [slope, worked] = fit_line_2d_func(x, y) 
  slope = 0;
  worked = 0;
  
  nPoints = length(x);
  if( nPoints < 2 )
    % Fail: infinitely many lines passing through this single point
    worked = 0;
  else 
    sumX=0;
    sumY=0;
    sumXY=0;
    sumX2=0;

    for i = 1:nPoints
      sumX = sumX + x(i);
      sumY = sumY + y(i);
      sumXY = sumXY + x(i) * y(i);
      sumX2 = sumX2 + x(i) * x(i);
    end

    xMean = sumX / nPoints;
    yMean = sumY / nPoints;
    denominator = sumX2 - sumX * xMean;

    % You can tune the eps (1e-7) below for your specific task
    if( abs(denominator) < 1e-7 )
      % Fail: it seems a vertical line
      worked = 0;
    else
      slope = (sumXY - sumX * yMean) / denominator;
      yInt = yMean - slope * xMean;
      worked = 1;
    end
  end
end