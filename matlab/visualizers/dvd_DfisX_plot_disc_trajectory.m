% args are <csv file path>, <animate>
function ld = dvd_DfisX_plot_disc_trajectory(varargin)
    
    close all; clc;

    % get path back up to test csv in case no path is provided
    script_path = (mfilename('fullpath'));
    if(isunix)
      script_path_parts = strsplit(script_path,'/');
      script_path_parts = {'/', script_path_parts{:}};
    else
      script_path_parts = strsplit(script_path,'\');      
    end    
    % prune 3 folders from end of path (where matlab script lives!)
    script_path_parts = script_path_parts(1:length(script_path_parts)-3);
    % generate filepath
    f = fullfile(script_path_parts{:}, 'dvd_DfisX', 'simulated_throw.csv')
    
    Defaults = {f, 1};
    idx = ~cellfun('isempty',varargin);
    Defaults(idx) = varargin(idx);
    
    filepath = Defaults{1}
    animate  = Defaults{2};

    %assume dt for now based on cpp code
    dt = 0.01;

    M = csvread(filepath, 1, 0);

    time_s          = M(:, 1);
    pos_xyz(:, 1)   = M(:, 2); %forward
    pos_xyz(:, 2)   = M(:, 3);
    pos_xyz(:, 3)   = M(:, 4);
    disc_state      = M(:, 5);
    orient_uv(:, 1) = M(:, 6);
    orient_uv(:, 2) = M(:, 7);
    orient_uv(:, 3) = M(:, 8);
    
    % differentiate a quick 3d vel
    vel_xyz = [[0,0,0]; diff(pos_xyz)./repmat(diff(time_s), 1, 3)];
    vel_mag = sqrt(vel_xyz(:, 1).^2 + vel_xyz(:, 2).^2 + vel_xyz(:, 3).^2);
    
    %time_s = 0:dt:length(pos_xyz(:, 1))-dt;    

    disp(sprintf('%d entries over %0.4f seconds', length(time_s), max(time_s) - min(time_s)));

    mainplot = 1;
    YZplot   = 2;
    XZplot   = 3;
    
    y_centre = (max(pos_xyz(:, 2)) - min(pos_xyz(:, 2)))/2;
    
    % plot all three views (first plot is true 3d)
    figure();
    for i=1:3
      if(i == mainplot)
        figure(mainplot); hold all;
        fig=gcf;
        fig.Units='normalized';
        fig.OuterPosition=[0 0 0.5 1];
        %subplot(2, 2, [1,3]); hold all;
      elseif(i == YZplot)
        figure(YZplot); hold all;
        fig=gcf;
        fig.Units='normalized';
        fig.OuterPosition=[0.5 0.5 0.5 0.5];
        %subplot(2, 2, 2); hold all;
      elseif(i == XZplot)
        figure(XZplot); hold all;
        fig=gcf;
        fig.Units='normalized';
        fig.OuterPosition=[0.5 0 0.5 0.5];
        %subplot(2, 2, 4); hold all;
      end
      
      plot3(pos_xyz(:, 1), pos_xyz(:, 2), pos_xyz(:, 3), '.-')    
      quiver3( ...
        min(pos_xyz(:, 1)), y_centre, 0, ...
        max(pos_xyz(:, 1)) - min(pos_xyz(:, 1)), 0, 0);
      xlabel('X (forward)'); ylabel('Y'); zlabel('Z');
      
      grid on;

      %axis equal
      if(i == mainplot)
        title('dvd_DfisX disc flight Full 3D (inspect this one!)')
        view(-90, 90)
        axis equal
      elseif(i == YZplot)
        title('dvd_DfisX disc flight YZ view')
        set(gca,'DataAspectRatio',[1 1 1])
        view(-90, 0)
      elseif(i == XZplot)
        title('dvd_DfisX disc flight XZ view')
        set(gca,'DataAspectRatio',[1 1 1])
        view(0, 0)
      end    
    end

    if(animate)
      time_stretch = 1;
      dt = [0; diff(time_s)]
      min_step_time = 0.1 % don't step smaller than this to speed things up

      idx_step = round(min_step_time / mean(dt))

      for k = 1:idx_step:length(time_s)
        tic
        for i=1:3
          if(i == mainplot)
            if(exist('maintext'))
              delete(maintext)
            end
            figure(mainplot);            
          elseif(i == YZplot)
            figure(YZplot);
          elseif(i == XZplot)
            figure(XZplot);
          end
          if(k > 1)
            % delete last disc render for yz and xz plots, but just dim
            % line for main plot
            if(i == mainplot)
              children = get(gca, 'children');
              children(1).LineWidth=0.5;
            else  
              children = get(gca, 'children');
              delete(children(1));
            end
          end
          
          %add some text
          if(i == mainplot)
            if(exist('maintext'))
              delete(maintext)
            end            
            maintext = text(0, 0, 0, 'Jim');
          end
          
          plot_disc_circle(pos_xyz(k, :), orient_uv(k, :));
        end
        plottime = toc;
        pause(max(0.0001, (dt(k) * idx_step * time_stretch) - plottime));

      end
    end
    pause
end


function plot_disc_circle(pos_xyz, orient_uv)
  %init
  % Original points, original plane
  disc_radius = 2.0; % exaggerate this a bit!
  disc_t = linspace(0,2*pi);
  disc_x = cos(disc_t) * disc_radius;
  disc_y = sin(disc_t) * disc_radius;
  disc_z = 0*disc_t;
  disc_pnts = [disc_x;disc_y;disc_z];
  % unit normal for original plane
  disc_n0 = [0;0;1]; 
  disc_n0 = disc_n0/norm(disc_n0);

  %run
  % unit normal for plane to rotate into 
  % plane is orthogonal to n1... given by equation
  % n1(1)*x + n1(2)*y + n1(3)*z = 0
  disc_n1 = orient_uv; 
  disc_n1 = disc_n1/norm(disc_n1); 
  % theta is the angle between normals
  c = dot(disc_n0,disc_n1) / ( norm(disc_n0)*norm(disc_n1) ); % cos(theta)
  s = sqrt(1-c*c);                        % sin(theta)
  u = cross(disc_n0,disc_n1) / ( norm(disc_n0)*norm(disc_n1) ); % rotation axis...
  u = u/norm(u); % ... as unit vector
  C = 1-c;
  % the rotation matrix
  R = [u(1)^2*C+c, u(1)*u(2)*C-u(3)*s, u(1)*u(3)*C+u(2)*s
      u(2)*u(1)*C+u(3)*s, u(2)^2*C+c, u(2)*u(3)*C-u(1)*s
      u(3)*u(1)*C-u(2)*s, u(3)*u(2)*C+u(1)*s, u(3)^2*C+c];
  % Rotated points
  newPnts = R*disc_pnts;

  fplot = plot3(pos_xyz(1) + newPnts(1,:),pos_xyz(2) + newPnts(2,:), pos_xyz(3) + newPnts(3,:),'k-', 'LineWidth', 3);  

end




















