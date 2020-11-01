
ccr

% test finding projection vectors for incoming air and form drag

airspeed = 20.0;
airspeed_vector = [1;0;0]; 

% rotate disc plane
disc_norm_base = [0;0;1];
r = 2;
disc_circle = [sin(0:0.1:pi*2+0.1); cos(0:0.1:pi*2+0.1); (0:0.1:pi*2+0.1).*0];
disc_circle2 = [sin(0:0.1:pi*2+0.1); cos(0:0.1:pi*2+0.1); (0:0.1:pi*2+0.1).*0 - r*0.1];
disc_circle_rot = [disc_circle,disc_circle];

pitch_sweep = -deg2rad(90):0.005:deg2rad(90);
aoa_series = [];
for k = 1:length(pitch_sweep)
  
  %R = rotationY(deg2rad(20)) * rotationX(deg2rad(0));
  R = rotationY(-pitch_sweep(k)) * rotationX(deg2rad(0));

  for i=1:length(disc_circle)

    disc_circle_rot(:,i*2-1) = (R * disc_circle(:, i));
    disc_circle_rot(:,i*2)  = (R * disc_circle2(:, i));

  end

  disc_norm = R * disc_norm_base;

  % get orth vector to airspeed and disc norm
  orth_vect = cross(disc_norm, airspeed_vector);

  % now use this vector to get the projected edge force vector along the disc plane
  edge_force_vect = cross(disc_norm, orth_vect);

  %example
  a = airspeed_vector;
  b = disc_norm;
  aoa = atan2(norm(cross(a, b)), dot(a, b)) - pi/2;
  aoa_series(k) = aoa;

  r = 0.5;%0.25/2;
  Cd = 1.5;
  rho = 1.225;
  A_plate = pi*r^2;
  % add a multiplier vs aoa for A_plate if the underside of the disc is
  % showing (attributed to the bottom lip getting exposed to the airflow?)
%   if(aoa < 0)
%     
%     A_plate = A_plate * (1+abs(aoa));
%     
%   end 

  edge_height = 0.02;
  A_edge = r * 2 * edge_height; %example 2cm rim height approx

  Fd_edge  = rho * airspeed.^2 * 0.5 * Cd * A_edge  * cos(aoa);
  Fd_plate = rho * airspeed.^2 * 0.5 * Cd * A_plate * sin(aoa);
  
  disp(sprintf('%d: AOA = %0.2f deg, Fd mag [plate,edge] = [%0.5f, %0.5f]', k, rad2deg(aoa), Fd_plate, Fd_edge))

  Fd_edge_vect = edge_force_vect .* Fd_edge;
  Fd_plate_vect = disc_norm .* Fd_plate;
  
  % let's add some bernoulli effects!
  % assume that this is a combination of two factor which affect the
  % pressure differential on the top and bottom of the disc
  % 1. Airfoil effect: 
  %  the arc length of the domed surface on the top of
  %  the disc creates an airfoil. The length of surface the air travels over
  %  is longe on the top than the bottom, and so the airspeed on top is
  %  higher, and subsequently, the pressure is lower. The resulting
  %  top/bottom pressure differential causes lift.
  % 2. The bottom lip air redirect:
  %  By redirecting the flow of air downward, the bottom lip produces lift
  %  with some re-direction forces (which should be encompassed by the
  %  plate form drag later ideally!), and ALSO slows the air at the bottom
  %  down a bit. This slower air increases the local pressure, and then
  %  this pressure differential with the top of the disc produces lift.
  % So now we need to try and compute the effective 'Cl * A' of the disc as
  % a combination of these two effects.

  % Grabbing some experimental data from DPFD paper:
  
  % 1. Use a fixed Reynolds number of 3.78*10^5 for linear motion (from page 90 DPFD)
  % I assume the reference area used for all these calculations was
  % A_ref = pi * r^2 due to the incorrect treatment of 'Cd' and 'Cl' as largely
  % variable quantities
  
  % 2. Effect of cavity depth vs disc diameter (from page 74,90 DPFD):
  % thickness / diameter = [NOCAV, 0.1, 0.15] for the three samples
  % with a nominal roof thickness of 0.004m, and rim thickness of 0.01m
  % Cl0 (zero aoa) = [0, 0.054, -0.039]
  % dCl/Daoa = [0.036, 0.049, 0.045]
  % Cd0 = [0.092, 0.124, 0.148]
  if(0)
    diameter = 0.200;
    thickness = [0.1, 0.1, 0.15].*diameter;
    rim_width = 0.01;
    roof_height = 0.004;

    %% SOLVED! This effect adds about half the lip surface area to the drag model as if it were edge height
    % This relationship for Cd0 is actually pretty interesting (unlike the
    % boring camber and thickness results below......)
    % let's assume that some component of the inner lip gets added to the
    % effective edge height for drag production. At AOA=0, this would imply
    % that air is ducking into the cavity, and becoming incident on the
    % back wall.
    % While is this undoubtably the case, there may be more complex wake
    % effects at work here as well. Let's check!
    Cd0 = [0.092, 0.124, 0.148];    
    added_inner_lip_height = [0, 0.1*diameter-roof_height, 0.15*diameter-roof_height];
    
    Cd0_eff = (diameter .* thickness).*15+0.015;

    % guess here is that the whole inner lips is not exposed by some factor
    lip_exposed_surface_factor = 0.5;
    A_eff_lip = (diameter-rim_width*2) .* ...
      added_inner_lip_height.*lip_exposed_surface_factor;
    Cd0_eff_lip = ...
      ( ...
        A_eff_lip ...
      ).*15+0.015;
    figure; hold on; grid on;
    plot(added_inner_lip_height, Cd0)
    plot(added_inner_lip_height, Cd0_eff + Cd0_eff_lip, '.-')
    xlabel('added_inner_lip_height (m)')
    title('Cd0 effective increase from inner lip exposure to airflow at AOA=0')
    
    
    %% TOSOLVE
    % This extra lift at aoa=0 implies that a cavity in the bottom of the
    % disc produces more bernoulli lift. Therefore, we assume that the
    % lower lip, or cavity plays some part in SLOWING down the lower
    % airflow, and thus, increasing the lower pressure.
    % Since we know from the above check that air hits the back of the
    % inner lip at AOA=0, we know this to be correct!
    % However, there is no linear relationship for extra
    % lift, correlated to the exposed underside area which will explain
    % these coeffs...
    % perhaps there is a diminishing effect as a function of cavity depth
    % vs cavity width?
    % don't try to fit the "no cavity' case here as a part of this model
    % for now
    Cl0 = [0, 0.054, -0.039];
    
    % This relationship for  A_eff_lip is obviously a bit weird
    % and non-linear, so in the next step, let's see if we can form an
    % easier quadratic or something
    Cl0_eff = [0, 0.0005./(A_eff_lip(2:end).^(0.95)) - 0.2];
    figure; hold on; grid on;
    plot(added_inner_lip_height, Cl0)
    plot(added_inner_lip_height, Cl0_eff, '.-')
    
    %% SOLVED! This is just edgedrag again
    % We know that this will at least be partly the vertical projection of
    % the 'edge' form drag on the inner lip.
    % let's check if we can mostly blame that
    % note that the cavity sizes don't follow the relationship we would
    % expect here, but they are so close, I am willing to bet that is a
    % numerical precision issue....
   
    dCl_Daoa = [0.036, 0.049, 0.045];
    aoa_range = -pi/4:0.1:pi/4;
    figure; hold on;grid on;
    j = 1; plot(aoa_range, (aoa_range .* dCl_Daoa(j)));
    j = 2; plot(aoa_range, (aoa_range .* dCl_Daoa(j)));
    j = 3; plot(aoa_range, (aoa_range .* dCl_Daoa(j)));
    reset_colours
    % the inner edge surface size is attenuated away from aoa=0 by cos, and
    % then we get the projection onto the vertical axis (vert component of
    % the 'lip' edge force along the disc plane) by sin(aoa) as below
    for j=1:3
      lift_from_projected_lip_edge_drag = sin(aoa_range).*(cos(aoa_range) * (Cd0_eff(j)*0.5 + Cd0_eff_lip(j)));
      
      % this effect at angle is DWARFED compared to the basic Cl0 factor
      
      lift_from_lip_area_bernoulli = Cl0_eff(j)*0; %???
      
      plot(aoa_range, lift_from_projected_lip_edge_drag + lift_from_lip_area_bernoulli, '.-');
    end
    title('Cl effective change from cavity, causing increased edge form drag, but being countered by extra bernoulli lift?')
    
    
  
  end
  
  % 3. Effect of camber (dome) (from page 73,90 DPFD):
  % dome peak height as a function of edge height:
  % camber_depth/edge_depth = [0.02/0.02, 0.02/0.015, 0.02/0.01] for the three samples
  % Cl0 (zero aoa) = [0, 0.057, 0.079]
  % dCl/Daoa = [0.036, 0.034, 0.033]
  % Cd0 = [0.092, 0.066, 0.056]
  if(0)
    diameter = 0.212;
    Cl0 = [0, 0.057, 0.079];
    camber_m_edge_depth = [0, 0.005, 0.01];
    
    %% SOLVED: approximately linear with arc-length ratio of the top disc!
    % let's approximate the extra arc-length of the camber
    % as a simple rectangle to start
    % we'll assume the ratio between the nominal diameter
    % and the camber arc length will cause a linear increase in
    % the resulting 'lift'
    camber_rect_arc_length = diameter + camber_m_edge_depth.*2;
    camber_arc_to_diameter_ratio = camber_rect_arc_length./diameter;
    
    % now let's compute the theoretical lift change as a function of the
    % increased 'top arc length'
    Cl0_eff = camber_arc_to_diameter_ratio;
    figure; hold on; grid on;
    plot(camber_m_edge_depth, Cl0, '.-');
    plot(camber_m_edge_depth, Cl0_eff-1, 'o-');
    title('Camber arc length ratio vs Cl0 from DPFD paper'); 
    xlabel('camber minus edge height')
    
    %% SOLVED: all a function of EDGE height!
    % this relationship is pretty clear (I think)
    % as the deg height gets smaller, effective Cd drops
    % and also (maybe), as the arc length of the camber increases, the
    % effective "plate" area also increases, let's check
    Cd0 = [0.092, 0.066, 0.056];
    edge_height = [0.02, 0.015, 0.01]

    % we'll borrow this from just below, but add
    % on the bit of a extra we expect from the camber
    Cd0_eff = (diameter .* (edge_height)).*15+0.02;
    figure; hold on; grid on;
    plot(edge_height, Cd0)
    plot(edge_height, Cd0_eff, '.-')
    title('Drag at AOA=0 for edge height at different cambers')
    xlabel('edge height')
    
    
    %% SOLVED: This is literally just the edge drag again, so easy!
    % Now we can check the change in the "lift coeff" as a function of aoa
    % Ideally this is simply an effect of the reduced 'edge size' 
    % (see Cd0_eff above), 
    % NOPE: and perhaps the increased effective 'plate area'
    % NOPE: on the top causing more form (or skin?) drag
    dCl_Daoa = [0.036, 0.034, 0.033];
    aoa_range = -pi/4:0.1:pi/4;
    figure; hold on;grid on;
    j = 1; plot(aoa_range, (aoa_range .* dCl_Daoa(j)));
    j = 2; plot(aoa_range, (aoa_range .* dCl_Daoa(j)));
    j = 3; plot(aoa_range, (aoa_range .* dCl_Daoa(j)));
    reset_colours
    % the surafce size is attenuated away from aoa=0 by cos, and
    % then we get the projection onto the vertical axis (vert component of
    % the edge force along the disc plane) by sin(aoa)
    for j=1:3
      lift_from_projected_edge_drag = sin(aoa_range).*(cos(aoa_range) * Cd0_eff(j));
      %NOPE, not a factor!
      lift_from_extra_plate_area = 0*(camber_rect_arc_length(j)/2).^2 * pi .* sin(aoa_range);
      
      plot(aoa_range, lift_from_projected_edge_drag + lift_from_extra_plate_area, '.-');
    end
    title('Cl effective change from camber, causing reduced edge form drag, and DEFINITELY NOT increasing plate form drag surface area!')
    
    
    
  end  
  
  % 4. Effect of disc thickness (this should be mostly covered by the 'edge'
  % drag above, but we can use this data to correlate and validate the
  % rectangular surface approximation)
  % ratio of thickness (depth) vs diameter
  % thickness/ diameter = [0.01, 0.025, 0.05, 0.1]
  % Cl0 (zero aoa) = [0, 0, 0, 0]
  % dCl/Daoa = [0.039, 0.038, 0.037, 0.036]
  % Cd0 = [0.032, 0.049, 0.062, 0.092]
  if(0)
    diameter = 0.200;
    thickness = [0.01, 0.025, 0.05, 0.1].*diameter;
    Cd0 = [0.032, 0.049, 0.062, 0.092];
    %% SOLVED: all a function of EDGE height!
    % for Cd0:
    % this is just linear with a small bias in alignment with our 'edge
    % form drag' model (draggy rectangle)
    Cd0_eff = (diameter .* thickness).*15+0.035; % the last two are just scale and bias to match the model
    figure; hold on; grid on;
    plot(thickness, Cd0);plot(thickness, Cd0_eff, 'o-');
    title('Effect of Edge thickness on Cd effective vs. Simply taking a rectangular model for A_eff')

    %% SOLVED: all a function of EDGE height!
    % for the added lift, we're looking to make sure that it is a function
    % of the added vertical component along the disc plane from edge
    % interference
    dCl_Daoa = [0.039, 0.038, 0.037, 0.036];
    aoa_range = -pi/4:0.1:pi/4;
    figure; hold on;grid on;
    j = 1; plot(aoa_range, (aoa_range .* dCl_Daoa(j)));
    j = 2; plot(aoa_range, (aoa_range .* dCl_Daoa(j)));
    j = 3; plot(aoa_range, (aoa_range .* dCl_Daoa(j)));
    j = 4; plot(aoa_range, (aoa_range .* dCl_Daoa(j)));
    reset_colours
    % the surafce size is attenuated away from aoa=0 by cos, and
    % then we get the projection onto the vertical axis (vert component of
    % the edge force along the disc plane) by sin(aoa)
    j = 1; plot(aoa_range, sin(aoa_range).*(cos(aoa_range) * Cd0_eff(j)), '.-');
    j = 2; plot(aoa_range, sin(aoa_range).*(cos(aoa_range) * Cd0_eff(j)), '.-');
    j = 3; plot(aoa_range, sin(aoa_range).*(cos(aoa_range) * Cd0_eff(j)), '.-');
    j = 4; plot(aoa_range, sin(aoa_range).*(cos(aoa_range) * Cd0_eff(j)), '.-');
    title('Cl effective change over AOA, DPFD paper data vs edge form drag vert component')
  end
  
  
  Cl_base = 2.0;  
  
  % height above edge for camber dome peak
  camber_m_edge_depth = 0.01; % 1cm for now

  % treat this like the pulled out edges of a rectangle, as above
  camber_rect_arc_length = r*2 + camber_m_edge_depth*2;
  camber_arc_to_diameter_ratio = camber_rect_arc_length./(r*2);
  
  % now let's compute the theoretical lift change as a function of the
  % increased 'top arc length'  
  % define the stall range for this effect
  aoa_arc_range = deg2rad([0, -45]);
  if(aoa < aoa_arc_range(1) & aoa > aoa_arc_range(2))
    % does this change with angle?
    ClA_toparc = camber_arc_to_diameter_ratio * A_plate * Cl_base * sin(aoa);
  else
    ClA_toparc = 0;
  end
  Fl_arc  = rho * airspeed.^2 * 0.5 * ClA_toparc;
  
  % scale effect for exposed edge of inner back rim
  roof_height = 0.004;
  rim_width = 0.02;
  added_inner_lip_height = 0.012;
  %edge_height defined above
  
  % guess here is that the whole inner lips is not exposed by some factor
  % (this is validated above) 
  lip_exposed_surface_factor = 0.5; %this should probably be around 0.5
  A_eff_lip = (r*2-rim_width*2) .* ...
      added_inner_lip_height.*lip_exposed_surface_factor;    
  
  % define the range for added edge drag for the inner lip
  %lip_scale_range = deg2rad([-90, 90]);
  %if(aoa > lip_scale_range(1) & aoa < lip_scale_range(2))
    Fd_lip(k) = rho * airspeed.^2 * 0.5 * Cd * A_eff_lip * cos(aoa);
  %else
  %  Fd_lip(k) = 0;
  %end
  
  % define the range for bernoulli from the inner lip
  scaling_factor = 3.0; % arbitrary model tuner for now
  %lip_scale_range = deg2rad([-90, 90]);
  %if(aoa > lip_scale_range(1) & aoa < lip_scale_range(2))
    lift_factor = -0.0005./(A_eff_lip.^(0.95)) * scaling_factor;
    Fl_lip(k) = rho * airspeed.^2 * 0.5 * lift_factor;
 % else
  %  Fl_lip(k) = 0;
  %end
  %Fl_lip  = rho * airspeed.^2 * 0.5 * ClA_botlip;  
  
  Fd_lip_vect = edge_force_vect .* Fd_lip(k);
  Fl_lip_vect = disc_norm .* Fl_lip(k);
  
  Fl_arc_vect = disc_norm .* Fl_arc;  
  
  F_drag_sum = Fd_edge_vect + Fd_plate_vect;
  F_lift_sum = Fl_arc_vect + Fl_lip_vect + Fd_lip_vect;
  
  F_sum = F_drag_sum + F_lift_sum;

%   % weird extra thing?
%   if(aoa < lip_scale_range(2))
%     F_sum(1) = F_sum(1) - 0.5*(rho * airspeed.^2 * 0.5);
%     F_sum(3) = F_sum(3) - 0.5*(rho * airspeed.^2 * 0.5);
%   end

  % assume only in X for now
  Cd_eff_base(k) = F_drag_sum(1) / (rho * airspeed.^2 * 0.5);
  Cl_eff_base(k) = F_drag_sum(3) / (rho * airspeed.^2 * 0.5);
  Cd_eff_lip(k) = (F_drag_sum(1)+Fl_lip_vect(1)+Fd_lip_vect(1)) / (rho * airspeed.^2 * 0.5);
  Cl_eff_lip(k) = (F_drag_sum(3)+Fl_lip_vect(3)+Fd_lip_vect(3)) / (rho * airspeed.^2 * 0.5);
  Cd_eff_arc(k) = (F_drag_sum(1)+Fl_arc_vect(1)) / (rho * airspeed.^2 * 0.5);
  Cl_eff_arc(k) = (F_drag_sum(3)+Fl_arc_vect(3)) / (rho * airspeed.^2 * 0.5);
  Cd_eff(k) = F_sum(1) / (rho * airspeed.^2 * 0.5);
  Cl_eff(k) = F_sum(3) / (rho * airspeed.^2 * 0.5);

  % comparison with papers which WRONGLY change the Cd and Cl instead of 
  % changing the effective area......
  Cd_paper(k) = 0.08 + 2.72*aoa^2;
  Cl_paper(k) = 0.15 + 1.4 * aoa;

plot_me = 0;
if(plot_me)
  if(k == 1) 
    figure; hold on; grid on; axis equal
    set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
  else
    delete(dc);
    delete(q1);
    delete(q2);
    delete(q3);
    delete(q4);
    delete(q5);
    delete(q6);
    delete(q7);
    delete(t1);
    delete(t2);
    delete(t3);
    delete(t4);
  end
  reset_colours
  dc = plot3(disc_circle_rot(1,:),disc_circle_rot(2,:),disc_circle_rot(3,:), 'k', 'LineWidth', 1);
  q1 = quiver3(0,0,0,disc_norm(1),disc_norm(2),disc_norm(3), 'LineWidth', 3);
  q2 = quiver3(airspeed_vector(1)*2,airspeed_vector(2)*2,airspeed_vector(3)*2, -airspeed_vector(1),-airspeed_vector(2),-airspeed_vector(3),2, 'LineWidth', 3, 'MaxHeadSize', 50);
  t1 = text(airspeed_vector(1),airspeed_vector(2),airspeed_vector(3)+0.5, sprintf('V∞ = %0.1f m/s\nAOA = %0.1f deg', airspeed, rad2deg(aoa)),'Color','red','FontSize',14);
  q3 = quiver3(0,0,0,orth_vect(1),orth_vect(2),orth_vect(3), '--', 'LineWidth', 1, 'ShowArrowHead', 'off');
  q4 = quiver3(0,0,0,edge_force_vect(1),edge_force_vect(2),edge_force_vect(3), '--', 'LineWidth', 1, 'ShowArrowHead', 'off');
  q5 = quiver3(0,0,0,Fd_edge_vect(1),Fd_edge_vect(2),Fd_edge_vect(3), '-', 'LineWidth', 3, 'MaxHeadSize', 50);
  t2 = text(Fd_edge_vect(1),Fd_edge_vect(2),Fd_edge_vect(3), sprintf('Edge Force (%0.2f N)', Fd_edge),'Color','red','FontSize',14, 'HorizontalAlignment', 'right');
  q6 = quiver3(0,0,0,Fd_plate_vect(1),Fd_plate_vect(2),Fd_plate_vect(3), '-', 'LineWidth', 3);
  t3 = text(Fd_plate_vect(1),Fd_plate_vect(2),Fd_plate_vect(3), sprintf('Plate Force (%0.2f N)', Fd_plate),'Color','red','FontSize',14, 'HorizontalAlignment', 'left');
  q7 = quiver3(0,0,0,F_sum(1),F_sum(2),F_sum(3), '-', 'LineWidth', 3);
  t4 = text(F_sum(1),F_sum(2),F_sum(3), sprintf('Net Force Mag (%0.2f N)', sqrt(F_sum(1)^2 + F_sum(2)^2 + F_sum(3)^2)),'Color','green','FontSize',16, 'HorizontalAlignment', 'right');

  if(k == 1) 
  click_legend({'disc circle', 'disc norm', 'airspeed vector', 'edge force line', 'edge force', 'plate force'});
  view([-17,26]);
  end
  
  pause(0.05);
end
  
end

figure; 
subplot(2,1,2);hold on; grid on
plot(rad2deg(-aoa_series), -Cd_eff_base)
plot(rad2deg(-aoa_series), -Cd_eff_arc, '-')
plot(rad2deg(-aoa_series), -Cd_eff_lip, '--')
plot(rad2deg(-aoa_series), -Cd_eff, '.')
%plot(rad2deg(pitch_sweep), Cd_paper)
title('Cd effective vs Cd from paper')
click_legend({'Form Drag Cd effective nolift', 'Form Drag Cd effective plus arc effect', ...
  'Form Drag Cd effective plus lip effect', ...
  'Form Drag Cd effective plus arc and lip effect', 'Cd from paper'})
xlabel('AOA (deg)')
ylabel('Cd effective')
ylim([0, 1.6])
subplot(2,1,1);hold on; grid on
plot(rad2deg(-aoa_series), -Cl_eff_base)
plot(rad2deg(-aoa_series), -Cl_eff_arc, '-')
plot(rad2deg(-aoa_series), -Cl_eff_lip, '--')
plot(rad2deg(-aoa_series), -Cl_eff, '.')
%plot(rad2deg(aoa_series), -Cl_paper)
title('Cl effective vs Cl from paper')
click_legend({'Form Drag Cl effective nolift', 'Form Drag Cl effective plus arc effect', ...
  'Form Drag Cl effective plus lip effect', ...
  'Form Drag Cl effective plus arc and lip effect', 'Cl from paper'})
xlabel('AOA (deg)')
ylabel('Cl effective')
ylim([-1, 2.5])


if(0)
  %% this model is not right!
  air_vel = 20.0;
  
  % all the way from freefall to free rise
  aoa = -pi/2:0.1:pi/2;

  r = 0.25/2;
  A_plate = pi*r^2;
  % let's say edge is 1.5cm tall
  A_edge = 0.015 * r * 2;

  % old stall model
  pav2by2 = 1.225 * A_plate * air_vel^2 * 0.5;
  drag_coefficient = 0.05000;
  old_stall_drag = [];
  old_form_drag = [];
  old_drag = [];

  % new form drag model
  Cd_plate = 1.17;
  Cd_edge = 1.1;
  pav2by2_new = 1.225 * air_vel^2 * 0.5;
  new_form_drag_edge = [];
  new_form_drag_plate = [];
  new_form_drag = [];

  for i=1:length(aoa)
    %% old stall model

    stall_induced_drag = cos(2*aoa(i))+0.55;
    if(aoa(i) > -0.52 & aoa(i) < 0.52)
      old_stall_drag(i) = stall_induced_drag * pav2by2 * -sign(air_vel);
    else
      old_stall_drag(i) = 0;
    end
    old_form_drag(i) = pav2by2 * drag_coefficient * -sign(air_vel);

    old_drag(i) = old_stall_drag(i) + old_form_drag(i);  

    %% new form drag model
    new_form_drag_edge(i) = pav2by2_new * A_edge * Cd_edge * cos(aoa(i)) * -sign(air_vel);
    new_form_drag_plate(i) = pav2by2_new * A_plate * Cd_plate * sin(aoa(i)) * -sign(air_vel);
    new_form_drag(i) = new_form_drag_edge(i) + new_form_drag_plate(i);

  end


  figure; hold on; grid on;
  plot(rad2deg(aoa), old_stall_drag)
  plot(rad2deg(aoa), old_form_drag)
  plot(rad2deg(aoa), old_drag, 'k', 'LineWidth', 2)
  reset_colours
  plot(rad2deg(aoa), new_form_drag_plate, '.-')
  plot(rad2deg(aoa), new_form_drag_edge, '.-')
  plot(rad2deg(aoa), new_form_drag, 'k.-', 'LineWidth', 2)
  legend( ...
    'old stall drag', 'old form drag (edge)', 'old drag sum', ...
    'new stall drag (plate)', 'new form drag (edge)', 'new drag sum');
  xlabel('AOA (deg)')
  ylabel('Force Magnitude along airspeed vector (Newtons)')
  title('Old stall+edgeform drag model vs new plate+edge drag model')

end




