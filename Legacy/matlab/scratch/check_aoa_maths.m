
ccr

airspeed_vector = [20, 0, 0];
airspeed_vector_unit = airspeed_vector./norm(airspeed_vector);

disc_normal = [sind(45),0,sind(45)];

aoa = atan2(norm(cross(airspeed_vector_unit, disc_normal)), dot(airspeed_vector_unit, disc_normal)) - pi/2;

aoa_deg = rad2deg(aoa)

edge_force_vector = cross(disc_normal, cross(disc_normal, airspeed_vector_unit))