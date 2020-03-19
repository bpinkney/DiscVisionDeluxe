# DiscVisionDeluxe Disc Aero and Physics

g++ main.cpp dfisx.cpp daero.cpp dgyro.cpp dpropagate.cpp -o main



Can currently simulate one disc at a time


Main input functions 


----Set the Throw


void new_throw 
  (Disc_Mold_Enum disc_mold_enum,
   
   Eigen::Vector3d thrown_disc_position,
   
   Eigen::Vector3d thrown_disc_velocity, 
   
   double thrown_disc_roll, 
   
   double thrown_disc_pitch, 
   
   double thrown_disc_radians_per_second, 
   
   double thrown_disc_wobble);






Set the beginning state of the throw to be simulated


----Simulate the throw


void simulate_throw();


will simulate the throw to completion using command line to output numbers using 10 ms time steps

*/
