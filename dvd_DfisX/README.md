# DiscVisionDeluxe Disc Aero and Physics

g++ main.cpp dfisx.cpp daero.cpp dgyro.cpp dpropagate.cpp -o main



Can currently simulate one disc at a time

 - 
 
 -
 
 Command Line Args
 -
must be followed by a number!
 
 -
 
 test||hyzer||pitch||posx||posy||posz||velx||vely||velz||wobble||spinrate||discmold
 
 -
 
 
 main hyzer 0.224 pitch -0.192 discmold 0 posx 0 posy 0 posz 2 velx 11 vely 3.6 velz 1.66 spinrate -46 wobble 0

 
 -
 
  main test 1
  
 -
 

Main input functions 
-

----Set the Throw


void new_throw 
  (Disc_Mold_Enum disc_mold_enum,
   
   Eigen::Vector3d thrown_disc_position,
   
   Eigen::Vector3d thrown_disc_velocity, 
   
   double thrown_disc_roll, 
   
   double thrown_disc_pitch, 
   
   double thrown_disc_radians_per_second, 
   
   double thrown_disc_wobble);



-


Set the beginning state of the throw to be simulated

-


----Simulate the throw


void simulate_throw();


will simulate the throw to completion using command line to output numbers using 10 ms time steps

*/
