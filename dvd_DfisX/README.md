# DiscVisionDeluxe Disc Aero and Physics

g++ main.cpp dfisx.cpp daero.cpp dgyro.cpp dpropagate.cpp -o main



Can currently simulate one disc at a time


Main input functions 


----Set the Throw


void new_throw (Disc_Object thrown_disc_object, Disc_State thrown_disc_state, double thrown_radians_per_second, double thrown_disc_wobble);





Set the beginning state of the throw to be simulated


----Simulate the throw


void simulate_throw();


will simulate the throw to completion using command line to output numbers using 10 ms time steps

*/
