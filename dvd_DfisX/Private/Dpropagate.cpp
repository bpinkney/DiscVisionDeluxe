

#include "Dpropagate.hpp"
#include "DfisX.hpp"

// unreal junk
#include "ModuleManager.h"
IMPLEMENT_MODULE(FDefaultModuleImpl, dvd_DfisX);
// unreal junk

namespace DfisX
{
const double hacky_spin_drag_rate = 5.0;

void propagate (Throw_Container &active_throw, float step_time)
{


	/////hacked in please dont judge//////////////////////fix meeeeeeeeee
	d_forces.angular_velocity -= d_forces.angular_velocity / abs (d_forces.angular_velocity) * hacky_spin_drag_rate * step_time;
	////
	active_throw.disc_state_array.push_back (d_state);
	d_velocity += d_forces.aero_force / d_object.mass * step_time;
	d_location += d_velocity * step_time;
	d_forces.step_count ++;
	p_state = d_state;

}

}