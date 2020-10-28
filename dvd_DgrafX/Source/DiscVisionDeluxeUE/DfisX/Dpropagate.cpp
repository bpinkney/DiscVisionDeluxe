

#include "Dpropagate.hpp"
#include "DfisX.hpp"

/*


*/
namespace DfisX
{
  const double hacky_spin_drag_rate = 5.0;

  void propagate(Throw_Container *throw_container, const float dt)
  {
    /////hacked in please dont judge//////////////////////fix meeeeeeeeee
    // I judged
    d_forces.angular_velocity -= d_forces.angular_velocity / abs(d_forces.angular_velocity) * hacky_spin_drag_rate * dt;

    ////
    throw_container->disc_state_array.push_back(d_state);
    d_velocity += d_forces.aero_force / d_object.mass * dt;
    d_location += d_velocity * dt;
    d_state.disc_rotation += d_forces.angular_velocity;
    d_forces.step_count ++;
    p_state = d_state;
  }
}