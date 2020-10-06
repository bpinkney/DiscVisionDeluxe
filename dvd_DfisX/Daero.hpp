#pragma once
#include "DfisX.hpp"

namespace DfisX
{

void 				step_Daero 				(Throw_Container &active_throw, float step_time);
void 				make_unit_vector 		(Eigen::Vector3d &vector_to_unitize);
Eigen::Vector3d 	get_unit_vector 		(Eigen::Vector3d vector_to_unitize);
double				angle_between_vectors	(Eigen::Vector3d a, Eigen::Vector3d b); 

extern Eigen::Vector3d disc_x_unit_vector;
extern Eigen::Vector3d disc_y_unit_vector;

extern double lift_induced_pitching_moment;

}