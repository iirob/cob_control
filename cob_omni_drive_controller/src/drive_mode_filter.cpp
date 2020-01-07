#include "drive_mode_filter.h"

#include<stdlib.h>

void DriveModeFilter::setMode(Mode new_mode) {
	this->mode = new_mode;
}

void DriveModeFilter::setCenter(double x, double y) {
	// Security measure.
	// Problem is, if you were to set the center of rotation far away, the
	// slightest rotation would produce huge movements.
	// I should probably give a ROS_INFO some time, instead of just clipping
	// the input.
	const double max = 2.;
	if (x >  max) x =  max;
	if (x < -max) x = -max;
	if (y >  max) y =  max;
	if (y < -max) y = -max;

	this->virtual_center_x = x;
	this->virtual_center_y = y;    
}

void DriveModeFilter::set_min_turning_radius(double r_min) {
}

void DriveModeFilter::filter (double v_x_in, double v_y_in, double r_z_in, 
			 double& v_x_out, double& v_y_out, double& r_z_out) {
	if (this->mode == OMNIDIRECTIONAL) {
		v_x_out = v_x_in;
		v_y_out = v_y_in;
		r_z_out = r_z_in;
	}
	else if ((this->mode == ACKERMANN or this->mode == DIFFERENTIAL) and r_z_in == 0.0) {
		// Not much to do if there is no rotation component.
		// Getting rid of the lateral component is enough.
		// Let's handle this case now so that I don't have to worry about
		// divide by zero later.
		v_x_out = v_x_in;
		v_y_out = 0.0;
		r_z_out = r_z_in;
		// TODO
		// With exotic gackermanns, a basis change is to come here.
	}
	else {
		// Just aliases.
		const double x = this->virtual_center_x;
		const double y = this->virtual_center_y;
		
		// TODO. v_lon will not have to be parallel to v_x.
		double v_lon = v_x_in;
		double v_lat = 0.0;
		double v_rot = r_z_in;
		// TODO
		// Now here's a fundamental problem. Say the center of Ackermann
		// is 3 meters behind the robot. The intuitive thing to do in
		// this case would be transforming this v_lat request into a
		// rotation. The exact transformation would depend on where the
		// center is.
		// For now, I ignore v_lat.
	
		if (abs(v_lon / v_rot) < this->r_min) {
			// Trying to turn with a radius smaller than the minimum.
			// We need to clip v_rot

			double max_s_rot = abs(v_lon / this->r_min);
			v_rot = max_s_rot * v_rot / abs(v_rot);
		}
	
		v_x_out = v_lon;
		v_y_out = -v_rot * x;
		r_z_out = v_rot;
	}	
}
