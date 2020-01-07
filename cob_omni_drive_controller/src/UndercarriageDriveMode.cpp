#include "UndercarriageDriveMode.h"

#include<stdlib.h>

UndercarriageDriveMode::UndercarriageDriveMode(ros::NodeHandle &root_nh, ros::NodeHandle& controller_nh) {
	this->mode = DIFFERENTIAL;

    this->virtual_center_x = 0.;
    this->virtual_center_y = 0.;    

	this->srv = root_nh.advertiseService("ucdm_conf", &UndercarriageDriveMode::config, this);
}

void UndercarriageDriveMode::setMode(Mode new_mode) {
	this->mode = new_mode;

    // So config is supposed to call this method. Why not directly in config?
}

void UndercarriageDriveMode::setCenter(double x, double y) {
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

	this->r_min = 0.4;
}

void UndercarriageDriveMode::set_min_turning_radius(double r_min) {
}

double absolute_value__todo_y_u_no_working(double x) {
    if (x < 0.0) return -x;
	return x;
}

void UndercarriageDriveMode::apply(double v_x_in, double v_y_in, double r_z_in, 
			 double& v_x_out, double& v_y_out, double& r_z_out) {
	ROS_INFO("mode: %d", (int)this->mode);
	ROS_INFO("apply on %lf %lf %lf", v_x_in, v_y_in, r_z_in);
	if (this->mode == OMNIDIRECTIONAL) {
	    ROS_INFO("OMNI. Doing nothing.");
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
	else if (this->mode == ACKERMANN) {
		// TODO. v_lon will not have to be parallel to v_x.
		double v_lat = 0.0;
		double v_rot = r_z_in;
		// TODO
		// Now here's a fundamental problem. Say the center of Ackermann
		// is 3 meters behind the robot. The intuitive thing to do in
		// this case would be transforming this v_lat request into a
		// rotation. The exact transformation would depend on where the
		// center is.
		// For now, I ignore v_lat.
	
		ROS_INFO("abs vx/vr %lf", absolute_value__todo_y_u_no_working(v_x_in / v_rot));
		if (absolute_value__todo_y_u_no_working(v_x_in / v_rot) < this->r_min) {
			// Trying to turn with a radius smaller than the minimum.
			// We need to clip v_rot

			double max_v_rot = absolute_value__todo_y_u_no_working(v_x_in / this->r_min);
			v_rot = max_v_rot * v_rot / absolute_value__todo_y_u_no_working(v_rot);
			ROS_INFO("Tried to turn with radius smaller than allowed. New v_rot: %lf", v_rot);
		}
	
		v_x_out = v_x_in;
		v_y_out = -v_rot * this->virtual_center_x;
		r_z_out = v_rot;
	}	
	else { // if (this->mode == DIFFERENTIAL) { TODO clean up
		v_x_out = v_x_in;
		v_y_out = - r_z_in * this->virtual_center_x;
		r_z_out = r_z_in;
	}	
	ROS_INFO("Done. %lf %lf %lf", v_x_out, v_y_out, r_z_out);
}

bool UndercarriageDriveMode::config(cob_omni_drive_controller::ucdm_cmd::Request& req,
									cob_omni_drive_controller::ucdm_cmd::Response& res) {
	switch (res.mode = req.mode) {
		case 0: this->mode = OMNIDIRECTIONAL; break;
		case 1: this->mode = ACKERMANN; break;
		case 2: this->mode = DIFFERENTIAL; break;
		default: 
			ROS_INFO("UCDM config: Invalid mode.");
			res.mode = -1;
			break;
	}

	res.virtual_center_x = this->virtual_center_x = req.virtual_center_x;
	res.virtual_center_y = this->virtual_center_y = req.virtual_center_y;
	res.direction = req.direction;
	ROS_INFO("UCDM config done.");
	return true;
}
