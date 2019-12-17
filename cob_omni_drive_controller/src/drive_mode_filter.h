class DriveModeFilter {

public:
	// TODO doc
    enum Mode = { OMNIDIRECTIONAL, ACKERMANN, DIFFERENTIAL };
    void setMode(Mode new_mode);
	void setCenter(double x, double y);
	void set_min_turning_radius(double r_min);
	void filter (double v_x_in, double v_y_in, double r_z_in, 
	             double& v_x_out, double& v_y_out, double& r_z_out);

private:
    // Current drive mode.
    Mode mode;

    // Coordinates of center of rotation in meters.
    double virtual_center_x = 0.;
    double virtual_center_y = 0.;    

	// Minimum turning radius. Only relevant in Ackermann mode.
	double r_min;

	// TODO Ackermann direction!
};
