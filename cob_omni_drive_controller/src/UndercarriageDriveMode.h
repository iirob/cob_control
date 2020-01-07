#include <ros/ros.h>
#include "cob_omni_drive_controller/ucdm_cmd.h"

class UndercarriageDriveMode {

public:
    UndercarriageDriveMode(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    // TODO doc
    enum Mode { OMNIDIRECTIONAL, ACKERMANN, DIFFERENTIAL };
    void setMode(Mode new_mode);
    void setCenter(double x, double y);
    void set_min_turning_radius(double r_min);
    void apply(double v_x_in, double v_y_in, double r_z_in, 
               double& v_x_out, double& v_y_out, double& r_z_out);
    bool config(cob_omni_drive_controller::ucdm_cmd::Request& req,
                cob_omni_drive_controller::ucdm_cmd::Response& res);

private:
    // Current drive mode.
    Mode mode;

    // Coordinates of center of rotation in meters.
    double virtual_center_x;
    double virtual_center_y;    

    // Minimum turning radius. Only relevant in Ackermann mode.
    double r_min;

    ros::ServiceServer srv;

    // TODO Ackermann direction! I.e. Whatever is in the ucdm_cmd.msg.

};
