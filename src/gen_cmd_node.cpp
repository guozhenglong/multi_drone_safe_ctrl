#include "multi_drone_safe_ctrl/safe_ctrl.h"
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "gen_geo_cmd_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    Safe_Ctrl::safe_ctrl Gen_Geo_Cmd(nh, pnh);

    return 0;
}