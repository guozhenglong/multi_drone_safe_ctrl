#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <stdlib.h>

//ROS Images
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <aruco_navigation/Navdata_aruco.h>

using namespace std;
#define PI 3.14159

namespace Safe_Ctrl{
class safe_ctrl{
    public:
        safe_ctrl(ros::NodeHandle& nh,ros::NodeHandle& pnh);
        double sig(double& x, double& d1,double& d2); 
        double grad_sig(double& x, double& d1,double& d2);
        double s_m(double& x, double& rs);
        double grad_s_m(double& x, double& rs);

        double norm_vec(geometry_msgs::Point& input);

        void nav_vec(geometry_msgs::Point& output, geometry_msgs::Point& pos,geometry_msgs::Point& vel);
        void dis_vec(geometry_msgs::Point& output, geometry_msgs::Point& pos1,geometry_msgs::Point& pos2);
        void dis_p2l_vec(double& output, geometry_msgs::Point& pos1,double& line,int type);
        void sat_vel_vec(geometry_msgs::Point& output, geometry_msgs::Point& input, double& Vmax);

        //void Vm_cmd(geometry_msgs::Point& output_vel, geometry_msgs::Point& dis_vec);
        void Vb_cmd(geometry_msgs::Point& output_vel, double& dis, double& k,int type);
        //void Vd_cmd(geometry_msgs::Point& output_vel, geometry_msgs::Point& dis_vec);

        void NavdataCallBack(const aruco_navigation::Navdata_aruco& msg);
        void CmdInputCallBack(const geometry_msgs::Twist& msg);
        
    private:
        ros::NodeHandle nh_;

        ros::Subscriber get_cmd_input;
        ros::Subscriber get_navdata;
        ros::Publisher pub_vel_cmd;
        
        geometry_msgs::Twist input_cmd;
        geometry_msgs::Twist output_cmd;
        aruco_navigation::Navdata_aruco navdata_msg;
        geometry_msgs::Point pos_current;
        geometry_msgs::Point vel_current;
		geometry_msgs::Vector3 euler;

        // bool empty_cmd;
        bool NavisNewMsg;
        double Hz;
        double Vmax;
        geometry_msgs::Point output_cmd_idea;

        geometry_msgs::Point pos_nav_vec;
        double l ;

        double geo_half_x;
        double geo_half_y;	
        double dis_geo_x;
        double dis_geo_y;

        geometry_msgs::Point cmd_geo_x;
        geometry_msgs::Point cmd_geo_y;

		double rm ;
		double ra ;
		double rh ;
		double e ;
		double rs ;
        double kx ;
        double ky;
        double geo_x_min;
        double geo_y_min;

  	
		double x_b ;
		double y_b ;
		double x_h ;
		double y_h ;

		bool debug;
        double gain_cmd;
   
};
}
