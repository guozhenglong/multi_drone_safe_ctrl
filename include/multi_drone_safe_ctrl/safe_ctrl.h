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
#include <nav_msgs/Odometry.h>
// #include <aruco_navigation/Navdata_aruco.h>
#include <multi_drone_safe_ctrl/DebugData.h>

using namespace std;
#define PI 3.14159

namespace Safe_Ctrl{
class safe_ctrl{
    public:
        safe_ctrl(ros::NodeHandle& nh,ros::NodeHandle& pnh);

		//void generate_cmd();
        double sig(double& x, double& d1,double& d2); 
        double grad_sig(double& x, double& d1,double& d2);
        double s_m(double& x);
        double grad_s_m(double& x);

        double norm_vec(geometry_msgs::Point& input);

        void nav_vec(geometry_msgs::Point& output, geometry_msgs::Point& pos,geometry_msgs::Point& vel);
        void dis_vec(geometry_msgs::Point& output, geometry_msgs::Point& pos1,geometry_msgs::Point& pos2);
        void dis_p2l_vec(double& output, geometry_msgs::Point& pos1,double& line,int type);
        void sat_vel_vec(geometry_msgs::Point& output, geometry_msgs::Point& input, double& Vmax);

        void Vm_cmd(geometry_msgs::Point &output_vel, geometry_msgs::Point &nav_vec1, geometry_msgs::Point &nav_vec2);
        void Vb_cmd(geometry_msgs::Point& output_vel, double& dis, double& k,int type);
        //void Vd_cmd(geometry_msgs::Point& output_vel, geometry_msgs::Point& dis_vec);

        // void NavdataCallBack1(const aruco_navigation::Navdata_aruco& msg);
        // void NavdataCallBack2(const aruco_navigation::Navdata_aruco &msg);
        void NavdataCallBack1(const nav_msgs::Odometry &msg);
        void NavdataCallBack2(const nav_msgs::Odometry &msg);

        void CmdInputCallBack1(const geometry_msgs::Twist& msg);
        void CmdInputCallBack2(const geometry_msgs::Twist &msg);

      private:
        ros::NodeHandle nh_;

        ros::Subscriber get_cmd_input1,get_cmd_input2;
        ros::Subscriber get_navdata1,get_navdata2;
        ros::Publisher pub_vel_cmd1, pub_vel_cmd2, debug_pub;
        
        geometry_msgs::Twist input_cmd1,input_cmd2;
        geometry_msgs::Twist output_cmd1,output_cmd2;
        geometry_msgs::Point geo_cmd1,geo_cmd2;

        // aruco_navigation::Navdata_aruco navdata_msg1, navdata_msg2;
        nav_msgs::Odometry navdata_msg1, navdata_msg2;
        geometry_msgs::Point pos_current1,pos_current2;
        geometry_msgs::Point vel_current1,vel_current2;
		geometry_msgs::Vector3 euler1,euler2;

        multi_drone_safe_ctrl::DebugData debug_msg;

        // bool empty_cmd;
        bool NavisNewMsg1,NavisNewMsg2;
        double Hz;
        double Vmax;
        geometry_msgs::Point output_cmd_idea1,output_cmd_idea2;

        geometry_msgs::Point pos_nav_vec1,pos_nav_vec2;
        double l ;

        double geo_half_x;
        double geo_half_y;	
        double dis_geo_x1,dis_geo_x2;
        double dis_geo_y1,dis_geo_y2;

        geometry_msgs::Point cmd_geo_x1,cmd_geo_x2;
        geometry_msgs::Point cmd_geo_y1,cmd_geo_y2;
        geometry_msgs::Point cmd_aviodance_1, cmd_aviodance_2;

        double rm ;
		double ra ;
		double rh ;
		double e ;
		double rs ;
        double kx ;
        double ky;
        double k_m;
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
