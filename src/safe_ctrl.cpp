#include "multi_drone_safe_ctrl/safe_ctrl.h"
namespace Safe_Ctrl
{
    safe_ctrl::safe_ctrl(ros::NodeHandle& nh, ros::NodeHandle& pnh) :nh_(nh)
	{
        pnh.param("debug", debug, true);
        pnh.param("Vmax", Vmax, 10.0);
        pnh.param("l", l, 0.2);
        pnh.param("rm", rm, 0.4);
        pnh.param("ra", ra, 1.0);
        pnh.param("rh", rh, 10.0);
        pnh.param("e", e, 0.01);
        pnh.param("kx", kx, 1.0);
        pnh.param("ky", ky, 1.0);
        pnh.param("k_m", k_m,1.0);
        pnh.param("x_b", x_b, 20.0);
        pnh.param("y_b", y_b, 20.0);
        pnh.param("Hz", Hz, 20.0);
        pnh.param("gain_cmd",gain_cmd,0.5);
        ros::Rate loopRate(Hz);
        x_h = x_b / 2.0;
        y_h = y_b / 2.0;

        geo_half_x = x_h;
        geo_half_y = y_h;

        geo_x_min = 0.0;
        geo_y_min = 0.0;

        pub_vel_cmd1 = nh_.advertise<geometry_msgs::Twist>("cmd_vel_add_geo", 1);

        get_cmd_input1 = nh_.subscribe("cmd_input_origin", 1, &safe_ctrl::CmdInputCallBack1, this);

        get_navdata1 = nh_.subscribe("navdata_uav", 2, &safe_ctrl::NavdataCallBack1, this);

        debug_pub =nh_.advertise<multi_drone_safe_ctrl::DebugData>("debug_topic", 1);

        while(ros::ok())
        {

            geo_cmd1.x = cmd_geo_x1.x + cmd_geo_y1.x;
            geo_cmd1.y = cmd_geo_x1.y + cmd_geo_y1.y;
            geo_cmd1.z = cmd_geo_x1.z + cmd_geo_y1.z;

            debug_msg.header.frame_id="debug";
            debug_msg.header.stamp=ros::Time::now();
            debug_msg.nav1 = navdata_msg1;
            debug_msg.in_cmd1 = input_cmd1;
            debug_msg.out_cmd_idea1 = output_cmd_idea1;
            debug_msg.cmd_geo1 = geo_cmd1;
            debug_msg.cmd_aviod1 = cmd_aviodance_1;
            safe_ctrl::sat_vel_vec(output_cmd_idea1, output_cmd_idea1, Vmax);

            output_cmd1.linear.x = output_cmd_idea1.x;
            output_cmd1.linear.y = output_cmd_idea1.y;
            output_cmd1.linear.z = output_cmd_idea1.z;
            output_cmd1.angular = input_cmd1.angular;

            debug_msg.out_cmd1 = output_cmd1;

            debug_pub.publish(debug_msg);
            if (debug)
            {
                cout << "output_cmd 1:" << endl;
                cout << output_cmd1 << endl;

            }

            pub_vel_cmd1.publish(output_cmd1);
            ros::spinOnce();
            loopRate.sleep();
        }
	}

    void safe_ctrl::CmdInputCallBack1(const geometry_msgs::Twist& msg)
    { 
            input_cmd1 = msg;
            if (debug)
                cout << "input_cmd:" << endl << msg << endl;
    }

    void safe_ctrl::NavdataCallBack1(const nav_msgs::Odometry &msg)
    {

            navdata_msg1 = msg;

            pos_current1 = navdata_msg1.pose.pose.position;
            vel_current1.x = navdata_msg1.twist.twist.linear.x;
            vel_current1.y = navdata_msg1.twist.twist.linear.y;
            vel_current1.z = navdata_msg1.twist.twist.linear.z;

            safe_ctrl::nav_vec(pos_nav_vec1, pos_current1, vel_current1);
            if (debug)
            {
                cout << "NavisNewMsg:New Update!" << endl;
                cout << "pos_current:" << endl;
                cout << pos_current1 << endl;
            }

            safe_ctrl::dis_p2l_vec(dis_geo_x1, pos_nav_vec1, geo_half_x, 1);
            safe_ctrl::dis_p2l_vec(dis_geo_y1, pos_nav_vec1, geo_half_y, 0);
            safe_ctrl::Vb_cmd(cmd_geo_x1, dis_geo_x1, kx, 1);
            safe_ctrl::Vb_cmd(cmd_geo_y1, dis_geo_y1, ky, 0);

    }

    void safe_ctrl::nav_vec(geometry_msgs::Point& output, geometry_msgs::Point& pos,geometry_msgs::Point& vel)
    {
        output.x = pos.x + l*vel.x;
        output.y = pos.y + l*vel.y;
        output.z = pos.z + l*vel.z;

    }

    void safe_ctrl::dis_vec(geometry_msgs::Point& output, geometry_msgs::Point& pos1,geometry_msgs::Point& pos2)
    {
        output.x = pos1.x - pos2.x;
        output.y = pos1.y - pos2.y;
        output.z = pos1.z - pos2.z;
    }

    void safe_ctrl::dis_p2l_vec(double& output, geometry_msgs::Point& pos1,double& line,int type)
    {
        if (type == 1)
        {
            output = pos1.x - line;
        }
        if (type == 0)
        {
            output = pos1.y - line;
        }

    }

    void safe_ctrl::Vm_cmd(geometry_msgs::Point &output_vel, geometry_msgs::Point &nav_vec1, geometry_msgs::Point &nav_vec2)
    {
        geometry_msgs::Point dis_vec;
        double dis;
        double rm_m2, rm_d2;

        safe_ctrl::dis_vec(dis_vec, nav_vec1, nav_vec2);
        dis = safe_ctrl::norm_vec(dis_vec);
        rm_m2 = 2.0*rm;
        rm_d2 = dis/rm_m2;
        double f = k_m*(safe_ctrl::sig(dis,rm_m2,ra));
        double f_grad = k_m*(safe_ctrl::grad_sig(dis,rm_m2,ra));
        double g = (1 + e) * dis - 2 * rm * (safe_ctrl::s_m(rm_d2));
        double g_grad = (1 + e) - (safe_ctrl::grad_s_m(rm_d2));
        double V = (f_grad*g-f*g_grad)/(g*g);

        output_vel.x = dis_vec.x / dis *V;
        output_vel.y = dis_vec.y / dis * V;
        //output_vel.z = dis_vec.z / dis * V;
        output_vel.z = 0.0;
    }

    void safe_ctrl::Vb_cmd(geometry_msgs::Point& output_vel, double& dis, double& k,int type)
    {
        int sgn;
        if(dis >= 0.0)
            sgn = -1;
        else
            sgn = 1;
        dis = abs(dis);
        double dis_h = rh - dis;
        double f = k*(safe_ctrl::sig(dis_h,rm,ra));
        double f_grad = - k*(safe_ctrl::grad_sig(dis_h,rm,ra));
        double tmp = (rh-rm)/(dis+e);
        double g = (rh-rm) - dis*(safe_ctrl::s_m(tmp));
        double g_grad = -(safe_ctrl::s_m(tmp)) + dis*(safe_ctrl::grad_s_m(tmp))*(rh-rm)/((dis+e)*(dis+e)) ;
        double V_= f/g;
        double V = (f_grad*g-f*g_grad)/(g*g);

        if(type == 1)
        {
            output_vel.x = sgn*V;
            output_vel.y = 0.0;
            output_vel.z = 0.0;
        }
        if(type == 0)
        {
            output_vel.x = 0.0;
            output_vel.y = sgn*V;
            output_vel.z = 0.0;
        }


    }

    double safe_ctrl::sig(double& x, double& d1,double& d2)
    {
//        double A = -2/((d1-d2)*(d1-d2)*(d1-d2));
//        double B =  3*(d1+d2)/((d1-d2)*(d1-d2)*(d1-d2));
//        double C = -6*d1*d2/((d1-d2)*(d1-d2)*(d1-d2));
//        double D = d2*d2*(3*d1-d2)/((d1-d2)*(d1-d2)*(d1-d2));

        if (x<d1)
            return 1.0;
        else if (x>d2)
            return 0.0;
        else
//            return A*x*x*x+B*x*x+C*x+D;
            return (d2-x)/(d2-d1);
    } 

    double safe_ctrl::grad_sig(double& x, double& d1,double& d2)
    {
//        double A = -2/((d1-d2)*(d1-d2)*(d1-d2));
//        double B =  3*(d1+d2)/((d1-d2)*(d1-d2)*(d1-d2));
//        double C = -6*d1*d2/((d1-d2)*(d1-d2)*(d1-d2));
//        double D = d2*d2*(3*d1-d2)/((d1-d2)*(d1-d2)*(d1-d2));

        if (x<d1)
            return 0.0;
        else if (x>d2)
            return 0.0;
        else
//            return 3*A*x*x+2*B*x+C;
              return -1.0/(d2-d1);
    }

    double safe_ctrl::s_m(double& x)
    {
        if (x<1.0)
            return x;
        else
            return 1.0;
    }

    double safe_ctrl::grad_s_m(double& x)
    {

        if (x<1.0)
            return 1.0;
        else
            return 0.0;
    }

    double safe_ctrl::norm_vec(geometry_msgs::Point& input)
    {
        double x = input.x;
        double y = input.y;
        return sqrt(x*x+y*y);
    }

    void safe_ctrl::sat_vel_vec(geometry_msgs::Point& output, geometry_msgs::Point& input, double& Vmax)
    {
        double norm_vec =  safe_ctrl::norm_vec(input);
        if (norm_vec < Vmax)
            output = input;
        else
        {
            output.x = input.x / norm_vec * Vmax;
            output.y = input.y / norm_vec * Vmax;
            //output.z = input.z / norm_vec * Vmax;
            output.z = input.z;
        }

    }

}
