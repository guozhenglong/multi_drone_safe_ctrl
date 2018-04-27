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
        pnh.param("rs", rs, 0.01);
        pnh.param("kx", kx, 1.0);
        pnh.param("ky", ky, 1.0);
        pnh.param("x_b", x_b, 20.0);
        pnh.param("y_b", y_b, 20.0);
        pnh.param("Hz", Hz, 20.0);
        pnh.param("gain_cmd",gain_cmd,0.5);
        ros::Rate loopRate(Hz);
        x_h = x_b / 2.0;
        y_h = y_b/2.0;

        geo_half_x = x_h;
        geo_half_y = y_h;

        geo_x_min = 0.0;
        geo_y_min = 0.0;

        // double dist_test = -10.0;
        // double sig_test = 0.0;
        // geometry_msgs::Point cmd_geo_test;
        // for(int i=0;i<40;i++)
        // {
        //     safe_ctrl::Vb_cmd(cmd_geo_test, dist_test, kx,1);
        //     //sig_test = safe_ctrl::grad_s_m(dist_test,rs);
        //     dist_test = dist_test + 0.5;
        //     cout<<"Seq=:"<<i<<endl;
        //     cout<<"R=:"<<endl<<cmd_geo_test<<endl;
        // }

		pub_vel_cmd = nh_.advertise<geometry_msgs::Twist>("/uav_cmd_vel", 1);
        get_cmd_input = nh_.subscribe("/cmd_input", 1, &safe_ctrl::CmdInputCallBack, this);
        get_navdata = nh_.subscribe("/navdata_uav", 2, &safe_ctrl::NavdataCallBack, this);

        while(ros::ok())
        {    
            if (NavisNewMsg)
            {
                pos_current = navdata_msg.pose.position;
                vel_current = navdata_msg.velocity;
                euler = navdata_msg.euler_angle;
                safe_ctrl::nav_vec(pos_nav_vec, pos_current, vel_current);
                if (debug)
                {
                    cout << "NavisNewMsg:New Update!" << endl;
                    cout << "pos_current:" << endl;
                    cout << pos_current << endl;
                }
                safe_ctrl::dis_p2l_vec(dis_geo_x, pos_nav_vec, geo_half_x, 1);
                safe_ctrl::dis_p2l_vec(dis_geo_y, pos_nav_vec, geo_half_y, 0);
                safe_ctrl::Vb_cmd(cmd_geo_x, dis_geo_x, kx, 1);
                safe_ctrl::Vb_cmd(cmd_geo_y, dis_geo_y, ky, 0);

                output_cmd_idea.x = input_cmd.linear.x + cmd_geo_x.x + cmd_geo_y.x;
                output_cmd_idea.y = input_cmd.linear.y - (cmd_geo_x.y + cmd_geo_y.y); // linear.y<0, fly to right
                output_cmd_idea.z = input_cmd.linear.z + cmd_geo_x.z + cmd_geo_y.z;
            }
            else
            {
                if (debug)
                {
                    cout << "No Update!" << endl;
                }   
                output_cmd_idea.x = input_cmd.linear.x ;
                output_cmd_idea.y = input_cmd.linear.y ; 
                output_cmd_idea.z = input_cmd.linear.z ;
            }
            
            safe_ctrl::sat_vel_vec(output_cmd_idea, output_cmd_idea, Vmax);
            output_cmd.linear.x = output_cmd_idea.x;
            output_cmd.linear.y = output_cmd_idea.y;
            output_cmd.linear.z = output_cmd_idea.z;
            output_cmd.angular = input_cmd.angular;
            if (debug)
            {
                cout << "output_cmd:" << endl;
                cout << output_cmd << endl;
            }
            pub_vel_cmd.publish(output_cmd);
            ros::spinOnce();
            loopRate.sleep();
        }
	}

    void safe_ctrl::CmdInputCallBack(const geometry_msgs::Twist& msg)
    { 
            input_cmd = msg;
            if (debug)
                cout << "input_cmd:" << endl << msg << endl;
    }

	void safe_ctrl::NavdataCallBack(const aruco_navigation::Navdata_aruco& msg)
	{
        if (msg.IsNewNav)
        {
            navdata_msg = msg;
            NavisNewMsg = true;
        }
        else
        {
            NavisNewMsg = false;
        } 
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

    // void safe_ctrl::Vm_cmd(geometry_msgs::Point& output_vel, double& dis, double& k,int type)
    // {
    //     double f = k*(safe_ctrl::sig(dis,rm,ra));
    //     double f_grad = k*(safe_ctrl::grad_sig(dis,rm,ra));
    //     double g = (1+e)*dis-2*rm*(safe_ctrl::s_m(dis,rs));
    //     double g_grad = (1+e)-2*rm*(safe_ctrl::grad_s_m(dis,rs));
    //     double V = (f_grad*g-f*g_grad)/(g*g);
    //     cout<<"f=:"<<f<<endl;
    //     cout<<"f_grad=:"<<f_grad<<endl;
    //     cout<<"g=:"<<g<<endl;
    //     cout<<"g_grad=:"<<g_grad<<endl;
    //     cout<<"V=:"<<V<<endl;
    //     if(type == 1)
    //     {
    //         output_vel.x = V;
    //         output_vel.y = 0.0;
    //         output_vel.z = 0.0;
    //     }
    //     if(type == 0)
    //     {
    //         output_vel.x = 0.0;
    //         output_vel.y = V;
    //         output_vel.z = 0.0;
    //     }


    // }

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
        double g = (rh-rm) - dis*(safe_ctrl::s_m(tmp,rs));
        double g_grad = -(safe_ctrl::s_m(tmp,rs)) + dis*(safe_ctrl::grad_s_m(tmp,rs))*(rh-rm)/((dis+e)*(dis+e)) ; 
        double V_= f/g;
        double V = (f_grad*g-f*g_grad)/(g*g);
        // cout<<"dis=:"<<dis<<endl;
        // cout<<"dis_h=:"<<dis_h<<endl; 
        // cout<<"f=:"<<f<<endl;
        // cout<<"f_grad=:"<<f_grad<<endl;
        // cout<<"g=:"<<g<<endl;
        // cout<<"g_grad=:"<<g_grad<<endl;
        // cout<<"V_=:"<<V_<<endl;
        // cout<<"V=:"<<V<<endl;
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
        double A = -2/((d1-d2)*(d1-d2)*(d1-d2));
        double B =  3*(d1+d2)/((d1-d2)*(d1-d2)*(d1-d2));
        double C = -6*d1*d2/((d1-d2)*(d1-d2)*(d1-d2));
        double D = d2*d2*(3*d1-d2)/((d1-d2)*(d1-d2)*(d1-d2));
        // cout<<"x=:"<<x<<"   d1=:"<<d1<<"    d2=:"<<d2<<endl;
        // cout<<"A=:"<<A<<"   B=:"<<B<<"    C=:"<<C<<"    D=:"<<D<<endl;
        if (x<d1)
            return 1.0;
        else if (x>d2)
            return 0.0;
        else
            return A*x*x*x+B*x*x+C*x+D;
    } 

    double safe_ctrl::grad_sig(double& x, double& d1,double& d2)
    {
        double A = -2/((d1-d2)*(d1-d2)*(d1-d2));
        double B =  3*(d1+d2)/((d1-d2)*(d1-d2)*(d1-d2));
        double C = -6*d1*d2/((d1-d2)*(d1-d2)*(d1-d2));
        double D = d2*d2*(3*d1-d2)/((d1-d2)*(d1-d2)*(d1-d2));
        // cout<<"x=:"<<x<<"   d1=:"<<d1<<"    d2=:"<<d2<<endl;
        // cout<<"A=:"<<A<<"   B=:"<<B<<"    C=:"<<C<<"    D=:"<<D<<endl;
        if (x<d1)
            return 0.0;
        else if (x>d2)
            return 0.0;
        else
            return 3*A*x*x+2*B*x+C;
    }

    double safe_ctrl::s_m(double& x, double& rs)
    {
        double x2 = 1+rs*(1/tan(67.5/180*PI));
        double x1 = x2 - rs*sin(45/180*PI);
        // cout<<"x=:"<<x<<"   rs=:"<<rs<<endl;
        // cout<<"x1=:"<<x1<<"   x2=:"<<x2<<endl;
        if (x>=0 && x<x1)
            return x;
        else if (x >x2)
            return 1.0;
        else
            return (1-rs)+sqrt(rs*rs-(x-x2)*(x-x2));
    }

    double safe_ctrl::grad_s_m(double& x, double& rs)
    {
        double x2 = 1+rs*(1/tan(67.5/180*PI));
        double x1 = x2 - rs*sin(45/180*PI);
        // cout<<"x=:"<<x<<"   rs=:"<<rs<<endl;
        // cout<<"x1=:"<<x1<<"   x2=:"<<x2<<endl;
        if (x>=0 && x<x1)
            return 1.0;
        else if (x >x2)
            return 0.0;
        else
            return (x2-x)/sqrt(rs*rs-(x-x2)*(x-x2));
    }


    double safe_ctrl::norm_vec(geometry_msgs::Point& input)
    {
        double x = input.x;
        double y = input.y;
        double z = input.z;
        return sqrt(x*x+y*y+z*z);
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
