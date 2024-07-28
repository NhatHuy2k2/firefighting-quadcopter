
#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>             //Local position, velocity and acceleration setpoint.
#include <mavros_msgs/AttitudeTarget.h>             //Attitude, angular rate and thrust setpoint.
#include <geometry_msgs/Point.h>                    //
#include <geometry_msgs/TwistStamped.h>             //Velocity data from FCU.
#include <geometry_msgs/PoseStamped.h>              //Local position from FCU.
#include <geometry_msgs/Vector3Stamped.h>           //Acceleration or force setpoint vector.

#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h> 
#include <mavros_msgs/CommandBool.h>                
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>

#include <cmath>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/GlobalPositionTarget.h>
mavros_msgs::State current_state;

//global var
bool start = 0;
int count_pi6 = 0;
int turn = 0;
int sum = 0;
double yaw_setpoint_gb = 0.0;
double yaw_setpoint = 0.0;
bool chan = 0;
int tien = 0;
double prev_error = 0.0;
double integral = 0.0;
double e_reset = 0.0;
double min_thrust_ = 0.3;
double max_thrust_ = 0.6;
double max_velocity_ = 3.0;
double max_angle_ = 0.2;
double U0 = 0.4;   //0.706/0.4
double yaw_test = 0.0;
double yaw2_test = 0.0;
double vel_yaw = 0.0;
double desired_local_position_yaw=0.0;
std_msgs::Int32 msg_motor;
double tam_x = 0.0;
double tam_y = 0.0;
double tam_yaw = 0.0;
geometry_msgs::Point desired_local_position_;
geometry_msgs::Vector3 current_velocity_;
geometry_msgs::Point current_position_;
geometry_msgs::Quaternion current_orientation_;
int x_0= 0.0;
int y_0= 0.0;
int t = 0.0;     
double a = 0.0;
double b = 0.0; 
uint16_t channel_4_value = 0;
uint16_t channel_5_value = 0;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::CommandBool arm_cmd;
int wait=1;
double a1, a2, a3;
int time_count = 0;


double yaw_angle = 0.0;

void compassHeadingCallback(const std_msgs::Float64ConstPtr& msg) 
{
    yaw_angle = msg->data;
    yaw_angle = 360+90-yaw_angle;
    yaw_angle = yaw_angle * M_PI / 180.0;
    //yaw_angle = yaw_angle*M_PI/180;
    //ROS_INFO("Yaw angle: %.2f degrees", yaw_angle * 180.0 / M_PI);
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
    {
        current_state = *msg;
    }

void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)                                 //ham thuc thi khi co gia tri moi gui len topic velocity_topic_. bien current_velocity la kieu geometry_msgs::Vector3
    {
        current_velocity_ = msg->twist.linear;
    } 

void positionCallback(const nav_msgs::Odometry::ConstPtr& msg)                                
    {
        current_position_ = msg->pose.pose.position;
        current_orientation_ = msg->pose.pose.orientation;
        yaw2_test = std::atan2(2*(current_orientation_.w*current_orientation_.z+current_orientation_.x*current_orientation_.y), 1-2*(current_orientation_.y*current_orientation_.y + current_orientation_.z*current_orientation_.z));
                                                                //con tro tro den truong position cua doi tuong pose
    }

double pid(double kp, double ki, double kd, double error)

    { 
        double output = kp * error + ki * integral + kd * (error - prev_error);
        integral += error;
        prev_error = error;


        return output;
    }
void detectionxCallback(const std_msgs::Int32::ConstPtr& msg)
{
    x_0 = msg->data;
    ROS_INFO("x0: %d", x_0);
}

void detectionyCallback(const std_msgs::Int32::ConstPtr& msg)
{
    y_0 = msg->data;
    ROS_INFO("y0: %d", y_0);

}

void rcInCallback(const mavros_msgs::RCIn::ConstPtr& msg)
{
    //RC
    channel_5_value = msg->channels[5];
    channel_4_value = msg->channels[4];


    

    //for (size_t i = 0; i < msg->channels.size(); ++i) {
        //ROS_INFO("Channel %zu: %u", i, msg->channels[i]);
    
    //}
}

double arcsin_formula(double x, double y, double z) {
    //double term1 = (cos(x) * sin(y) * cos(z) + sin(x) * sin(z)) * sin(z);
    //double term2 = (cos(x) * sin(y) * sin(z) - sin(x) * cos(z)) * cos(z);
    double gh = x*sin(z) - y*cos(z);
    gh = std::max(std::min(gh, 1.0), -1.0);
    //ROS_INFO("gh: %f", gh); 
    double result = asin(gh); //-
    //ROS_INFO("XXXXXXXXXXXXX: %f", result);
    return result;
 
}


double arcsin_formula_1(double x, double x1, double y, double z) {
    double term1 = (x * cos(z) + y * sin(z));
    //double term2 = (cos(x) * sin(y) * sin(z) - sin(x) * cos(z)) * sin(z);
    //double numerator = term1 + term2;  //+
    double denominator = cos(x1);
    double gh = term1/ denominator;
    gh = std::max(std::min(gh, 1.0), -1.0);
    double result = asin(gh);
    //ROS_INFO("YYYYYYYYYYYYY: %f", result); 
    //ROS_INFO("----------------------");
    return result;
     
}

void attitudeTargetCallback(const mavros_msgs::AttitudeTargetConstPtr& msg) {
    vel_yaw =  msg->body_rate.z;

}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
  
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber position_sub_ = nh.subscribe("/mavros/local_position/odom", 10, positionCallback);
    ros::Subscriber velocity_sub_ = nh.subscribe("/mavros/local_position/velocity_local", 10, velocityCallback);
    ros::Publisher attitude_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    ros::Publisher motor = nh.advertise<std_msgs::Int32>("motor_control", 10);
    ros::Subscriber attitude_sub_ = nh.subscribe("/mavros/setpoint_raw/target_attitude", 10, attitudeTargetCallback);
    ros::Subscriber camera_sub_x_ = nh.subscribe("/fire_detection_x", 10, detectionxCallback);
    ros::Subscriber camera_sub_y_ = nh.subscribe("/fire_detection_y", 10, detectionyCallback);
    ros::Subscriber compass_sub = nh.subscribe("/mavros/global_position/compass_hdg", 10, compassHeadingCallback);
    ros::Subscriber rc_sub = nh.subscribe("/mavros/rc/in", 10, rcInCallback);


    // Create service clients for setting mode and arming
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    ros::Rate rate(20.0); // Choose your desired control rate
    ros::Time last_request = ros::Time::now();
    ros::Time time_detect = ros::Time::now();
    ros::Time standby = ros::Time::now();
    ros::Time ready = ros::Time::now();
    ros::Time fake = ros::Time::now();
    ros::Time delay_2s = ros::Time::now();
    
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }






    while (ros::ok())
    {

        if (current_state.mode != "OFFBOARD" && channel_5_value > 1850)
        {

            //yaw_test = std::atan2(2*(current_orientation_.w*current_orientation_.z+current_orientation_.x*current_orientation_.y), 1-2*(current_orientation_.y*current_orientation_.y + current_orientation_.z*current_orientation_.z));
            //desired_local_position_yaw = yaw_angle;
            offb_set_mode.request.custom_mode = "OFFBOARD";
            start = 1;
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                if (ros::Time::now() - time_detect > ros::Duration(5.0))
                {
                    nh.param("setpoint_x", desired_local_position_.x, current_position_.x); //current_position_.x
                    nh.param("setpoint_y", desired_local_position_.y, current_position_.y); //current_position_.y
                    nh.param("setpoint_z", desired_local_position_.z, current_position_.z); //current_position_.z
                    nh.param("setpoint_yaw", desired_local_position_yaw , yaw_angle);
                    wait=1; 
                    msg_motor.data = 1;  //1 va LOW
                    //ROS_INFO("Offboard enabled");
                }
                else 
                {
                    last_request = ros::Time::now();
                }
            }
            time_detect = ros::Time::now();

        }

        else if (current_state.mode == "OFFBOARD")
        {
            time_detect = ros::Time::now();
        }

        else if (current_state.mode != "OFFBOARD")
        {
            start = 0;
        }

        

        else
        {
            
            if (!current_state.armed && channel_4_value>1500 )
            {
                arm_cmd.request.value = true;
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    //ROS_INFO("Vehicle armed");
                }

            }
        }

        if(start)
        {
            if (desired_local_position_yaw >= 2*M_PI)
            {
                desired_local_position_yaw = desired_local_position_yaw - 2*M_PI;
            }
            if (wait)
            {
                last_request = ros::Time::now();
                wait = 0;
                tien = 0;
                time_count = 0;
                count_pi6=0;
                fake = ros::Time::now();
                sum = 0;
                start = 0;
            }
            if (turn == 1 && x_0 == 0 && y_0 == 0 && time_count<4)
            {
            	if (sum <= 120)
            	{
            		sum+=1;
            		desired_local_position_yaw = desired_local_position_yaw + M_PI/360;
            		
            		last_request = ros::Time::now();
            	}
            	
            	else 
            	{
            		//count_pi6++;
            		sum = 1;
            		turn = 0;
            	}
            }
            //ROS_INFO("pi6: %d", count_pi6);
            //ROS_INFO("turn: %d", turn);
            	
		
            if (x_0 == 0 && y_0 == 0 && ros::Time::now() - last_request > ros::Duration(4.0) && count_pi6<7 && time_count<4)
            { 
                time_count = 0;
                max_angle_ = 0.03;
                //desired_local_position_yaw = desired_local_position_yaw + M_PI/3;
                count_pi6++;
                last_request = ros::Time::now();
                if (count_pi6 < 7)
                {
                	turn = 1;
                	sum = 1;
                }
                
            }


             if (x_0 == 0 && y_0 == 0 && count_pi6 == 7 && ros::Time::now() - last_request > ros::Duration(1.0) && time_count<4)
            {
                //if (desired_local_position_yaw != (M_PI/2) && desired_local_position_yaw != (3*M_PI/2))
                //{
                    //a = tan(desired_local_position_yaw);
                    //b = desired_local_position_.y - a*desired_local_position_.x;
                //desired_local_position_yaw = desired_local_position_yaw - 2*M_PI;
                time_count = 0;
                max_angle_ = 0.03;
                desired_local_position_.y = desired_local_position_.y+sin(desired_local_position_yaw)*1;
                desired_local_position_.x = desired_local_position_.x+cos(desired_local_position_yaw)*1;
                count_pi6=0;
                last_request = ros::Time::now();
                    
                //}
            }
            
            if (x_0 == 0 && y_0 == 0)
            {
            	fake = ros::Time::now();
            }

            else if (x_0 != 0 && y_0 != 0 && time_count<4)
            {

                max_angle_ = 0.03;
                if (ros::Time::now() - fake > ros::Duration(0.4))
                {
                	last_request = ros::Time::now();
                	turn = 0;
                	sum = 1;
                }
                else if (ros::Time::now() - fake < ros::Duration(0.4))
                {
                	delay_2s = ros::Time::now() - ros::Duration(2.0);
                }
                //1
                if (x_0 <= 310 || x_0 >= 410 ) //310, 410
                {
                    
                    time_count = 0;
                    if (x_0 <= 310 && x_0 >= 210 && ros::Time::now() - delay_2s > ros::Duration(0.0))         //qua phai
                    {	
                        delay_2s = ros::Time::now();          //2s camera xử lý
                        //msg_motor.data = 0;
                        desired_local_position_.x = desired_local_position_.x + sin(desired_local_position_yaw)*(desired_local_position_.z*0.032);
                        desired_local_position_.y = desired_local_position_.y - cos(desired_local_position_yaw)*(desired_local_position_.z*0.032);
                    }

                    else if (x_0 < 210 && ros::Time::now() - delay_2s > ros::Duration(0.0))   //qua phai
                    {
                        delay_2s = ros::Time::now();          //2s camera xử lý
                        desired_local_position_.x = desired_local_position_.x + sin(desired_local_position_yaw)*(desired_local_position_.z*0.032*2);
                        desired_local_position_.y = desired_local_position_.y - cos(desired_local_position_yaw)*(desired_local_position_.z*0.032*2);
                        //msg_motor.data = 1;
                    }

                    else if (x_0 >= 410 && x_0 <= 510 && ros::Time::now() - delay_2s > ros::Duration(0.0))   //qua trai
                    {
                        delay_2s = ros::Time::now();          //2s camera xử lý
                        desired_local_position_.x = desired_local_position_.x - sin(desired_local_position_yaw)*(desired_local_position_.z*0.032);
                        desired_local_position_.y = desired_local_position_.y + cos(desired_local_position_yaw)*(desired_local_position_.z*0.032);
                        //msg_motor.data = 1;
                    }

                    else if (x_0 > 510 && ros::Time::now() - delay_2s > ros::Duration(0.0))   //qua trai
                    {
                        delay_2s = ros::Time::now();          //2s camera xử lý
                        desired_local_position_.x = desired_local_position_.x - sin(desired_local_position_yaw)*(desired_local_position_.z*0.032*2);
                        desired_local_position_.y = desired_local_position_.y + cos(desired_local_position_yaw)*(desired_local_position_.z*0.032*2);
                        //msg_motor.data = 1;
                    }






                }
                //2
                
                else if (y_0 <= 190 || y_0 >= 290 )
                {
                    
                    time_count = 0;
                    if (y_0 <= 190 && y_0 >= 90 && ros::Time::now() - delay_2s > ros::Duration(0.0))         //di toi
                    {
                        delay_2s = ros::Time::now();
                        desired_local_position_.y = desired_local_position_.y+sin(desired_local_position_yaw)*(desired_local_position_.z*0.028);
                        desired_local_position_.x = desired_local_position_.x+cos(desired_local_position_yaw)*(desired_local_position_.z*0.028);
                    }

                    if (y_0 < 90 && ros::Time::now() - delay_2s > ros::Duration(0.0))         //di toi
                    {
                        delay_2s = ros::Time::now();
                        desired_local_position_.y = desired_local_position_.y+sin(desired_local_position_yaw)*(desired_local_position_.z*0.028*2);
                        desired_local_position_.x = desired_local_position_.x+cos(desired_local_position_yaw)*(desired_local_position_.z*0.028*2);
                    }

                    else if (y_0 >= 290 && y_0 <= 390 && ros::Time::now() - delay_2s > ros::Duration(0.0))         //di lui
                    {
                        delay_2s = ros::Time::now();
                        desired_local_position_.y = desired_local_position_.y-sin(desired_local_position_yaw)*(desired_local_position_.z*0.028);
                        desired_local_position_.x = desired_local_position_.x-cos(desired_local_position_yaw)*(desired_local_position_.z*0.028);
                    }

                    else if (y_0 > 390 && ros::Time::now() - delay_2s > ros::Duration(0.0))         //di lui
                    {
                        delay_2s = ros::Time::now();
                        desired_local_position_.y = desired_local_position_.y-sin(desired_local_position_yaw)*(desired_local_position_.z*0.028*2);
                        desired_local_position_.x = desired_local_position_.x-cos(desired_local_position_yaw)*(desired_local_position_.z*0.028*2);
                    }
                }
                
                else                         //chinh giua
                {
                    if (ros::Time::now() - standby > ros::Duration(2.5))
                    {
                        time_count = time_count+1;
                        if (time_count==1)
                        {
                            desired_local_position_.y = current_position_.y;
                            desired_local_position_.x = current_position_.x;
                            tam_x = desired_local_position_.x;
                            tam_y = desired_local_position_.y;
                            tam_yaw = desired_local_position_yaw;
                        }
                        standby = ros::Time::now();
                    }

                }

                
            }

            else if (time_count>=4) 
            {
                max_angle_ = 0.25;
                
                //lui 5 o
                if (tien == 0)
                {
                    max_angle_ = 0.03;
                    desired_local_position_.y = desired_local_position_.y-sin(desired_local_position_yaw)*2;
                    desired_local_position_.x = desired_local_position_.x-cos(desired_local_position_yaw)*2;
                    tien = 1;
                    ready = ros::Time::now();
                }

                else if ((tien == 1) && ros::Time::now() - ready > ros::Duration(4.0) )
                {
                    max_angle_ = 0.03;
                    desired_local_position_.y = desired_local_position_.y+sin(desired_local_position_yaw)*(round(desired_local_position_.z*10)/10+2);
                    //desired_local_position_.z*tan(45) = khoảng cách từ máy bay đến lửa
                    desired_local_position_.x = desired_local_position_.x+cos(desired_local_position_yaw)*(round(desired_local_position_.z*10)/10+2);
                    tien = 2;
                    ready = ros::Time::now();
                    
                }
                
                if (tien == 2 && ros::Time::now() - ready > ros::Duration(4.0))
                {
                    msg_motor.data = 0;
                }
                
                else if((tien == 2) && ros::Time::now() - ready > ros::Duration(5.0))
                {
                    desired_local_position_.y = tam_y;
                    desired_local_position_.x = tam_x;
                    max_angle_ = 0.03;
                    tien = 3;
                }
                


            }


            
            
                
            // Compute PID outputs for each axis
            // Calculate error+
            double error_local_pos_x = desired_local_position_.x - current_position_.x;                                       //current_position_.x doc gia tri toa do x hien tai 
            double error_local_pos_y = desired_local_position_.y - current_position_.y;
            double error_local_pos_z = desired_local_position_.z - current_position_.z;    
            //double error_local_pos_yaw = desired_local_position_yaw - yaw_angle;  
            //ROS_INFO("current_x: %f", current_position_.x);
            //ROS_INFO("current_y: %f", current_position_.y);
            //ROS_INFO("current_z: %f", current_position_.z);   
            //ROS_INFO("Received direction: %d", dir);

            // ROS_INFO("error_local_pos_x: %f", error_local_pos_x);
            // ROS_INFO("error_local_pos_y: %f", error_local_pos_y);
            // ROS_INFO("error_local_pos_z: %f", error_local_pos_z);

            double velocity_setpoint_local_x = pid(0.4, 0.0, 0.0, error_local_pos_x);     //position x   1.0                  //tinh toan pid cho vi tri setpoint x
            double velocity_setpoint_local_y = pid(0.4, 0.0, 0.0, error_local_pos_y);     //position y    0.4
            double velocity_setpoint_local_z = pid(0.4, 0.0, 0.0, error_local_pos_z);     //position z
            //double velocity_setpoint_local_yaw = pid(1.0, 0.0, 0.0, error_local_pos_yaw);
        

            
            velocity_setpoint_local_x = std::max(std::min(velocity_setpoint_local_x, max_velocity_), -max_velocity_);          // dam bao van toc trong khoang -max (max theo chieu nguoc lai) den max
            velocity_setpoint_local_y = std::max(std::min(velocity_setpoint_local_y, max_velocity_), -max_velocity_);
            velocity_setpoint_local_z = std::max(std::min(velocity_setpoint_local_z, 3.0), -1.0);                              //van toc bay cao theo truc z trong khoang -1 den 3, nghia la bay len nhanh nhat la 3, bay xuong nhanh nhat la 1
            //velocity_setpoint_local_yaw = std::max(std::min(velocity_setpoint_local_yaw, 2.0), -2.0);


            
            double error_velocity_local_x = velocity_setpoint_local_x - current_velocity_.x;                                   //current_velocity_.x la truong van toc theo truc x  cua doi tuong Twist.linear
            double error_velocity_local_y = velocity_setpoint_local_y - current_velocity_.y;
            //ROS_INFO("current_velocity_.y: %f", current_velocity_.y); 
            //double yaw_setpoint = std::atan2(error_local_pos_y, error_local_pos_x);
            double error_velocity_local_z = velocity_setpoint_local_z - current_velocity_.z;
            //double error_velocity_local_yaw = velocity_setpoint_local_yaw - vel_yaw;


            double pitch_setpoint_gb = pid(0.4, 0.0, 0.0, error_velocity_local_x);                                             //0.4 tinh toan pid cho goc quay quanh truc pitch (muc dich de drone nghien truoc hoac sau de bay toi hoac lui)  (ux)
            double roll_setpoint_gb = pid(0.4, 0.0, 0.0, error_velocity_local_y);                                            //tinh toan pid cho goc quay quanh truc roll (muc dich de drone nghien trai hoac phai de bay trai hoac phai)   (uy)
            //ROS_INFO("roll_setpoint_gb: %f", roll_setpoint_gb);
            double thrust_setpoint_gb = pid(0.3, 0.0, 0.0, error_velocity_local_z); 



            double roll_setpoint_gb_1 = arcsin_formula(pitch_setpoint_gb, roll_setpoint_gb , desired_local_position_yaw);
            double pitch_setpoint_gb_1 = arcsin_formula_1(pitch_setpoint_gb, roll_setpoint_gb_1, roll_setpoint_gb, desired_local_position_yaw);

            // ROS_INFO("thrust_setpoint_gb: %f", thrust_setpoint_gb);

            pitch_setpoint_gb_1 = std::max(std::min(pitch_setpoint_gb_1, max_angle_), -max_angle_);
            roll_setpoint_gb_1 = std::max(std::min( roll_setpoint_gb_1, max_angle_), -max_angle_);
            thrust_setpoint_gb = std::max(std::min(thrust_setpoint_gb + U0, max_thrust_), min_thrust_);

            //ROS_INFO("pitch_setpoint_gb_1: %f", pitch_setpoint_gb_1);  
            //ROS_INFO("--------------------------");  

            //yaw_setpoint_gb = std::max(std::min(yaw_setpoint_gb, 0.000001), -0.000001);




            // ROS_INFO("pitch_setpoint_gb: %f", pitch_setpoint_gb);
            // ROS_INFO("roll_setpoint_gb: %f", roll_setpoint_gb);
            // ROS_INFO("thrust_setpoint_after_minmax: %f", thrust_setpoint_gb);


            tf::Quaternion quat;   
                                                                                                    //convert roll, pitch, yaw to x, y, z, w
            //double yaw_setpoint = std::atan2(error_local_pos_y, error_local_pos_x);

            quat.setRPY(roll_setpoint_gb_1, pitch_setpoint_gb_1, desired_local_position_yaw);  //1200.0 quay 360
            //ROS_INFO("desired_local_position_yaw: %f", desired_local_position_yaw);

            //t+=100.0;
            double qx = quat.getX();
            double qy = quat.getY();
            double qz = quat.getZ();
            double qw = quat.getW();

            //ROS_INFO("-------------------------");
            
            mavros_msgs::AttitudeTarget attitude_msg;
            attitude_msg.type_mask = 4; 
            attitude_msg.header.stamp = ros::Time::now();
            attitude_msg.header.frame_id = "base_footprint";

            attitude_msg.orientation.x = qx;
            attitude_msg.orientation.y = qy;
            attitude_msg.orientation.z = qz;
            attitude_msg.orientation.w = qw;
            attitude_msg.thrust = thrust_setpoint_gb;

            //publish

            attitude_pub_.publish(attitude_msg);
            motor.publish(msg_motor);


        }
        ros::spinOnce();
        rate.sleep();
    
    }

    return 0;
}


