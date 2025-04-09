#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PointStamped.h"
#include <tf/transform_broadcaster.h>

class Odometer
{
private:

    //Node handle object
    ros::NodeHandle nh;

    //tf and Odometry messages
    tf::TransformBroadcaster br;
    tf::Transform tr;
    tf::Quaternion q;
    nav_msgs::Odometry odom_msg;

    //publisher and subscriber objects
    ros::Subscriber sub;
    ros::Publisher pub;

    //time of the last computed odometry
    ros::Time last_time;

    //position variables
    double x;
    double y;
    double theta;

    //parameters
    double wheelbase;
    int scale_factor;

    void odom_callback(const geometry_msgs::PointStamped::ConstPtr& msg){

        //read message data
        double speed = msg->point.y; //in km/h
        speed = speed / 3.6; //convert speed in m/s
        double steer = msg->point.x; //in grad
        steer = (steer/this->scale_factor) * M_PI / 180; //convert it in grad
        
        if((std::abs(steer) < 0.005) || (speed < 0.1)) steer = 0;
        //compute time slice 
        ros::Time current_time = msg->header.stamp;

        if (this->last_time.isZero()){
            this->last_time = current_time;
            return;
        }
        double dt = (current_time - this->last_time).toSec();

        if(dt <= 0 || dt > 1) return;

        //bicycle kinematics
        double omega = speed * tan(steer) / this->wheelbase;

        //Runge-Kutta integration
        double dx = speed * dt * cos(this->theta + (omega * dt / 2));
        double dy = speed * dt * sin(this->theta + (omega * dt / 2));
        double dtheta = dt * omega;

        this->x += dx;
        this->y += dy;
        this->theta += dtheta;
        
        //Odometry message
        this->odom_msg.header.stamp = current_time;
        this->odom_msg.header.frame_id = "world";
        this->odom_msg.child_frame_id = "odom";
        this->odom_msg.pose.pose.position.x = this->x;
        this->odom_msg.pose.pose.position.y = this->y;
        this->odom_msg.pose.pose.position.z = 0.0;
        this->odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->theta);

        this->odom_msg.twist.twist.linear.x = speed;
        this->odom_msg.twist.twist.angular.z = dtheta / dt;

        this->pub.publish(this->odom_msg);
        
        //Tf
        this->tr.setOrigin(tf::Vector3(this->x, this->y, 0));
        this->q.setRPY(0, 0, this->theta);
        this->tr.setRotation(q);

        this->br.sendTransform(tf::StampedTransform(tr, current_time, "world", "odom"));

        //Update last time 
        this->last_time = current_time;
    }

public:
    Odometer(){

        //init variables
        this->x = 0;
        this->y = 0;
        this->theta = 0;

        //Subscrive to /speedsteer and publis to /odom
        sub = nh.subscribe("speedsteer", 10, &Odometer::odom_callback, this);
        pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

        //load parameters
        nh.param("wheelbase", this->wheelbase, 1.765);
        nh.param("scale_factor", this->scale_factor, 32);
    }
   
};



int main(int argc, char **argv){


    ros::init(argc, argv, "odometer");
    Odometer odometer;
    ros::spin();
    return 0;
}
