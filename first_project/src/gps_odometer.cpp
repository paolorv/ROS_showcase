#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <cmath>

class Gps_Odometry{
    public:
    Gps_Odometry(){
        //init parameters
        //previous state
        this->x0 = 0;
        this->y0 = 0;
        this->z0 = 0;
        //current_state
        this->x1 = 0;
        this->y1 = 0;
        this->z1 = 0;
        //
        this->previous_x_enu = 0;
        this->previous_y_enu = 0;
        this->x_enu = 0;
        this->y_enu = 0;
        this->z_enu = 0;
        this->private_nh = ros::NodeHandle("~");

        //publish and subscribe
        sub = nodeHandle.subscribe("/swiftnav/front/gps_pose",1000,&Gps_Odometry::odom_callback,this);
        pub = nodeHandle.advertise<nav_msgs::Odometry>("/gps_odom",1000);

        this->nodeHandle.param("starting_gps_theta", this->theta, 1.474981);
        this->private_nh.param("motion_threshold", this->motion_threshold, 0.1);

    }

    private:
    //node Handle Object
    ros::NodeHandle nodeHandle;
    ros::NodeHandle private_nh;

    //tf
    tf::TransformBroadcaster br;
    tf::Transform tr;
    tf::Quaternion q;
    nav_msgs::Odometry odom_msg;

    //ros publisher and subscriber
    ros::Subscriber sub;
    ros::Publisher pub;

    //time of the last computed odometry
    ros::Time last_time;

    //parameters of node
    bool initialized = false;

    //origin
    double lat_r;
    double lon_r;
    double alt_r;

    //state origin
    double x0,y0,z0;

    //this state ENU
    double x_enu,y_enu,z_enu;

    //previous state ENU
    double previous_x_enu,previous_y_enu;

    //this state ECEF
    double x1;
    double y1;
    double z1;
    double theta;

    //threshold
    double motion_threshold;

    //computes gps to ecef transform
    void GPS_to_ECEF(double lat_deg,double lon_deg,double alt_deg, double &x, double&y, double &z){

        double a = 6378137.0;             // WGS84 major axis
        double f = 1 / 298.257223563;     // WGS84 flattening
        double e2 = f * (2 - f);

        double lat = lat_deg * M_PI / 180.0;
        double lon = lon_deg * M_PI / 180.0;


        double sin_lat = sin(lat);
        double cos_lat = cos(lat);
        double sin_lon = sin(lon);
        double cos_lon = cos(lon);

        double N = a / sqrt(1 - e2 * sin_lat * sin_lat);

        x = (N + alt_deg) * cos_lat * cos_lon;
        y = (N + alt_deg) * cos_lat * sin_lon;
        z = ((1 - e2) * N + alt_deg) * sin_lat;
       
    }

    void odom_callback(const sensor_msgs::NavSatFix::ConstPtr& msg){


        //initialize reference coordinates
        if(!this->initialized){
          this->lat_r = msg->latitude;
          this->lon_r = msg->longitude;
          this->alt_r = msg->altitude;
          GPS_to_ECEF(this->lat_r, this->lon_r, this->alt_r, this->x0, this->y0, this->z0);
          this->initialized = true;
          return;
        }

        //get data from messages
        double lat_deg = msg->latitude;
        double lon_deg = msg->longitude;
        double alt_deg = msg->altitude;

        //setup current time 
        ros::Time current_time = msg->header.stamp;

        //initialize last time
        if (this->last_time.isZero()){
            this->last_time = current_time;
            return;
        }

        //compute delta t
        double dt = (current_time - this->last_time).toSec();

        //checks on dt
        if(dt <= 0 || dt > 1) return;

        //gps to ecef computation
        GPS_to_ECEF(lat_deg, lon_deg, alt_deg, this->x1, this->y1, this->z1);

        double lat = lat_r * M_PI / 180.0;
        double lon = lon_r * M_PI / 180.0;

        double dx = this->x1 - this->x0;
        double dy = this->y1 - this->y0;
        double dz = this->z1 - this->z0;

        //ecef to enu computation
        this->x_enu  = -sin(lon) * dx + cos(lon) * dy;
        this->y_enu = -sin(lat) * cos(lon) * dx - sin(lat) * sin(lon) * dy + cos(lat) * dz;
        this->z_enu = cos(lat) * cos(lon) * dx + cos(lat) * sin(lon) * dy + sin(lat) * dz;

        double dx_enu = this->x_enu - this->previous_x_enu;
        double dy_enu = this->y_enu - this->previous_y_enu;

        this->previous_x_enu = this->x_enu;
        this->previous_y_enu = this->y_enu;

        //speed and angular
        double speed = (dx_enu) / dt;
        double dtheta = this->theta - atan2(dy_enu, dx_enu);

        //checks on motion 
        if(std::hypot(dx_enu, dy_enu) > this->motion_threshold) this->theta = atan2(dy_enu, dx_enu);

        //Odometry message
        this->odom_msg.header.stamp = current_time;
        this->odom_msg.header.frame_id = "world";
        this->odom_msg.child_frame_id = "gps_odom";
        this->odom_msg.pose.pose.position.x = this->x_enu;
        this->odom_msg.pose.pose.position.y = this->y_enu;
        this->odom_msg.pose.pose.position.z = 0.0;
        this->odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->theta);
        this->odom_msg.twist.twist.linear.x = speed;
        this->odom_msg.twist.twist.angular.z = dtheta / dt;
        this->pub.publish(this->odom_msg);

         //Tf
        this->tr.setOrigin(tf::Vector3(this->x_enu, this->y_enu, 0));
        this->q.setRPY(0, 0, this->theta);
        this->tr.setRotation(q);
        this->br.sendTransform(tf::StampedTransform(tr, current_time, "world", "gps_odom"));

        //update last time 
        this->last_time = current_time;

    }


};


int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_odometer");
    Gps_Odometry gps_odom;
    ros::spin();
    return 0;
}